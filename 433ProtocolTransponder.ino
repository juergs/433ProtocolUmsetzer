/* *********************************************************

Purpose:
========
Transforn 42Bit Protocol to LaCrosse-Format

Initial: 20170114_juergs.

Credentials:
    http://helmpcb.com/software/reading-writing-structs-floats-and-other-objects-to-eeprom
    https://github.com/jacobsax/bitArray-library-for-Arduino
    https://github.com/RFD-FHEM/RFFHEM
protocol-definition: 
    https://docs.google.com/document/d/121ZH3omAZsdhFi3GSB-YdnasMjIQSGIcaS7QW6KsACA/mobilebasic?pli=1
    https://forum.arduino.cc/index.php?topic=136836.75
Antenna:
    http://www.mikrocontroller.net/articles/433_MHz_Funk%C3%BCbertragung
Helpers:
    http://graphics.stanford.edu/~seander/bithacks.html
    http://www.geeksforgeeks.org/memory-layout-of-c-program/
    http://www.geeksforgeeks.org/bit-fields-c/



Printf-Formats:
==============
%d %i	Decimal signed integer.
%o	    Octal integer.
%x %X	Hex integer.
%u	    Unsigned integer.
%c	    Character.
%s	    String.	siehe unten.
%f	    double
%e %E	double.
%g %G	double.
%p	    pointer.
%n	    Number of characters written by this printf. No argument expected.
%%	%.  No argument expected.

from: http://home.fhtw-berlin.de/~junghans/cref/FUNCTIONS/format.html

Basis + Decoding: Arduino sketch to receive NC7427 temperature/humidity RF sensor telegrams
Written by 'jurs' for German Arduino Forum: 
    https://github.com/jacobsax/bitArray-library-for-Arduino

    where: C:\Users\Jürgen\Documents\GitHub\42Bit_Protocol
********************************************************* */

#include <limits.h>


/*
typedef union 
{
    byte byteval[4];
    long longval;
} LongBytes;
LongBytes myVal;

    You can then access:
        myVal.longval = 4893723284;
    And also:
        myVal.byteval[i];
    where i is 0-3.
*/

//--- for debugging pupouses
#define DEBUG_1_PIN   11
#define DEBUG_2_PIN   10
#define DEBUG_3_PIN   9
#define DEBUG_4_PIN   8
#define DEBUG_5_PIN   7

#define RX433DATA 2       // receive pin for hardware interrupts
#define RX433INTERRUPT 0  // interrupt number for receive pin

#define NC7427_SYNC         8000    // length in µs of starting pulse
#define NC7427_SYNC_GLITCH  1100    // pulse length variation for ONE and ZERO pulses
#define NC7427_ONE          4000    // length in µs of ONE pulse
#define NC7427_ZERO         2000    // length in µs of ZERO pulse
#define NC7427_GLITCH        400    // pulse length variation for ONE and ZERO pulses, was 350
#define NC7427_MESSAGELEN     42    //36, number of bits in one message

union proto_union
{
    unsigned long long raw;
    struct
    {
        byte raw_byt[8];
    } b;
    struct proto_struct
    {
        unsigned long dummy : 22;
        byte lead   : 2;
        byte id     : 8;
        byte bat    : 2;
        byte chan   : 2;
        unsigned short temp : 12;
        byte hum    : 8;
        byte crc    : 8;
    } d;
} p;

typedef union
{
    unsigned long long raw;
    byte byteval[8];
} container;

//======================================================================
volatile boolean flagReady = false;

FILE serial_stdout;                             // needed for printf 

volatile unsigned long long raw;

volatile byte buf[NC7427_MESSAGELEN];

container   protocol;

//======================================================================
void    printResults(unsigned long value);
boolean crcValid(unsigned long value, byte checksum);
void    rx433Handler2();
int     freeRam();
void    printBits(size_t const size, void const * const ptr);
int     serial_putchar(char c, FILE* f);
unsigned short reverseBits(unsigned short number); 
void swapArrayBinPos(byte first, byte last);
//======================================================================

//-------------------------------------------------------------
void setup()
{
    //--- set stdout for printf to serial
    fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serial_stdout;

    //Serial.begin(57600);
    Serial.begin(115200);

    pinMode(RX433DATA, INPUT);
    attachInterrupt(RX433INTERRUPT, rx433Handler2, CHANGE);
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);

    pinMode(DEBUG_1_PIN, OUTPUT);
    digitalWrite(DEBUG_1_PIN, LOW);

    pinMode(DEBUG_2_PIN, OUTPUT);
    digitalWrite(DEBUG_2_PIN, LOW);

    pinMode(DEBUG_3_PIN, OUTPUT);
    digitalWrite(DEBUG_3_PIN, LOW);

    pinMode(DEBUG_4_PIN, OUTPUT);
    digitalWrite(DEBUG_4_PIN, LOW);

    pinMode(DEBUG_5_PIN, OUTPUT);
    digitalWrite(DEBUG_5_PIN, LOW);

    Serial.println("===============================================================");
    //Serial.println();
    Serial.print(F("\tFree RAM: ")); Serial.println(freeRam());
    //Serial.println();
    Serial.println("===============================================================");

   /* 64Bit sample:
    uint64_t pipe = 0x12345ABCD9LL;//lets assume  the data is 12345ABCD9 hexadecimal
    char buf[50];

    if(pipe > 0xFFFFFFFFLL) {
    sprintf(buf, "%lX%08lX", (unsigned long)(pipe>>32), (unsigned long)(pipe&0xFFFFFFFFULL));
    } else {
    sprintf(buf, "%lX", (unsigned long)pipe);
    }
    Serial.println( buf );
   */    
}
//-------------------------------------------------------------
void loop()
{    
    if (flagReady)
    {
        noInterrupts();

        //--- debug flag 
        digitalWrite(DEBUG_2_PIN, HIGH);
      
        printf("\n---------------------------------------------------------------------------------------\n");       
        printf("0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 \n");
        for (int i = 0; i < NC7427_MESSAGELEN; i++)
        {
            printf("%d ", buf[i]);
        }
        printf("\n---------------------------------------------------------------------------------------\n");
        printf("    01      23456789       01      23        456789012345       67890123       456789012345 \n");
        printf("---------------------------------------------------------------------------------------\n");
        

        p.raw = 0; 

        //buf[1] = 1; // test

        for (int i = 0; i < NC7427_MESSAGELEN; i++)
        {

            switch (i)
            {

                case 0:
                    printf("LD: ");
                    break;
                case 2:
                    printf(" |ID: ");
                    break;
                case 10:
                    printf(" |BAT: ");
                    break;
                case 12:
                    printf(" |CH: ");
                    break;
                case 14:
                    printf(" |TEMP: ");
                    break;
                case 26:
                    printf(" |HUM: ");
                    break;
                case 34:
                    printf(" |CRC: ");
                    break;
                default:
                    break; 
            }
            printf("%d", buf[i]);

        }
        
        //--- prepare temperatur bitpos swappen
        swapArrayBinPos(25, 14);
        
        //--- prepare hum bitpos swappen
        swapArrayBinPos(33, 26);

        printf("\n----swaped-----------------------------------------------------------------------------\n");
        for (int i = 0; i < NC7427_MESSAGELEN; i++)
        {

            switch (i)
            {

            case 0:
                printf("LD: ");
                break;
            case 2:
                printf(" |ID: ");
                break;
            case 10:
                printf(" |BAT: ");
                break;
            case 12:
                printf(" |CH: ");
                break;
            case 14:
                printf(" |TEMP: ");
                break;
            case 26:
                printf(" |HUM: ");
                break;
            case 34:
                printf(" |CRC: ");
                break;
            default:
                break;
            }
            printf("%d", buf[i]);

        }


/*
        printf("\n------------------------------------------------------------\n");
        for (int i = 0; i < NC7427_MESSAGELEN; i++)
        {

            switch (i)
            {
            case 2:
            case 11:
            case 13:
            case 15:
            case 27:
            case 35:
                printf(" | ");
                break;
            default:
                break;
            }
            printf("%d", buf[i]);

        }
 */
 //       if (false) 
 //       {
                        
            
            printf("\n---------------------------------------------------------------------------------------\n");
            for (int i = 0; i < NC7427_MESSAGELEN; i++)
            {
            
                byte n = i ; 

                if ( n < 2)
                {
                    p.d.lead += (buf[i] == 1) ? 1<<i : 0;
                    printf ("[%2d] LEAD:\t%8d - %d\t",n ,p.d.lead , buf[i]);                           
                    //printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw); 
                    //printBits(sizeof(p.b.raw_byt[0] ), &p.b.raw_byt[0]);
                    Serial.println(); 
                }
                else if (n >= 2 && n < 10)
                {             
                    p.d.id |= (buf[i] == 1) ? 1<<(i - 3) : 0;
                    printf("[%2d] ID:\t%8d - %d\t", n, p.d.id, buf[i]);
                    //printBits(sizeof(p.d.id), &p.d.id);
                    //printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                    Serial.println();
                }
                else if (n >= 10 && n < 12)
                {
                    p.d.bat |= (buf[i] == 1) ? 1<<(i - 11) : 0;
                    printf("[%2d] BAT:\t%8d - %d",n, p.d.bat, buf[i]);
                    //printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                    Serial.println();
                }
                else if (n >= 12 && n < 14)
                {             
                    p.d.chan |= (buf[i] == 1) ? 1<<(i - 12) : 0;
                    printf("[%2d] CH:\t%8d - %d",n , p.d.chan, buf[i]);
                    //printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                    Serial.println();
                }
                else if (n >= 14 && n < 26)
                {
                    p.d.temp |= (buf[i] == 1) ? 1<<(i - 15) : 0;
                    printf("[%2d] T:\t\t%8d - %d  0x%X",n , p.d.temp, buf[i], p.d.temp);
                    ///printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                    Serial.println();
                }
                else if (n >= 26 && n < 34)
                {                
                    p.d.hum |= (buf[i] == 1) ? 1<<(i-27) : 0;
                    printf("[%2d] H:\t\t%8d - %d", n, p.d.hum, buf[i]);
                    //printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                    Serial.println();
                }
                else if (n >= 34 )
                {                
                    p.d.crc |= (buf[i]==1) ? 1<<(i-35) : 0;
                    printf("[%2d] CRC:\t%8d - %d", n, p.d.crc, buf[i]);
                    //printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                    Serial.println();
                };
            
            }
            //printf("\n");
  //      }

        
        flagReady = false;
        digitalWrite(DEBUG_2_PIN, LOW);

        printf("------------------------------------------------------------\n");

        printf("ld:\t%d\n", p.d.lead);
        printf("id:\t%d\n", p.d.id);
        printf("bat:\t%d\n", p.d.bat);
        printf("ch:\t%d\n", p.d.chan);
        //unsigned short theta = (unsigned short) p.d.temp; // 12 auf 16 bit !  
        
        printf("temp (union):\t 0x%X \n", p.d.temp); //printf("\t"); printBits(sizeof(p.d.temp), &p.d.temp);
        printf("temp (LLL):\t 0x%X \n", p.d.temp & 0b111100000000); 
        printf("temp (MMM):\t 0x%X \n", p.d.temp & 0b000011110000); 
        printf("temp (HHH):\t 0x%X \n", p.d.temp & 0b000000001111); 
        puts("");
        uint16_t tbits = (uint16_t)p.d.temp<<1; // TODO ls-bit fehlt noch, deshalb einmal links schieben!  0x19B 110011011, sollte temp (bits):	0000001100110110 sein
        printf("tbits:\t 0x%X \n", tbits );
        printf("temp (LLL):\t 0x%X \n", tbits & 0b111100000000);
        printf("temp (MMM):\t 0x%X \n", tbits & 0b000011110000);
        printf("temp (HHH):\t 0x%X \n", tbits & 0b000000001111);
        printf("temp (bits):\t", p.d.temp & 0b000000001111);  printBits(sizeof(tbits), &tbits); puts("");

        //////////////////////////////////////////////////////////////////
        //unsigned short theta = (unsigned short)p.d.temp; 
        //unsigned short thetaRev = reverseBits(theta);        
        //unsigned short theta = p.d.temp;
       // unsigned short thetaRev = theta >> 4; 
       // thetaRev = reverseBits(p.d.temp);
               
        
       // printf("temp(theta):\t 0x%X \n", p.d.temp); printf("\t"); printBits(sizeof(theta), &theta); printf("\t"); printBits(sizeof(thetaRev), &thetaRev);
        
        //p.d.temp = thetaRev; 
        //printf("tempRev :\t 0x%X \n", p.d.temp);
        //printf("thetaRev:\t 0x%X \n", thetaRev>>4); printf("\t"); printBits(sizeof(thetaRev), &thetaRev);
        
        
        //////////////////////////////////////////////////////////////////
        
        
        printf("hum:\t%d\n", p.d.hum);
        printf("crc:\t%d\n", p.d.hum);

        uint64_t ll = p.raw;
        uint64_t xx = ll / 1000000000ULL;

        printf("Raw:\t");
        if (xx >0) Serial.print((long)xx);
            Serial.print((long)(ll - xx * 1000000000));
            Serial.println();

        interrupts();        
    }
}
//-------------------------------------------------------------
void rx433Handler2()
{    
    static long             rx433LineUp, rx433LineDown;    
    static byte             crcBits = 0;
    volatile static byte    cnt = 0;
    volatile static byte    cntSync = 0;
    volatile static byte    frameCnt = 0;
    static volatile boolean lastPulseWasSync = false;
    long                    LowVal;
    long                    HighVal;
    boolean                 isPulseForHigh = false;
    boolean                 isPulseForLow = false;
    boolean                 isPulseSync = false;
    boolean                 isPulseUndef = false;

    byte rx433State = digitalRead(RX433DATA); // current pin state

    if (rx433State)  // pin is now HIGH -> fallende Flanke 
    {
        rx433LineUp = micros(); // line went HIGH, after being LOW at this time  
        LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time                                                 
        isPulseSync    = (LowVal > NC7427_SYNC - NC7427_GLITCH && LowVal < NC7427_SYNC + NC7427_SYNC_GLITCH);
        isPulseForHigh = (LowVal > NC7427_ONE - NC7427_GLITCH  && LowVal < NC7427_ONE  + NC7427_GLITCH);
        isPulseForLow  = (LowVal > NC7427_ZERO - NC7427_GLITCH && LowVal < NC7427_ZERO + NC7427_GLITCH);
        isPulseUndef   = !(isPulseForHigh || isPulseForLow || isPulseSync);
        //--- uncritical: 51uS for this calcs only

        if (isPulseSync)
        {
            cnt = 0;            
            // printf("\n*\n");
            digitalWrite(DEBUG_4_PIN, HIGH);
            lastPulseWasSync = true;
        }
        else if (isPulseForHigh)
        {           
            digitalWrite(DEBUG_1_PIN, HIGH);                
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 1; 
            
            cnt++;

            lastPulseWasSync = false;
        }
        else if (isPulseForLow)
        {            
            digitalWrite(DEBUG_1_PIN, HIGH);            
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 0;            
            
            cnt++;

            lastPulseWasSync = false;
        }
        else if (isPulseUndef)
        {
            cnt = 0;
            digitalWrite(DEBUG_5_PIN, HIGH);
            lastPulseWasSync = false;            
            cntSync = 0;
        }
        // trigger on start pulses, 
        // in our case this NCS protocol has 13 start frames with 8000uS..9000uS lentgh 
        if (lastPulseWasSync)
        {
            cntSync++;
            if (cntSync > 11)
            {                
                frameCnt = 0;
                printf("~\n");   // flags first frame after start sequence                 
            }
        }
        else
        {
            cntSync = 0;
        }

        if (cnt >= (NC7427_MESSAGELEN)) // all bits received
        {
            digitalWrite(12, HIGH);  // LED external pin12 on
            frameCnt++; 
            //printf("\tCount[M]: %d\n", cnt);
            printf("[%d]:", frameCnt);
            cnt = 0;            
            cntSync = 0;
            digitalWrite(DEBUG_3_PIN, HIGH);
            flagReady = true;            
        }
    }
    else
    {   
        //--- high values have no information with them, ignore.
        digitalWrite(12, LOW);
        rx433LineDown = micros();               //--- line went LOW after being HIGH
        HighVal = rx433LineDown - rx433LineUp;  //--- calculate the HIGH pulse time
    }

    volatile unsigned long start_time = micros();

    while (micros() - start_time < 10);
    {
        // wait 10 uS displaytime on Loganalyzer
        ;
    }

    digitalWrite(DEBUG_1_PIN, LOW);
    digitalWrite(DEBUG_2_PIN, LOW);
    digitalWrite(DEBUG_3_PIN, LOW);
    digitalWrite(DEBUG_4_PIN, LOW);
    digitalWrite(DEBUG_5_PIN, LOW);
}

//-------------------------------------------------------------
int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
//-------------------------------------------------------------
void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*)ptr;
    unsigned char byte;
    int i, j;

    for (i = size - 1; i >= 0; i--)
    {
        for (j = 7; j >= 0; j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");  // inkl. CRLF
}
//-------------------------------------------------------------
//--- function that printf and related will use to print
int serial_putchar(char c, FILE* f)
{
    if (c == '\n') serial_putchar('\r', f);
    return Serial.write(c) == 1 ? 0 : 1;
}
//-------------------------------------------------------------
void printResults(unsigned long value)
{
    // Sensor ID
    byte id = value & 0b11001111; // bit 0, 1, 2, 3, 6, 7, random change bits when battery is changed
    Serial.print('\t'); Serial.print(id);
    // Channel (as set on sensor)
    byte channel = 2 * bitRead(value, 4) + bitRead(value, 5); // bit 4, 5 are channel number
    Serial.print('\t'); Serial.print(channel);
    // Temperature
    int temperature = value >> 12 & 0b111111111111;  // bit 12..23
                                                     // if sign bit is set, adjust two's complement to fit a 16-bit number
    if (bitRead(temperature, 11)) temperature = temperature | 0b1111000000000000;
    Serial.print('\t'); Serial.print(temperature / 10.0, 1);
    // temperature tendency
    byte trend = value >> 9 & 0b11; // bit 9, 10
    Serial.print('\t');
    if (trend == 0) Serial.print('=');       // temp tendency steady
    else if (trend == 1) Serial.print('-');  // temp tendency falling
    else if (trend == 2) Serial.print('+');  // temp tendency rising
                                             // Humidity
    byte humidity = (value >> 24 & 0b11111111) - 156; // bit 24..31
    Serial.print('\t'); Serial.print(humidity);
    // Battery State
    bool lowBat = value >> 8 & 0b1;      // bit 8 is set if battery voltage is low
    Serial.print('\t'); Serial.print(lowBat);
    // Trigger
    bool forcedSend = value >> 11 & 0b1;  // bit 11 is set if manual send button was pressed
    Serial.print('\t'); Serial.print(forcedSend);
}
//-------------------------------------------------------------
boolean crcValid(unsigned long value, byte checksum)
// check if received crc is correct for received value
{
    byte calculatedChecksum = 0;
    for (int i = 0; i < 8; i++) calculatedChecksum += (byte)(value >> (i * 4));
    calculatedChecksum &= 0xF;
    return calculatedChecksum == checksum;
}

//-------------------------------------------------------------
void swapArrayBinPos(byte last, byte first)
{
    byte buf2[NC7427_MESSAGELEN];
    byte tmp = 0;
    byte idx = 0;
    
    for (byte n = first; n <= last; n++)
    {
        buf2[n] = buf[n];
    }
    
    for (byte i = first; i <= last; i++)
    {
        idx = first + (last - i);
        buf[i] = buf2[idx]; 
        //printf("buf_i: %d %t buf_idx: %d\n", buf[i], buf2[idx]);
    }
}
//-------------------------------------------------------------
unsigned short reverseBits(unsigned short number)
{
    //Reverse bits the obvious way

    unsigned short v = number;     // input bits to be reversed
    unsigned short r = v;          // r will be reversed bits of v; first get LSB of v
    int s = sizeof(number) * CHAR_BIT - 1; // extra shift needed at end

    for (v >>= 1; v; v >>= 1)
    {
        r <<= 1;
        r |= v & 1;
        s--;
    }
    r <<= s; // shift when v's highest bits are zero

    return r; 

}

//==================================================================
// <eof>
//==================================================================