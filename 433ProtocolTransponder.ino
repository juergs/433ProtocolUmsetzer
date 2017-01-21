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
        byte lead : 2;
        byte id : 8;
        byte bat : 2;
        byte chan : 2;
        unsigned short temp : 12;
        byte hum : 8;
        byte crc : 8;
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
static byte buf[NC7427_MESSAGELEN];
container   protocol;
//======================================================================
void    printResults(unsigned long value);
boolean crcValid(unsigned long value, byte checksum);
void    rx433Handler2();
int     freeRam();
void    printBits(size_t const size, void const * const ptr);
int     serial_putchar(char c, FILE* f);
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
    Serial.println();
    Serial.print(F("Free RAM: ")); Serial.println(freeRam());
    Serial.println();
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
        
        printf("\n------------------------------------------------------------\n");
        
        p.raw = 0; 
        for (int i = 1; i <= NC7427_MESSAGELEN; i++)
        {

            switch (i - 1)
            {
                case 2:
                case 10:
                case 12:
                case 14:
                case 26:
                case 34:
                    printf(" | ");
                    printf("%d", buf[i]);
                    break;
                default:
                    printf("%d", buf[i]);
            }
        }
        printf("\n------------------------------------------------------------\n");
        for (int i = 1; i <= NC7427_MESSAGELEN; i++)
        {
            
            byte n = i-1 ; 

            if (n > 0 && n <= 2)
            {
                p.d.lead += (buf[i] == 1) ? 1<<i : 0;
                printf ("[%2d] LEAD:\t%8d - %d",n ,p.d.lead , buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw); 
                Serial.println(); 
            }
            else if (n > 2 && n <= 10)
            {             
                p.d.id |= (buf[i] == 1) ? 1<<(i - 3) : 0;
                printf("[%2d] ID:\t%8d - %d", n, p.d.id, buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                Serial.println();
            }
            else if (n > 10 && n <= 12)
            {
                p.d.bat |= (buf[i] == 1) ? 1<<(i - 11) : 0;
                printf("[%2d] BAT:\t%8d - %d",n, p.d.bat, buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                Serial.println();
            }
            else if (n > 12 && n <= 14)
            {             
                p.d.chan |= (buf[i] == 1) ? 1<<(i - 12) : 0;
                printf("[%2d] CH:\t%8d - %d",n , p.d.chan, buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                Serial.println();
            }
            else if (n > 14 && n <= 26)
            {
                p.d.temp |= (buf[i] == 1) ? 1<<(i - 15) : 0;
                printf("[%2d] T:\t\t%8d - %d",n , p.d.temp, buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                Serial.println();
            }
            else if (n > 26 && n <= 34)
            {                
                p.d.hum |= (buf[i] == 1) ? 1<<(i-27) : 0;
                printf("[%2d] H:\t\t%8d - %d", n, p.d.hum, buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                Serial.println();
            }
            else if (n > 34 )
            {                
                p.d.crc |= (buf[i]==1) ? 1<<(i-35) : 0;
                printf("[%2d] CRC:\t%8d - %d", n, p.d.crc, buf[i]);
                printf("\traw: %u = ", p.raw); printBits(sizeof(p.raw), &p.raw);
                Serial.println();
            };
            
        }

        printf("\n");
        flagReady = false;
        digitalWrite(DEBUG_2_PIN, LOW);

        printf("\n------------------------------\n");
        printf("ld:\t%d\n", p.d.lead);
        printf("id:\t%d\n", p.d.id);
        printf("bat:\t%d\n", p.d.bat);
        printf("ch:\t%d\n", p.d.chan);
        printf("temp:\t%d\n", p.d.temp);
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
            cnt++;
            digitalWrite(DEBUG_1_PIN, HIGH);                
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 1; 

            lastPulseWasSync = false;
        }
        else if (isPulseForLow)
        {            
            cnt++;
            digitalWrite(DEBUG_1_PIN, HIGH);            
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 0;            
            
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
    puts("");
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

//==================================================================
// <eof>
//==================================================================