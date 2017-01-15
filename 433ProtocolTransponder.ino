/* *********************************************************

Purpose:
========
Transforn 42Bit Protocol to LaCrosse-Format

Initial: 20170114_juergs.

Credentials:
http://helmpcb.com/software/reading-writing-structs-floats-and-other-objects-to-eeprom
https://github.com/jacobsax/bitArray-library-for-Arduino
https://github.com/RFD-FHEM/RFFHEM
protocol-definition: https://docs.google.com/document/d/121ZH3omAZsdhFi3GSB-YdnasMjIQSGIcaS7QW6KsACA/mobilebasic?pli=1
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

//C:\Users\Jürgen\Documents\GitHub\42Bit_Protocol

#define DEBUG_1_PIN   11
#define DEBUG_2_PIN   10
#define DEBUG_3_PIN   9
#define DEBUG_4_PIN   8
#define DEBUG_5_PIN   7



#include <bitArray.h> //byte-wide only!

#define RX433DATA 2       // receive pin for hardware interrupts
#define RX433INTERRUPT 0  // interrupt number for receive pin

#define NC7427_SYNC         8000    // length in µs of starting pulse
#define NC7427_SYNC_GLITCH  1100    // pulse length variation for ONE and ZERO pulses
#define NC7427_ONE          4000    // length in µs of ONE pulse
#define NC7427_ZERO         2000    // length in µs of ZERO pulse
#define NC7427_GLITCH        400    // pulse length variation for ONE and ZERO pulses, was 350
#define NC7427_MESSAGELEN     42    //36, number of bits in one message

#define FIFOSIZE 100  // Fifo Buffer size 8 can hold up to 7 items

volatile unsigned long fifoBuf[FIFOSIZE]; // ring buffer, war long 

volatile byte fifoReadIndex, fifoWriteIndex;  // read and write index into ring buffer
volatile boolean flagReady = false;


FILE      serial_stdout;          // needed for printf 

bitArray  proto;

volatile unsigned long long raw;

static byte             buf[NC7427_MESSAGELEN];



union p
{
    unsigned long long raw;
    struct
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

    
container protocol;


void    printResults(unsigned long value);
boolean crcValid(unsigned long value, byte checksum);
//void  rx433Handler();
void    rx433Handler2();
void    fifoWrite(long item);
unsigned long fifoRead();
boolean fifoAvailable();
int     freeRam();
void    printBits(size_t const size, void const * const ptr);
int     serial_putchar(char c, FILE* f);
void    populateBitArray(byte pos, byte value);
byte    readBitArray(byte pos);
void    printLLNumber(unsigned long long n, uint8_t base);

//-------------------------------------------------------------
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

    Serial.println();
    Serial.print(F("Free RAM: ")); Serial.println(freeRam());
    Serial.println();
    Serial.println(F("Seconds\tCRC\tID\tChannel\tTemp C\tTrend\trH %\tLowBat\tForced send"));
    Serial.println(F("         1         2         3         4         5"));
    Serial.println(F("12345678901234567890123456789012345678901234567890 \n"));
    //                  000000000000000000000000000000000000000000


    /*
    p.raw = 61186399307584ULL;

    printf("1101111010011000010001100110001000011101000000\n");
    printf("61186399307584\n");
    printf("0x37A6119887400\n");
    printf("TEST:  %l - %d - %d - %d \n", p.d.dummy, p.d.id, p.d.bat, p.d.chan);
    // Serial.print("TEST2: "); Serial.println((unsigned long long)p.raw,8);
    Serial.println("Internal: ");
    printLLNumber(p.raw,8);

    Serial.println();

    Serial.println("Internal: ");
    uint64_t ll = 123456789012345678ULL;
    uint64_t xx = ll/1000000000ULL;

    if (xx >0) Serial.print((long)xx);
    Serial.print((long)(ll-xx*1000000000));

    Serial.println();

    Serial.print("P: ");
    ll = p.raw;
    xx = ll/1000000000ULL;

    if (xx >0) Serial.print((long)xx);
    Serial.println((long)(ll-xx*1000000000));

    Serial.println();

    ptst.d.b7=128;
    Serial.print("ptst: ");
    ll = ptst.raw;
    xx = ll/1000000000ULL;
    if (xx >0) Serial.print((long)xx);
    Serial.println((long)(ll-xx*1000000000));


    uint64_t pipe = 0x12345ABCD9LL;//lets assume  the data is 12345ABCD9 hexadecimal
    char buf[50];

    if(pipe > 0xFFFFFFFFLL) {
    sprintf(buf, "%lX%08lX", (unsigned long)(pipe>>32), (unsigned long)(pipe&0xFFFFFFFFULL));
    } else {
    sprintf(buf, "%lX", (unsigned long)pipe);
    }

    Serial.println( buf );

    */

    Serial.println("===============================================================");
}
//-------------------------------------------------------------
void loop()
{    

    if (flagReady)
    {
        noInterrupts();

        //--- debug flag 
        digitalWrite(DEBUG_2_PIN, HIGH);
        
        //printf("OUT:");
        printf("\n------------------------------------------------------------\n");
        //p.d.lead = p.d.bat = p.d.chan = p.d.temp = p.d.hum = p.d.crc = 0; 
        p.raw = 0ULL; 
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
                p.d.lead |= (buf[i] == 1) ? 2 ^ (i) : 0;
                printf ("[%2d] LEAD:\t%d - %d\n",n ,p.d.lead , buf[i]);
            }
            else if (n > 2 && n <= 10)
            {             
                p.d.id |= (buf[i] == 1) ? 2 ^ (i - 3) : 0;
                printf("[%2d] ID:\t%d - %d\n", n, p.d.id, buf[i]);
            }
            else if (n > 10 && n <= 12)
            {
                p.d.bat |= (buf[i] == 1) ? 2 ^ (i - 11) : 0;
                printf("[%2d] BAT:\t%d - %d\n",n, p.d.bat, buf[i]);
            }
            else if (n > 12 && n <= 14)
            {             
                p.d.chan |= (buf[i] == 1) ? 2 ^ (i - 12) : 0;
                printf("[%2d] CH:\t%d - %d\n",n , p.d.chan, buf[i]);
            }
            else if (n > 14 && n <= 26)
            {
                p.d.temp |= (buf[i] == 1) ? 2 ^ (i - 15) : 0;
                printf("[%2d] T:\t\t%d - %d\n",n , p.d.temp, buf[i]);
            }
            else if (n > 26 && n <= 34)
            {                
                p.d.hum |= (buf[i] == 1) ? 2^(i-27) : 0;
                printf("[%2d] H:\t\t%d - %d\n", n, p.d.hum, buf[i]);
            }
            else if (n > 34 )
            {                
                p.d.crc |= (buf[i]==1) ? 2^(i-35) : 0;
                printf("[%2d] CRC:\t%d - %d\n", n, p.d.crc, buf[i]);
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
    static unsigned long    rxBits = 0;
    static byte             crcBits = 0;
    volatile static byte    cnt = 0;
    volatile static byte    cntSync = 0;
    volatile static byte    frameCnt = 0;
    static volatile boolean startCondition = false;
    static volatile boolean lastPulseWasSync = false;
    long                    LowVal, HighVal;
    unsigned long           dauer, timestamp;
    boolean                 isPulseForHigh = false;
    boolean                 isPulseForLow = false;
    boolean                 isPulseSync = false;
    boolean                 isPulseUndef = false;

    byte rx433State = digitalRead(RX433DATA); // current pin state

    if (rx433State)  // pin is now HIGH -> fallende Flanke 
    {
        rx433LineUp = micros(); // line went HIGH, after being LOW at this time  
        LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time                                                 
        isPulseSync = (LowVal > NC7427_SYNC - NC7427_GLITCH && LowVal < NC7427_SYNC + NC7427_SYNC_GLITCH);
        isPulseForHigh = (LowVal > NC7427_ONE - NC7427_GLITCH && LowVal < NC7427_ONE + NC7427_GLITCH);
        isPulseForLow = (LowVal > NC7427_ZERO - NC7427_GLITCH && LowVal < NC7427_ZERO + NC7427_GLITCH);
        isPulseUndef = !(isPulseForHigh || isPulseForLow || isPulseSync);
        //--- uncritical: 51uS for this calcs only

        if (isPulseSync)
        {
            cnt = 0;
            
            //if (startCondition) 
            //    printf("\n*\n");
            digitalWrite(DEBUG_4_PIN, HIGH);
            lastPulseWasSync = true;
        }
        else if (isPulseForHigh)
        {
            cnt++;
            digitalWrite(DEBUG_1_PIN, HIGH);
            //printf("[H] %d ", cnt);
            //printf("H%d ", startCondition);
            //if (startCondition) 
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 1; 
            
            //printf("1 - %d - %d\n", cnt,  buf[cnt]);
            //sFIFO.enqueue(1);
            
            lastPulseWasSync = false;
        }
        else if (isPulseForLow)
        {            
            cnt++;

            //
            digitalWrite(DEBUG_1_PIN, HIGH);

            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 0;

            //printf("[L] %d ", cnt);
            //printf("L%d ", startCondition);
            //if (startCondition) 
            
            //printf("0 - %d - %d\n", cnt, buf[cnt]);
            
            //sFIFO.enqueue(0);

            lastPulseWasSync = false;
        }
        else if (isPulseUndef)
        {
            cnt = 0;
            digitalWrite(DEBUG_5_PIN, HIGH);
            lastPulseWasSync = false;
            startCondition = false;
            cntSync = 0;
            //frameCnt = 0; 
            //printf("#\n");
        }

        // trigger on start pulses, 
        // in our case this NCS protocol has 13 start frames with 8000uS..9000uS lentgh 
        if (lastPulseWasSync)
        {
            cntSync++;
            if (cntSync > 11)
            {
                startCondition = true;
                frameCnt = 0;
                printf("~\n");   // flags first frame after start sequence 
                //printf("\nSync: %d [S] %d   \n", cntSync, startCondition);
            }
        }
        else
        {
            //startCondition = false; 
            cntSync = 0;
        }

        if (cnt >= (NC7427_MESSAGELEN)) // all bits received
        {
            digitalWrite(12, HIGH);  //LED external pin12 on

            frameCnt++; 
            //printf("\tCount[M]: %d\n", cnt);
            printf("[%d]:", frameCnt);
            cnt = 0;            
            cntSync = 0;

            digitalWrite(DEBUG_3_PIN, HIGH);

            flagReady = true;

            /*
            printf("RAW: %u \t| RAW: %u \t| ID: %d \t| BAT: %d \t| CHAN: %d \t| TEMP: %d \t| HUM: %d \t| CRC: %d \n",raw, p.raw, p.d.id, p.d.bat, p.d.chan, p.d.temp, p.d.hum, p.d.crc );

            uint64_t ll = p.raw;
            uint64_t xx = ll / 1000000000ULL;
            printf("p.raw: ");
            if (xx >0) Serial.print((long)xx);
            Serial.print((long)(ll - xx * 1000000000));
            Serial.print("  - ");
            */

            /*
            ll = raw;
            xx = ll / 1000000000ULL;
            printf("raw: ");
            if (xx >0) Serial.print((long)xx);
            Serial.println((long)(ll - xx * 1000000000));
            */

            /*
            for (int i=0; i<42; i++)
            Serial.print(readBitArray(i));
            Serial.println("");
            */

            /*
            //noInterrupts();
            unsigned long start_time = micros();
            unsigned long duration = (10 * 1000 * 1000);
            while ((micros() - start_time) < duration );
            {
            // wait 10 uS for display on Loganalyzer - delay not functional?
            ;
            }
            //interrupts();
            */
        }
    }
    else
    { // High values have no information with them
        digitalWrite(12, LOW);
        rx433LineDown = micros();               // line went LOW after being HIGH
        HighVal = rx433LineDown - rx433LineUp; // calculate the HIGH pulse time
    }

    volatile unsigned long start_time = micros();

    while (micros() - start_time < 10);
    {
        // wait 10 uS for display on Loganalyzer
        ;
    }

    digitalWrite(DEBUG_1_PIN, LOW);
    digitalWrite(DEBUG_2_PIN, LOW);
    digitalWrite(DEBUG_3_PIN, LOW);
    digitalWrite(DEBUG_4_PIN, LOW);
    digitalWrite(DEBUG_5_PIN, LOW);
}
//-------------------------------------------------------------
void populateBitArray(byte pos, byte value)
{
    proto.writeBit(pos, value);
    /*
    if (pos <=7)
    proto1.writeBit(pos,value);
    else if (pos > 7 && pos <= 15)
    proto2.writeBit(pos,value);
    else if (pos> 15 && pos <= 23)
    proto3.writeBit(pos,value);
    else if (pos >23 && pos <= 31)
    proto4.writeBit(pos,value);
    else if (pos >31 && pos <= 39)
    proto5.writeBit(pos,value);
    else
    proto6.writeBit(pos,value);
    */
}
//-------------------------------------------------------------
byte readBitArray(byte pos)
{
    byte val = LOW;

    val = proto.readBit(pos);
    /*
    if (pos <=7)
    val = proto1.readBit(pos);
    else if (pos > 7 && pos <= 15)
    val = proto2.readBit(pos);
    else if (pos> 15 && pos <= 23)
    val = proto3.readBit(pos);
    else if (pos >23 && pos <= 31)
    val = proto4.readBit(pos);
    else if (pos >31 && pos <= 39)
    val = proto5.readBit(pos);
    else
    val = proto6.readBit(pos);
    */

    return val;
}
//-------------------------------------------------------------
/*
///////////////////////////////////////////////////////////////////////////////////////
/// dont touch!
void rx433Handler()
{
static long           rx433LineUp, rx433LineDown;
static unsigned long  rxBits=0;
static unsigned long  rxBits_2=0;
static byte           crcBits=0;
static byte           cnt=0;
long                  LowVal, HighVal;

byte rx433State = digitalRead(RX433DATA); // current pin state
if (rx433State) // pin is now HIGH
{
rx433LineUp = micros(); // line went HIGH after being LOW at this time

LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time

if (LowVal>NC7427_SYNC-2*NC7427_GLITCH && LowVal<NC7427_SYNC+2*NC7427_GLITCH)
{
rxBits=0;
rxBits_2 = 0;
crcBits=0;
cnt=0;
}
else if (LowVal>NC7427_ONE-NC7427_GLITCH && LowVal<NC7427_ONE+NC7427_GLITCH)
{
// set the one bits, war 32  = ulong size !
if (cnt<32)
{
bitSet(rxBits,cnt);
printf("cnt %d - rxBits: %d \n",cnt,rxBits);
}
else
{
bitSet(crcBits, cnt-32);
printf("cnt %d - crcBits: %d \n",cnt,crcBits);
}
if (cnt >= 32)
{
bitSet(rxBits_2,cnt);
printf("cnt %d - rxBits_2: %d \n",cnt,rxBits_2);
}
cnt++;
}
else if (LowVal>NC7427_ZERO-NC7427_GLITCH && LowVal<NC7427_ZERO+NC7427_GLITCH)
{ // setting zero bits is not necessary, but count them
cnt++;
}
else // received bit is not a SYNC, ONE or ZERO bit, so restart
{
rxBits=0;
rxBits_2=0;
crcBits=0;
cnt=0;
}
if (cnt>=NC7427_MESSAGELEN) // all bits received
{
digitalWrite(12,HIGH);
printf("Bit counted: %d\n",cnt);

fifoWrite(rxBits); // write valid value to FIFO buffer

//      if (crcValid(rxBits,crcBits))
//        fifoWrite(rxBits); // write valid value to FIFO buffer
//      else
//        fifoWrite(0);  // write 0 to FIFO buffer (0 = invalid value received)

rxBits=0;
crcBits=0;
cnt=0;


}
}
else
{ // High values have no information with them
digitalWrite(12,LOW);
rx433LineDown = micros(); // line went LOW after being HIGH
HighVal = rx433LineDown - rx433LineUp; // calculate the HIGH pulse time
}
}
*/

//-------------------------------------------------------------
/* rxhandler old
void rx433Handler_fifo()
{
static long rx433LineUp, rx433LineDown;
static unsigned long rxBits = 0;
static byte crcBits = 0;
static byte cnt = 0;
long LowVal, HighVal;
byte rx433State = digitalRead(RX433DATA); // current pin state
if (rx433State) // pin is now HIGH
{
rx433LineUp = micros(); // line went HIGH after being LOW at this time
LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time
if (LowVal>NC7427_SYNC - 2 * NC7427_GLITCH && LowVal<NC7427_SYNC + 2 * NC7427_GLITCH)
{
rxBits = 0;
crcBits = 0;
cnt = 0;
}
else if (LowVal>NC7427_ONE - NC7427_GLITCH && LowVal<NC7427_ONE + NC7427_GLITCH)
{ // set the one bits
if (cnt<32)
bitSet(rxBits, cnt);
else
bitSet(crcBits, cnt - 32);
cnt++;
}
else if (LowVal>NC7427_ZERO - NC7427_GLITCH && LowVal<NC7427_ZERO + NC7427_GLITCH)
{ // setting zero bits is not necessary, but count them
cnt++;
}
else // received bit is not a SYNC, ONE or ZERO bit, so restart
{
rxBits = 0;
crcBits = 0;
cnt = 0;
}
if (cnt >= NC7427_MESSAGELEN) // all bits received
{
if (crcValid(rxBits, crcBits))
fifoWrite(rxBits); // write valid value to FIFO buffer
else
fifoWrite(0);  // write 0 to FIFO buffer (0 = invalid value received)

rxBits = 0;
crcBits = 0;
cnt = 0;
}
}
else
{ // High values have no information with them
rx433LineDown = micros(); // line went LOW after being HIGH
HighVal = rx433LineDown - rx433LineUp; // calculate the HIGH pulse time
}
}
*/

//-------------------------------------------------------------
void fifoWrite(long item)
// write item into ring buffer
{
    fifoBuf[fifoWriteIndex] = item; // store the item
    if (!(fifoWriteIndex + 1 == fifoReadIndex || (fifoWriteIndex + 1 >= FIFOSIZE && fifoReadIndex == 0)))
        fifoWriteIndex++;  // advance write pointer in ringbuffer
    if (fifoWriteIndex >= FIFOSIZE) fifoWriteIndex = 0; // ring buffer is at its end
}

//-------------------------------------------------------------
unsigned long fifoRead()
// always check first if item is available with fifoAvailable()
// before reading the ring buffer using this function
{
    unsigned long item;
    item = fifoBuf[fifoReadIndex];
    cli(); // Interrupts off while changing the read pointer for the ringbuffer
    fifoReadIndex++;
    if (fifoReadIndex >= FIFOSIZE) fifoReadIndex = 0;
    sei(); // Interrupts on again
    return(item);
}
//-------------------------------------------------------------
boolean fifoAvailable()
// item is available for reading if (fifoReadIndex!=fifoWriteIndex)
{
    return (fifoReadIndex != fifoWriteIndex);
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

void printLLNumber(unsigned long long n, uint8_t base)
{
    unsigned char buf[16 * sizeof(long)]; // Assumes 8-bit chars.
    unsigned long long i = 0;

    if (n == 0) {
        printf("0");
        return;
    }

    while (n > 0) {
        buf[i++] = n % base;
        n /= base;
    }
    char s;
    s = (char)(buf[i - 1] < 10 ? "0" + buf[i - 1] : "A" + buf[i - 1] - 10);
    for (; i > 0; i--)
        printf("%d\n", s);
}
//==================================================================
// <eof>
//==================================================================