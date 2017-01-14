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

    
********************************************************* */

#include "fifo.h"

/*
*
* typedef union {
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

// Arduino sketch to receive KW9010 temperature/humidity RF sensor telegrams
// Written by 'jurs' for German Arduino Forum
//https://github.com/jacobsax/bitArray-library-for-Arduino
#include <bitArray.h> //byte-wide only!

#define RX433DATA 2       // receive pin for hardware interrupts
#define RX433INTERRUPT 0  // interrupt number for receive pin

#define KW9010_SYNC   8000    // length in µs of starting pulse
#define KW9010_SYNC_GLITCH 1000     // pulse length variation for ONE and ZERO pulses
#define KW9010_ONE    4000     // length in µs of ONE pulse
#define KW9010_ZERO   2000    // length in µs of ZERO pulse
#define KW9010_GLITCH 250     // pulse length variation for ONE and ZERO pulses
#define KW9010_MESSAGELEN 42 //36  // number of bits in one message

#define FIFOSIZE 100  // Fifo Buffer size 8 can hold up to 7 items

volatile unsigned long fifoBuf[FIFOSIZE]; // ring buffer, war long 

volatile byte fifoReadIndex, fifoWriteIndex;  // read and write index into ring buffer
volatile boolean flagReady = false; 


FILE      serial_stdout;          // needed for printf 

bitArray  proto;

volatile unsigned long long raw;

union p
{
    unsigned long long raw;
    struct
    {
        unsigned long dummy : 22;
        byte lead : 2;
        byte id   : 8;
        byte bat  : 2;
        byte chan : 2;
        unsigned short temp : 12;
        byte hum  : 8;
        byte crc  : 8;
    } d;
} p;

union pd
{
    unsigned long long raw;
    struct
    {
        byte b0;
        byte b1;
        byte b2;
        byte b3;
        byte b4;
        byte b5;
        byte b6;
        byte b7;
    } d;
} ptst;

SimpleFIFO<uint8_t, FIFOSIZE> sFIFO; //store 8 bytes


                           /*
bitArray  proto1;
bitArray  proto2;
bitArray  proto3;
bitArray  proto4;
bitArray  proto5;
bitArray  proto6;
*/
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

    Serial.println("#############");
}
//-------------------------------------------------------------
void loop()
{
    /*
    if (fifoAvailable())
    {

    unsigned long dataReceived=fifoRead();
    Serial.print(millis()/1000);
    if (dataReceived!=0)
    {
    Serial.print(F("\tOK\t"));
    printBits(sizeof(dataReceived),&dataReceived);
    //Serial.println("");
    printResults(dataReceived);
    }
    else
    Serial.print(F("\tFAIL"));
    Serial.println();
    }
    */


    if (flagReady)
    {
        noInterrupts(); 
        //printf("F: %d \n", sFIFO.count());
        //Serial.println(F("         1         2         3         4         5"));
        //Serial.println(F("12345678901234567890123456789012345678901234567890 \n"));
        for (int i = 0; i < sFIFO.count(); i++)
        {
            printf("%d", sFIFO.dequeue()); 
            
            //Serial.print(F("Dequeue "));
            //Serial.println(sFIFO.dequeue());
        }
        printf("\n");
        flagReady = false; 
        sFIFO.flush();
        interrupts(); 
    }
    //else
    //    printf("F: %d \n", sFIFO.count());

    //delay(10000); 

}


//-------------------------------------------------------------
void rx433Handler2()
{
    static long           rx433LineUp, rx433LineDown;
    static unsigned long  rxBits = 0;
    static byte           crcBits = 0;
    volatile static byte    bitsCounted = 0;
    volatile static byte    cntSync = 0;
    volatile static boolean startCondition = false;
    volatile static boolean lastPulseWasSync = false;
    long                  LowVal, HighVal;
    unsigned long         dauer, timestamp;
    boolean               isPulseForHigh = false;
    boolean               isPulseForLow = false;
    boolean               isPulseSync = false;
    boolean               isPulseUndef = false;

    byte rx433State = digitalRead(RX433DATA); // current pin state

    if (rx433State)  // pin is now HIGH -> fallende Flanke 
    {
        rx433LineUp = micros(); // line went HIGH, after being LOW at this time  
        LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time                                                 
        isPulseSync = (LowVal > KW9010_SYNC - KW9010_GLITCH && LowVal < KW9010_SYNC + KW9010_SYNC_GLITCH);
        isPulseForHigh = (LowVal > KW9010_ONE - KW9010_GLITCH && LowVal < KW9010_ONE + KW9010_GLITCH);
        isPulseForLow = (LowVal > KW9010_ZERO - KW9010_GLITCH && LowVal < KW9010_ZERO + KW9010_GLITCH);
        isPulseUndef = !(isPulseForHigh || isPulseForLow || isPulseSync);
        //--- uncritical: insgesamt 51uS fuer diese Berechnungen

        if (isPulseSync)
        {
            bitsCounted = 0;
            //if (startCondition)
            //printf("\n*");
            digitalWrite(DEBUG_4_PIN, HIGH);
            lastPulseWasSync = true; 
        }
        else if (isPulseForHigh)
        {                                                                                                                   
            bitsCounted++;
            digitalWrite(DEBUG_1_PIN, HIGH);
            //printf("[H] %d ", bitsCounted);
            //printf("H%d ", startCondition);
            //if (startCondition)
            //    printf("1");
            sFIFO.enqueue(1);
            lastPulseWasSync = false;
        }
        else if (isPulseForLow)
        {
            //populateBitArray(bitsCounted,LOW);                
            bitsCounted++;
            digitalWrite(DEBUG_2_PIN, HIGH);
            digitalWrite(DEBUG_1_PIN, HIGH);
            
            //printf("[L] %d ", bitsCounted);
            //printf("L%d ", startCondition);
            //if (startCondition)
            //    printf("0");
            sFIFO.enqueue(0);

            lastPulseWasSync = false;
        }
        else if (isPulseUndef)
        {
            bitsCounted = 0;
            p.raw = 0;
            raw = 0;
            
            digitalWrite(DEBUG_5_PIN, HIGH);
            
            //printf("\n ");
            
            lastPulseWasSync = false;
            startCondition = false; 
            cntSync = 0;            
        }

        if (lastPulseWasSync)
        {
            cntSync++; 
            if (cntSync > 11)
            {
                startCondition = true;                
                printf("\nSync: %d [S] %d   \n", cntSync, startCondition);
            }
        }
        else
        { 
            //startCondition = false; 
            cntSync = 0; 
        }

        if ( bitsCounted >= (KW9010_MESSAGELEN) ) // all bits received
        {
            digitalWrite(12, HIGH);  //LED external pin12 on
            
            printf("\tCount[M]: %d\n", bitsCounted);
            
            bitsCounted = 0;
            
            p.raw = 0LL;
            raw = 0;
            
            startCondition = false;
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
static byte           bitsCounted=0;
long                  LowVal, HighVal;

byte rx433State = digitalRead(RX433DATA); // current pin state
if (rx433State) // pin is now HIGH
{
rx433LineUp = micros(); // line went HIGH after being LOW at this time

LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time

if (LowVal>KW9010_SYNC-2*KW9010_GLITCH && LowVal<KW9010_SYNC+2*KW9010_GLITCH)
{
rxBits=0;
rxBits_2 = 0;
crcBits=0;
bitsCounted=0;
}
else if (LowVal>KW9010_ONE-KW9010_GLITCH && LowVal<KW9010_ONE+KW9010_GLITCH)
{
// set the one bits, war 32  = ulong size !
if (bitsCounted<32)
{
bitSet(rxBits,bitsCounted);
printf("bitsCounted %d - rxBits: %d \n",bitsCounted,rxBits);
}
else
{
bitSet(crcBits, bitsCounted-32);
printf("bitsCounted %d - crcBits: %d \n",bitsCounted,crcBits);
}
if (bitsCounted >= 32)
{
bitSet(rxBits_2,bitsCounted);
printf("bitsCounted %d - rxBits_2: %d \n",bitsCounted,rxBits_2);
}
bitsCounted++;
}
else if (LowVal>KW9010_ZERO-KW9010_GLITCH && LowVal<KW9010_ZERO+KW9010_GLITCH)
{ // setting zero bits is not necessary, but count them
bitsCounted++;
}
else // received bit is not a SYNC, ONE or ZERO bit, so restart
{
rxBits=0;
rxBits_2=0;
crcBits=0;
bitsCounted=0;
}
if (bitsCounted>=KW9010_MESSAGELEN) // all bits received
{
digitalWrite(12,HIGH);
printf("Bit counted: %d\n",bitsCounted);

fifoWrite(rxBits); // write valid value to FIFO buffer

//      if (crcValid(rxBits,crcBits))
//        fifoWrite(rxBits); // write valid value to FIFO buffer
//      else
//        fifoWrite(0);  // write 0 to FIFO buffer (0 = invalid value received)

rxBits=0;
crcBits=0;
bitsCounted=0;


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
/*
void rx433Handler_fifo()
{
    static long rx433LineUp, rx433LineDown;
    static unsigned long rxBits = 0;
    static byte crcBits = 0;
    static byte bitsCounted = 0;
    long LowVal, HighVal;
    byte rx433State = digitalRead(RX433DATA); // current pin state
    if (rx433State) // pin is now HIGH
    {
        rx433LineUp = micros(); // line went HIGH after being LOW at this time
        LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time
        if (LowVal>KW9010_SYNC - 2 * KW9010_GLITCH && LowVal<KW9010_SYNC + 2 * KW9010_GLITCH)
        {
            rxBits = 0;
            crcBits = 0;
            bitsCounted = 0;
        }
        else if (LowVal>KW9010_ONE - KW9010_GLITCH && LowVal<KW9010_ONE + KW9010_GLITCH)
        { // set the one bits
            if (bitsCounted<32)
                bitSet(rxBits, bitsCounted);
            else
                bitSet(crcBits, bitsCounted - 32);
            bitsCounted++;
        }
        else if (LowVal>KW9010_ZERO - KW9010_GLITCH && LowVal<KW9010_ZERO + KW9010_GLITCH)
        { // setting zero bits is not necessary, but count them
            bitsCounted++;
        }
        else // received bit is not a SYNC, ONE or ZERO bit, so restart
        {
            rxBits = 0;
            crcBits = 0;
            bitsCounted = 0;
        }
        if (bitsCounted >= KW9010_MESSAGELEN) // all bits received
        {
            if (crcValid(rxBits, crcBits))
                fifoWrite(rxBits); // write valid value to FIFO buffer
            else
                fifoWrite(0);  // write 0 to FIFO buffer (0 = invalid value received)

            rxBits = 0;
            crcBits = 0;
            bitsCounted = 0;
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