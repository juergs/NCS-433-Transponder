/* 
**********************************************
Version 1.0.1 20170129 juergs. Github upload.
**********************************************

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


*/



#include <stdint.h>
#include "fifo_buffer.h"
#include "LaCrosse.h"

// --- for debugging purpouses
#define PIN_LED       13  
#define DEBUG_1_PIN   11
#define DEBUG_2_PIN   10
#define DEBUG_3_PIN   9
#define DEBUG_4_PIN   8
#define DEBUG_5_PIN   7
#define DEBUG_6_PIN   6

#define PIN_SEND      5  // TX 433-Module Pin 

#define RX433DATA 2       // receive pin for hardware interrupts
#define RX433INTERRUPT 0  // interrupt number for receive pin

#define NC7427_SYNC         8000    // length in µs of starting pulse
#define NC7427_SYNC_GLITCH  1100    // pulse length variation for ONE and ZERO pulses
#define NC7427_ONE          4000    // length in µs of ONE pulse
#define NC7427_ZERO         2000    // length in µs of ZERO pulse
#define NC7427_GLITCH        400    // pulse length variation for ONE and ZERO pulses, was 350
#define NC7427_MESSAGELEN     42    //36, n

union proto_union
{
    unsigned long long raw;
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

//======================================================================
volatile boolean    flagReady = false;
FILE                serial_stdout;                             // needed for printf 
volatile unsigned long long raw;
volatile byte       buf[NC7427_MESSAGELEN];
volatile bool       blinkLED = false;
uint64_t            var;
uint64_t            *pvar;

//====prototypes========================================================
void    rx433Handler2();
int     freeRam();
void    printBits(size_t const size, void const * const ptr);
int     serial_putchar(char c, FILE* f);
void    swapArrayBinPos(byte first, byte last);
void    fillProtocolPattern();
void    printSwappedBuffer();
void    printNonSwappedBuffer();
void    printBuffer();
void    printBufferRawValue();
void    printLongRaw(uint64_t raw);
void    LaCrosseSend(byte sensorId, double temperature, double humidity);

//=======================================================================

void setup()
{
    //--- set stdout for printf to serial
    fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serial_stdout;

    //Serial.begin(57600);
    Serial.begin(115200);

    pinMode(PIN_SEND, OUTPUT);  // 433 TX sender 
    digitalWrite(PIN_SEND, LOW);

    pinMode(RX433DATA, INPUT);      // 433 RX receiver
    attachInterrupt(RX433INTERRUPT, rx433Handler2, CHANGE);
    
    pinMode(PIN_LED, OUTPUT);  // internal
    digitalWrite(PIN_LED, LOW);
    
    pinMode(12, OUTPUT);        // External LED, flashes by receiving NCS-telegrams 
    digitalWrite(12, LOW);

    /*
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

    pinMode(DEBUG_6_PIN, OUTPUT);
    digitalWrite(DEBUG_6_PIN, LOW);

*/ 
    

    Serial.println("===============================================================");
    Serial.print(F("\tFree RAM: ")); Serial.println(freeRam());
    Serial.println("===============================================================");
    Serial.println();
}
//-------------------------------------------------------------
void loop()
{
    volatile static unsigned long start_time = 0;

    if (flagReady)
    {
        start_time = millis();

        noInterrupts();

        //--- debug flag - 
        //digitalWrite(DEBUG_2_PIN, HIGH);

        p.raw = 0;

        //--- id bitpos swapping.
        swapArrayBinPos(9, 2);

        //--- ch bitpos swapping.
        swapArrayBinPos(13, 12);

        //--- temperature bitpos swapping.
        swapArrayBinPos(25, 14);

        //--- humidity bitpos swapping
        swapArrayBinPos(33, 26);

        //--- crc bitpos swapping
        swapArrayBinPos(41, 34);

        fillProtocolPattern();

        //printf("Stored.\n" );  //printLongRaw(p.raw);

        var = p.raw;

        uint8_t ret = BufferIn(var);
        if (ret == BUFFER_FAIL)
            printf("Buffer-IN-Error\n");

        //digitalWrite(DEBUG_2_PIN, LOW);

        flagReady = false;
        interrupts();
    }
    else
    {

        //digitalWrite(DEBUG_6_PIN, HIGH);
        unsigned long delta = millis() - start_time;
        if (delta > 5000)
        {
            //digitalWrite(PIN_LED, HIGH);
            //digitalWrite(DEBUG_6_PIN, HIGH);
            //printf("delta: %d\n", delta); 
            //printBufferRawValue(); printf("\t"); printBits(sizeof(uint64_t), &p.raw); 
            //printBuffer();
            //printBufferRawValue();
            printBuffer();
            start_time = 0;
        }
        /*
        else
        {
        delay(1000);
        }
        */
    }
    /*
    digitalWrite(DEBUG_6_PIN, LOW);
    digitalWrite(DEBUG_2_PIN, LOW);
    */
    digitalWrite(PIN_LED, LOW);


}
//-------------------------------------------------------------
void rx433Handler2()
{
    static long             rx433LineUp, rx433LineDown;
    //static byte            crcBits        = 0;
    volatile static byte    cnt = 0;
    volatile static byte    cntSync = 0;
    volatile static byte    frameCnt = 0;
    static volatile boolean lastPulseWasSync = false;
    unsigned long           LowVal = 0l;
    unsigned long           HighVal = 0l;
    boolean                 isPulseForHigh = false;
    boolean                 isPulseForLow = false;
    boolean                 isPulseSync = false;
    boolean                 isPulseUndef = false;

    byte rx433PinState = digitalRead(RX433DATA);    //--- current pin state    
    if (rx433PinState)                          //--- pin is now HIGH -> fallende Flanke 
    {
        rx433LineUp = micros();             //--- line went HIGH, after being LOW at this time  
        LowVal = rx433LineUp - rx433LineDown; //--- calculate the LOW pulse time                                                 
        isPulseSync = (LowVal > NC7427_SYNC - NC7427_GLITCH && LowVal < NC7427_SYNC + NC7427_SYNC_GLITCH);
        isPulseForHigh = (LowVal > NC7427_ONE - NC7427_GLITCH  && LowVal < NC7427_ONE + NC7427_GLITCH);
        isPulseForLow = (LowVal > NC7427_ZERO - NC7427_GLITCH && LowVal < NC7427_ZERO + NC7427_GLITCH);
        isPulseUndef = !(isPulseForHigh || isPulseForLow || isPulseSync);
        //--- uncritical: 51uS for this calcs only

        if (isPulseSync)
        {
            cnt = 0;
            lastPulseWasSync = true;
        }
        else if (isPulseForHigh)
        {
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 1;
            cnt++;
            lastPulseWasSync = false;
        }
        else if (isPulseForLow)
        {
            if (cnt <= NC7427_MESSAGELEN)
                buf[cnt] = 0;
            cnt++;
            lastPulseWasSync = false;
        }
        else if (isPulseUndef)
        {
            cnt = 0;
            lastPulseWasSync = false;
            cntSync = 0;
        }
        // trigger on start pulses, 
        // in our case this NCS protocol has 13 start frames with 8000uS..9000uS+TOL lentgh 
        if (lastPulseWasSync)
        {
            cntSync++;
            if (cntSync > 11)
            {
                frameCnt = 0;
                //printf("~\n");   // flags first frame after start sequence                 
            }
        }
        else
        {
            cntSync = 0;
        }

        if (cnt >= (NC7427_MESSAGELEN)) // all bits received
        {
            digitalWrite(12, HIGH);  // LED external pin12 on signal ready 
            frameCnt++;
            //printf("[%d]:", frameCnt);   // displays number of protocol-repetition 
            cnt = 0;
            cntSync = 0;
            flagReady = true;   // flag ready to loop function
        }
    }
    else
    {
        //--- high values have no information with them, ignore.
        digitalWrite(12, LOW);  // external led off
        rx433LineDown = micros();               //--- line went LOW after being HIGH
        HighVal = rx433LineDown - rx433LineUp;  //--- calculate the HIGH pulse time
    }

    //volatile unsigned long start_time = micros();
    //while (micros() - start_time < 10);
    //{
    //    // wait 10 uS displaytime on Loganalyzer
    //    ;
    //}

    //digitalWrite(DEBUG_1_PIN, LOW);
    //digitalWrite(DEBUG_2_PIN, LOW);
    //digitalWrite(DEBUG_3_PIN, LOW);
    //digitalWrite(DEBUG_4_PIN, LOW);
    //digitalWrite(DEBUG_5_PIN, LOW);    
}

//----------------------------------------------------------------------------
void fillProtocolPattern()
{
    for (int i = 0; i < NC7427_MESSAGELEN; i++)
    {
        byte n = i;

        if (n < 2)
        {
            p.d.lead += (buf[i] == 1) ? 1 << i : 0;
        }
        else if (n >= 2 && n < 10)
        {
            p.d.id |= (buf[i] == 1) ? 1 << (i - 2) : 0;
        }
        else if (n >= 10 && n < 12)
        {
            p.d.bat |= (buf[i] == 1) ? 1 << (i - 10) : 0;
        }
        else if (n >= 12 && n < 14)
        {
            p.d.chan |= (buf[i] == 1) ? 1 << (i - 12) : 0;
        }
        else if (n >= 14 && n < 26)
        {
            p.d.temp |= (buf[i] == 1) ? 1 << (i - 14) : 0;
        }
        else if (n >= 26 && n < 34)
        {
            p.d.hum |= (buf[i] == 1) ? 1 << (i - 26) : 0;
        }
        else if (n >= 34)
        {
            p.d.crc |= (buf[i] == 1) ? 1 << (i - 34) : 0;
        };

    }
}
//----------------------------------------------------------------------------
void swapArrayBinPos(byte last, byte first)
{
    byte buf2[NC7427_MESSAGELEN];
    //byte tmp = 0;
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
//----------------------------------------------------------------------------
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
//----------------------------------------------------------------------------
void printLongRaw(uint64_t raw)
{
    /*      // is 64bit int*/
    uint64_t ll = raw;
    uint64_t xx = ll / 1000000000ULL;

    printf("raw:\t");
    if (xx > 0) Serial.print((long)xx);
    Serial.print((long)(ll - xx * 1000000000));
    //Serial.println();
}
//----------------------------------------------------------------------------
int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
//----------------------------------------------------------------------------
//--- function that printf and related will use to print
int serial_putchar(char c, FILE* f)
{
    if (c == '\n') serial_putchar('\r', f);
    return Serial.write(c) == 1 ? 0 : 1;
}
//----------------------------------------------------------------------------
void printBufferRawValue()
{
    var = 0;
    pvar = &var;
    //--- read ringbuffer 
    uint8_t ret = BufferOut(pvar);
    if (ret == BUFFER_SUCCESS)
    {
        p.raw = *pvar;
    }
}
//----------------------------------------------------------------------------
void printBuffer()
{
    var = 0;
    pvar = &var;

    uint8_t ret = BufferOut(pvar);
    if (ret == BUFFER_SUCCESS)
    {
        //printf("\n------------------------------------------------------------\n");
        printf("\n");
        printBits(sizeof(uint64_t), &p.raw);

        //printNonSwappedBuffer();
        //printSwappedBuffer();

        // fill data-structure 
        p.raw = *pvar;

        //--- report readings 
        printf("ld:\t%d\n", p.d.lead);
        printf("id:\t%d\n", p.d.id);
        printf("id_c:\t%d\n", p.d.id & 127); //-- skip upper id-bit to fit in LaCrosse-Id (max): 127 allowed. 
        printf("bat:\t%d\n", p.d.bat);
        printf("ch:\t%d\n", p.d.chan);

        //////////////////////////////////////////////////////////////////        
        //--- temperature, juggling bitpositions  (;-(( 
        //--- there might be a better solution, this one works. 
        //////////////////////////////////////////////////////////////////        
        uint16_t tbits = (uint16_t)p.d.temp;
        uint16_t tbitsL = (tbits & 0b111100000000) >> 8;
        uint16_t tbitsM = tbits & 0b000011110000;
        uint16_t tbitsH = (tbits & 0b000000001111) << 8;
        uint16_t tempF = (tbitsH + tbitsM + tbitsL);        //--- join inversed nibbles       

        //////////////////////////////////////////////////////////////////        
        //--- conversion FahrenheitToCelsius => C = (F-32) / 1.8
        //--- other:  °C = map(°F, 32, 212, 0, 100)
        //--- and protocol specific offset
        //////////////////////////////////////////////////////////////////        
        double tempC = (double)tempF - 900;
        tempC = ((tempC / 10) - 32) / 1.8;

        //////////////////////////////////////////////////////////////////        

        printf("tempF:\t 0x%X \n", tempF);
        printf("tempC:\t "); Serial.println(tempC);

        //////////////////////////////////////////////////////////////////                

        printf("hum:\t%d\n", p.d.hum);
        printf("crc:\t%d\n", p.d.crc);

        printLongRaw(p.raw); puts("");

        printf("------------------------------------------------------------\n");
        byte lacrosse_Id = p.d.id & 127; // strip msb while unsupported in LaCrosse
        LaCrosseSend(lacrosse_Id, tempC, (double)p.d.hum);
        delay(1000); //2 Sekunden Pause zwischen den Repetitions 
        printf("------------------------------------------------------------\n");

    }
    //else
    //{
    //    //printf("*** no data. \n");
    //}

}
//-------------------------------------------------------------------------
void printNonSwappedBuffer()
{
    printf("\n---------------------------------------------------------------------------------------\n");
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
    printf("\n---------------------------------------------------------------------------------------\n");
}
//-------------------------------------------------------------------------
void printSwappedBuffer()
{
    printf("\n----[swapped]--------------------------------------------------------------------------\n");
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
    printf("\n---------------------------------------------------------------------------------------\n");
}
//-------------------------------------------------------------
void LaCrosseSend(byte sensorId, double temperature, double humidity)
{
    LaCrosse.t = temperature;
    LaCrosse.bSensorId = sensorId;
    LaCrosse.setSensorId(sensorId);
    digitalWrite(PIN_LED, HIGH);
    LaCrosse.sendTemperature();
    digitalWrite(PIN_LED, LOW);

    LaCrosse.sleep(1); 

    LaCrosse.h = humidity;
    LaCrosse.bSensorId = sensorId;
    LaCrosse.setSensorId(sensorId);
    digitalWrite(PIN_LED, HIGH);
    LaCrosse.sendHumidity();
    digitalWrite(PIN_LED, LOW);


    printf("FUNC \tID: %d [%d]\t", sensorId, LaCrosse.bSensorId);
    Serial.print("T: "); Serial.print((double)temperature); printf("\t");
    Serial.print("H: "); Serial.println((double)humidity);


    printf("TX \tID: %d\t", sensorId);
    Serial.print("T: "); Serial.print((double)LaCrosse.t); printf("\t");
    Serial.print("H: "); Serial.println((double)LaCrosse.h);
    //%d \t- T:%e  \t-H: %e\n", sensorId, temperature, humidity); 


}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
