/*
    Name:       RFremotee12V.ino
    Created:	19.04.2021 
    Author:     DESKTOP-I9\Dejan
*/

#include <avr/io.h> 
#include <avr/sleep.h>
#include <EEPROM.h>
#include <EEWrap.h>
#include "rfcodes.h"

#define RF_TX_PIN 0 // the RF_TX_PIN applied to the 433MHz modulator pin
#define DIP1_PIN 1 // DIPswitch/jumper pin 1
#define DIP2_PIN 2 // DIPswitch/jumper pin 2

//#define DEBUG

#ifdef DEBUG
#include <SoftwareSerial.h>
#define SERIAL_SPEED 9600
#define RX_PIN 0
#define TX_PIN 3
SoftwareSerial MySerial(RX_PIN, TX_PIN);
#define Serial MySerial

#define DebugBegin(s) MySerial.begin(s);
#define DebugPrint(x) MySerial.print(x);
#define DebugPrintln(x) MySerial.println(x);
#define DebugPrintF(x, y) MySerial.print(x, y);
#define DebugPrintFln(x, y) MySerial.println(x, y);
#define DebugPrintV(x, y) MySerial.print(x); MySerial.print(y); 
#define DebugPrintVln(x, y) MySerial.print(x); MySerial.println(y); 
#else
#define DebugBegin(s) ;
#define DebugPrint(x) ;
#define DebugPrintln(x) ;
#define DebugPrintF(x, y) ;
#define DebugPrintFln(x, y) ;
#define DebugPrintV(x, y) ; 
#define DebugPrintVln(x, y) ; 
#endif

// Routines to set and clear bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// EEPROM variables
// counters kept in EEPROM to keep tack of the last sent code
uint16_e ee_rfc_idx1 EEMEM; // 
uint16_e ee_rfc_idx2 EEMEM; // 

void transmitT(uint64_t message, uint8_t mlen, int wakeup = 11, int waitwake = 95)
{
    const unsigned int syncup = 1475; // 1450;
    const unsigned int syncdown = 1675; // 1650;
    const unsigned int per = 1586; // 1560;
    const unsigned int oneup = 31UL * per / 52;
    const unsigned int onedown = 21UL * per / 52;
    const unsigned int zeroup = 14UL * per / 52;
    const unsigned int zerodown = 38UL * per / 52;

    DebugPrint(F("T:"));
    DebugPrintF((long)(message >> 32), HEX);
    DebugPrintFln((long)message, HEX);

    if (wakeup > 0)
    {
        digitalWrite(RF_TX_PIN, HIGH);
        delay(wakeup);
        digitalWrite(RF_TX_PIN, LOW);
        delay(waitwake);
    }

    // send message
    // send sync 111000
    digitalWrite(RF_TX_PIN, HIGH);
    delayMicroseconds(syncup);
    digitalWrite(RF_TX_PIN, LOW);
    delayMicroseconds(syncdown);

    // send content
    uint64_t mask = 1ULL << (mlen - 1);
    while (mask)
    {
        if (message & mask)
        {
            // transmit 1 - long hi (930u) followed by shorter low (640us) 
            digitalWrite(RF_TX_PIN, HIGH);
            delayMicroseconds(oneup);
            digitalWrite(RF_TX_PIN, LOW);
            delayMicroseconds(onedown);
        }
        else
        {
            // transmit 0 - short hi (420u) followed by long low (1140us)
            digitalWrite(RF_TX_PIN, HIGH);
            delayMicroseconds(zeroup);
            digitalWrite(RF_TX_PIN, LOW);
            delayMicroseconds(zerodown);
        }
        mask >>= 1;
    }
    // send sync 111000
    digitalWrite(RF_TX_PIN, HIGH);
    delayMicroseconds(syncup);
    digitalWrite(RF_TX_PIN, LOW);
    delayMicroseconds(syncdown);

    delay(17); //17.2 exact
}

void transmitC(uint32_t message)
{
    const unsigned int syncup = 390;
    const unsigned int syncdown = 430; //440;
    const unsigned int pause_after_sync = 3600; //3720
    const unsigned int per = 1233; //1248
    const unsigned int oneup = 2 * per / 3; // 808 us exact
    const unsigned int onedown = 1 * per / 3; // 440 us exact
    const unsigned int zeroup = 1 * per / 3; // 393 us exact
    const unsigned int zerodown = 2 * per / 3; // 855 us exact
    const uint64_t prologue = 0b1100101011101110110010101110101111;

    DebugPrint(F("T:"));
    DebugPrintFln((long)message, HEX);

    // send sync 101010101010101010101010
    for (int i = 0; i < 12; i++)
    {
        digitalWrite(RF_TX_PIN, HIGH);
        delayMicroseconds(syncup);
        digitalWrite(RF_TX_PIN, LOW);
        delayMicroseconds(syncdown);
    }
    delayMicroseconds(pause_after_sync);

    // send content
    uint32_t mask = 0x80000000;
    while (mask)
    {
        if (message & mask)
        {
            // transmit 1 - long hi (810u) followed by shorter low (440us) (actually 0)
            digitalWrite(RF_TX_PIN, HIGH);
            delayMicroseconds(oneup);
            digitalWrite(RF_TX_PIN, LOW);
            delayMicroseconds(onedown);
        }
        else
        {
            // transmit 0 - short hi (395u) followed by long low (855us) (actually 1)
            digitalWrite(RF_TX_PIN, HIGH);
            delayMicroseconds(zeroup);
            digitalWrite(RF_TX_PIN, LOW);
            delayMicroseconds(zerodown);
        }
        mask >>= 1;
    }

    //    prologue = 0b1100101011101110110010101110101111;
    uint64_t pmask = 0b1000000000000000000000000000000000;
    while (pmask)
    {
        if (prologue & pmask)
        {
            // transmit 1 - long hi (810u) followed by shorter low (440us) 
            digitalWrite(RF_TX_PIN, HIGH);
            delayMicroseconds(oneup);
            digitalWrite(RF_TX_PIN, LOW);
            delayMicroseconds(onedown);
        }
        else
        {
            // transmit 0 - short hi (395u) followed by long low (855us)
            digitalWrite(RF_TX_PIN, HIGH);
            delayMicroseconds(zeroup);
            digitalWrite(RF_TX_PIN, LOW);
            delayMicroseconds(zerodown);
        }
        pmask >>= 1;
    }
    delay(16); //16.250 exact
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() 
{
    cbi(ADCSRA, ADEN);                    // switch Analog to Digital converter OFF
    digitalWrite(RF_TX_PIN, LOW);       // turn off modulator

    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();
    sleep_mode();                        // System actually sleeps here
    sleep_disable();                     // System continues execution here when watchdog timed out 
    sbi(ADCSRA, ADEN);                    // switch Analog to Digitalconverter ON
}

bool first_time = true;
bool remote;
int cnt = 0;
uint64_t code;
uint32_t codeC;
bool dip1, dip2;

void setup()
{
    pinMode(RF_TX_PIN, OUTPUT);
    pinMode(DIP1_PIN, INPUT_PULLUP);
    pinMode(DIP2_PIN, INPUT_PULLUP);

    DebugBegin(SERIAL_SPEED);
    DebugPrintln("");
    DebugPrintV(F("idx was: "), ee_rfc_idx1);
    DebugPrintVln(F(" / "), n_msgsP);

    dip1 = digitalRead(DIP1_PIN);
    dip2 = digitalRead(DIP2_PIN);

    remote = dip1;

    if (remote)
    {
        ee_rfc_idx1++; // advance to next code
        if (ee_rfc_idx1 >= n_msgsP)
            ee_rfc_idx1 = 0;
        if (!dip2) // if jumper1 on - reset code counter to 0 - start codes list from begining
            ee_rfc_idx1 = 0;

        DebugPrintVln("idx  is: ", ee_rfc_idx1);

        memcpy_P(&code, &msgsP[ee_rfc_idx1], sizeof(code));

        DebugPrint(F("code:"));
        DebugPrintF((long)(code >> 32), HEX);
        DebugPrintFln((long)code, HEX);
    }
    else
    {
        ee_rfc_idx2++; // advance to next code
        if (ee_rfc_idx2 >= n_msgsC)
            ee_rfc_idx2 = 0;
        if (!dip2) // if jumper1 on - reset code counter to 0 - start codes list from begining
            ee_rfc_idx2 = 0;

        DebugPrintVln("idx  is: ", ee_rfc_idx2);

        memcpy_P(&codeC, &msgsC[ee_rfc_idx2], sizeof(codeC));

        DebugPrint(F("code:"));
        DebugPrintFln((long)codeC, HEX);
    }
}

uint8_t upcounter = 0xfe;
uint8_t downcounter = 0xff;
const uint8_t msglen = 52;
bool firsttime = true;

void loop()
{
    if (remote)
    {
        upcounter++;
        downcounter--;
        uint64_t ins = (downcounter & 0b1100) | (upcounter & 0b11) | 0b00010000;

        ins <<= 44;
        if (firsttime)
        {
            transmitT(code | ins, msglen);
            firsttime = false;
        }
        else
        {
            transmitT(code | ins, msglen, 0);
        }
    }
    else
    {
        transmitC(codeC);
    }

    cnt++;
    DebugPrintF(cnt, HEX);

    if (cnt > 15) // go to sleep after sending the code 16 times even if the button is still pressed
    {
#ifdef DEBUG
        while (1)
        {
            delay(5000);
            DebugPrint('.');
        }
#else
        system_sleep();
#endif
    }

}
