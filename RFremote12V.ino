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

#define RF_TX_PIN 0 // the TxPin applied to the 433MHz modulator pin
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

const long codel = 2180L / 4; // us
//const int RF_TX_PIN = RF_TX_PIN; // the TxPin applied to the 433MHz modulator pin

// EEPROM variables
// counter kept in EEPROM to keep tack of the last sent code

//uint16_e vrefl EEMEM; // 
//uint16_e vrefr EEMEM; // 
//uint8_e idx_pos1 EEMEM; // 
//uint8_e idx_pos2 EEMEM; // 
//uint8_e idx1t EEMEM; // 
uint16_e ee_rfc_idx EEMEM; // 

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


// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {

    cbi(ADCSRA, ADEN);                    // switch Analog to Digital converter OFF
    digitalWrite(RF_TX_PIN, LOW);       // turn off modulator

    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();
    sleep_mode();                        // System actually sleeps here
    sleep_disable();                     // System continues execution here when watchdog timed out 
    sbi(ADCSRA, ADEN);                    // switch Analog to Digitalconverter ON
}

bool first_time = true;
int cnt = 0;
uint16_t rfcode_idx = 0;
uint64_t code;
uint64_t codeP;
bool dip1, dip2;

void setup()
{
    pinMode(RF_TX_PIN, OUTPUT);
    pinMode(DIP1_PIN, INPUT_PULLUP);
    pinMode(DIP2_PIN, INPUT_PULLUP);

    DebugBegin(SERIAL_SPEED);
    DebugPrintln("");
    DebugPrintV(F("idx was: "), ee_rfc_idx);
    DebugPrintVln(F(" / "), n_msgsP);

    dip1 = digitalRead(DIP1_PIN);
    dip2 = digitalRead(DIP2_PIN);

    if(dip1) // if jumper1 off - advance to next code
        ee_rfc_idx++;
    // if jumper1 on - keep transmitting the same code

    if (!dip2) // if jumper1 on - reset code counter to 0 - start codes list from begining
        ee_rfc_idx = 0;

    if (ee_rfc_idx >= n_msgsP)
        ee_rfc_idx = 0;

    DebugPrintVln("idx  is: ", ee_rfc_idx);

    //code = msgs[ee_rfc_idx];
    memcpy_P(&codeP, &msgsP[ee_rfc_idx], sizeof(codeP));
    code = codeP;
        
    DebugPrint(F("code:"));
    DebugPrintF((long)(code>>32), HEX);
    DebugPrintFln((long)code, HEX);
    DebugPrint(F("codeP:"));
    DebugPrintF((long)(codeP >> 32), HEX);
    DebugPrintFln((long)codeP, HEX);

    //rfcode_idx = ee_rfc_idx;
    //Serial.print(F("t_size: "));
    //Serial.print(n_msgsC);
    //Serial.print(F(" "));
    //Serial.println(n_msgs);
}

uint8_t upcounter = 0xfe;
uint8_t downcounter = 0xff;
const uint8_t msglen = 52;
bool firsttime = true;

void loop()
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

    DebugPrintF(cnt,HEX);
    cnt++;

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
