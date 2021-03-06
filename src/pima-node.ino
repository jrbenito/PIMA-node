// PIMA energy meter protocol Node
//
// This node reads PIMA protocol from compatible energy meters (like Nansen and others
// sold to Brasilian market) and transmits data to a central node/Gateway using RFM69
// radio.
// 
/////////////////////////////////////////////////////////////////////////////////////
// Copyright Josenivaldo Benito Junior 2017
// http://benito.com.br/
/////////////////////////////////////////////////////////////////////////////////////
// License
/////////////////////////////////////////////////////////////////////////////////////
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
/////////////////////////////////////////////////////////////////////////////////////
#include <avr/wdt.h>
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlashA.h>      //get it here: https://github.com/lowpowerlab/spiflash
#define SPIFlash SPIFlashA
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <SoftwareSerial.h> // SoftSerial
#include <Thread.h>        // execution threads
#include <ThreadController.h>
#include <device.h>        // Computurist message format
#include <pima-node.h>      // headers for system functions

#define VERSION "PIMA V0.1"
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        3     // node ID used for this unit
#define NETWORKID   100
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY_EXACT 916000000
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -72
//*****************************************************************************************************************************
//#define BR_300KBPS             //run radio at max rate of 300kbps!
//*****************************************************************************************************************************
#define ACK_TIME    30  // # of ms to wait for an ack
#define ENCRYPTKEY "sampleEncryptKey" //(16 bytes of your choice - keep the same on all encrypted nodes)
#define RETRIES 5
//*****************************************************************************************************************************

// Onboard LED and flash chip select pins
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Miniwirless have LEDs on D9
  #define FLASH_SS      5 // and FLASH SS on D5
#endif


#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

/**************************************
device settings
**************************************/
#define DEBUG
#if defined(DEBUG)
//  SoftwareSerial mySerial(6, 7); //verify pins available
//  #define DebugSer(a) (Serial.print(a))
//  #define DebugSerln(a) (Serial.println(a))
//  #define DebugSerbg(a) (Serial.begin(a))
  SoftwareSerial mySerial(7, 6); //verify pins available
  #define DebugSer(a) (mySerial.print(a))
  #define DebugSerln(a) (mySerial.println(a))
  #define DebugSerbg(a) (mySerial.begin(a))
#else
//  SoftwareSerial mySerial(6, 7); //verify pins available
  #define mySerial Serial
  #define DebugSer(a)
  #define DebugSerln(a)
  #define DebugSerbg(a)
#endif

#define PIMASer Serial//mySerial //Serial // user hardware serial RX to attach PIMA
#define ESCOPO 0x0A
#define EATIVA_D 0x02
#define EREATIVAI_D 0x07
#define EREATIVAC_D 0x0C
#define EATIVA_R 0x51 

SPIFlash flash(FLASH_SS, 0x12018); //12018 for 128Mbit Spanion flash

/**************************************
  global variables
  **************************************/
int     TXinterval = 45;            // periodic transmission interval in seconds
bool    setACK = false;             // send ACK message on 'SET' request
bool    toggle = false;
bool    updatesSent = false;
Power power;

volatile long wdtCounter = 0;
#define MAX_PKG_SIZE 20
uint8_t PIMAPacket[MAX_PKG_SIZE];
uint32_t SerialNumber = -1;
int CRC16L, CRC16H;

const Message DEFAULT_MSG = {NODEID, 0, 0, 0, 0, VERSION};

/**************************************
  configure devices
  **************************************/

//Device name(devID, tx_periodically, read_function, optional_write_function)

Device uptimeDev(0, true, readUptime);
Device txIntDev(1, false, readTXInt, writeTXInt);
Device rssiDev(2, false, readRSSI);
Device verDev(3, false);
Device voltDev(4, false, readVoltage);
Device ackDev(5, false, readACK, writeACK);
Device toggleDev(6, false, readToggle, writeToggle);
Device ledDev(16, false, readLED, writeLED);
Device pwrADirDev(64, true, readPwrADir);
Device pwrRIndDev(65, true, readPwrRInd);
Device pwrRCapDev(66, true, readPwrRCap);
Device pwrARevDev(67, true, readPwrARev);

//ThreadController controll = ThreadController();
//Thread blinkLed = Thread();
static Device devices[] = {uptimeDev, txIntDev, rssiDev, verDev,
                    voltDev, ackDev, toggleDev, ledDev, pwrADirDev,
                    pwrRIndDev, pwrRCapDev, pwrARevDev};

/*******************************************
put non-system read/write functions here
********************************************/

void readLED(Message *mess) {
    digitalRead(LED) ? mess->intVal = 1 : mess->intVal = 0;
}

void writeLED(const Message *mess) {
    digitalWrite(LED, mess->intVal);
}

void readPwrADir(Message *mess) {
    mess->intVal = power.pwrActivDir;
}

void readPwrRInd(Message *mess) {
    mess->intVal = power.pwrReactInd;
}

void readPwrRCap(Message *mess) {
    mess->intVal = power.pwrReactCap;
}

void readPwrARev(Message *mess) {
    mess->intVal = power.pwrActivRev;
}

void receivePIMAPacket() {

    uint8_t c;
    static uint8_t pkgLength = MAX_PKG_SIZE-6;
    static int8_t i = -2;
    // Serial packet starts with 0xAA55 (2 bytes)
    // followed by serial number (5 bytes)
    // hence, at least 7 bytes might be available
    while (PIMASer.available() > 0) { 
        c = PIMASer.read();
        // Wating for preamble
        if ((i < -1) && (c == 0xAA)) {
            i++;
        }
        else if ((i < 0) && (c == 0x55)) {
            i++;
        }
        else if ((i >=0)) {
            PIMAPacket[i] = c;
            i++;
            // next byte after serial number define the length of data
            // (6th byte - not count preamble)
            if (i == 6) {
                pkgLength = c;
                if ((pkgLength+i) > MAX_PKG_SIZE) {
                    //invalid package length
                    DebugSerln(F("Invalid Packet!"));
                    i = -2;
                    pkgLength = MAX_PKG_SIZE-6;
                    break;
                }
            }
            else if (i >= (pkgLength + 6 + 2)) { // +2 from CRC
                // package complete
                DebugSerln(F("Pkt OK"));
                i = -2;
                pkgLength = MAX_PKG_SIZE-6;
                processPIMAPacket();
                break;
            }
            else if (i >= MAX_PKG_SIZE) {
                i = -2;
                pkgLength = MAX_PKG_SIZE-6; 
                break;
            }
        }
        else {
            DebugSer(F("No Preamble! "));
            DebugSerln(c);
            i = -2;
            pkgLength = MAX_PKG_SIZE-6;
        }
    }
}

void processPIMAPacket() {

    int32_t value;
    uint8_t index = 0;
    uint8_t buff[6];

#if 0
    DebugSer(F("RAW: "));
    for (int i = 0; i <12; i++) {
        DebugSer("0123456789ABCDEF"[0x0F & ((unsigned char)PIMAPacket[i]>>4)]);
        DebugSer("0123456789ABCDEF"[0x0F & (unsigned char)PIMAPacket[i]]);
    }
    DebugSerln(".");
#endif

    // First 5 bytes are the 10 digit serial number (BCD)
    SerialNumber = 0;
    for(int i = 0; i < 5; i++) {
        buff[i] = bcd2dec(PIMAPacket[i]);
        SerialNumber = SerialNumber * 100 + (uint32_t)buff[i];
    }
    DebugSer(F("SN: "));
    DebugSerln(SerialNumber);

    // the 6th byte is size, 7th and 8th are scope and index
    // registers for KWh and KVAh are all 3 bytes BCD, hence,
    // size cannot be larger than 5 (2 escope+index and 3 data)
    if ((PIMAPacket[6] == ESCOPO) && (PIMAPacket[5] >= 5)) {

        // take the CRC bytes
        CRC16H = PIMAPacket[11];
        CRC16L = PIMAPacket[12];

        // catch index and convert BCD to integer
        index   = PIMAPacket[7];
        value   = 0;
        buff[0] = bcd2dec(PIMAPacket[8]);
        value = value * 100 + (uint32_t)buff[0];
        buff[1] = bcd2dec(PIMAPacket[9]);
        value = value * 100 + (uint32_t)buff[1];
        buff[2] = bcd2dec(PIMAPacket[10]);
        value = value * 100 + (uint32_t)buff[2];

        switch(index) {
            case EATIVA_D:
                power.pwrActivDir = value;
                break;
            case EREATIVAI_D:
                power.pwrReactInd = value;
                break;
            case EREATIVAC_D:
                power.pwrReactCap = value;
                break;
            case EATIVA_R:
                power.pwrActivRev = value;
                break;
            default:
                break;
        }
    }
    else {
        DebugSer(F("Error escope "));
        DebugSer(PIMAPacket[6]);
        DebugSer(F(" size "));
        DebugSerln(PIMAPacket[5]);
    }
}

/******************************************/

void setup() {
	//disable watchdog timer during setup
	wdt_disable();

	//set all pins as input with pullups, floating pins can waste power
	//DDRD &= B00100011;       // set Arduino pins 2 to 7 as inputs, leaves 0 & 1 (RX & TX) and 5 as is
	//DDRB &= B11111110;        // set pins 8 to input, leave others alone since they are used by SPI and OSC
	//PORTD |= B11011100;      // enable pullups on pins 2 to 7, leave pins 0 and 1 alone
	//PORTB |= B00000001;      // enable pullups on pin 8 leave others alone

    // Initialize I/O
    pinMode(LED, OUTPUT);     // ensures LED is output
    digitalWrite(LED, LOW);   // LED off

	// Radio setup
	radio.initialize(FREQUENCY, NODEID, NETWORKID);
	radio.rcCalibration();
	radio.encrypt(ENCRYPTKEY);
#ifdef FREQUENCY_EXACT
	radio.setFrequency(FREQUENCY_EXACT);
#endif
#ifdef ENABLE_ATC
	radio.enableAutoPower(ATC_RSSI);
#endif
#ifdef IS_RFM69HW_HCW
	radio.setHighPower();
#endif
#ifdef BR_300KBPS
	radio.writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
	radio.writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
	radio.writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
	radio.writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
	radio.writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
	radio.writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
	radio.writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
#endif
    
    DebugSerbg(19200);

	flash.initialize();
	PIMASer.begin(2400);
	//PIMASer.flush();

    power.pwrActivDir = -1;
    power.pwrReactInd = -1;
    power.pwrReactCap = -1;
    power.pwrActivRev = -1;

	//configure watchdog as 1s counter for uptime and to wake from sleep 
	watchdogSetup();	

	//send wakeup message
	Message wakeup = DEFAULT_MSG;
	wakeup.devID = 99;
	txRadio(&wakeup);
}

void loop() {
    Message reply = DEFAULT_MSG;

	// Check for existing RF data, potentially for a new sketch wireless upload
	// For this to work this check has to be done often enough to be
	// picked up when a GATEWAY is trying hard to reach this node for a new sketch wireless upload
	if (radio.receiveDone()) {

        DebugSerln("OTA...");
		CheckForWirelessHEX(radio, flash, false, 9);

		// if we got here, the message was not a FOTA handshake
		if (radio.DATALEN != sizeof(Message)) {
			DebugSerln("INVALID PACKET");
		} else {
		    // change from cast to a byte copy for security and portability
			Message mess = *(Message*)radio.DATA;

			if (radio.ACKRequested()) {
				DebugSerln("sending ack");
				radio.sendACK();
			} else {
				DebugSerln("ack not requested");
			}
			bool match = false;

			//check if message is for any devices registered on node
			for (unsigned int i = 0; i < sizeof(devices) / sizeof(Device); i++) {
				if (mess.devID == devices[i].getId()) {
					match = true;
					reply.devID = devices[i].getId();
					//write for cmd 0
					if (mess.cmd == 0) {
						devices[i].write(&mess);
#ifdef DEBUG
						DebugSer("writing node ");
						DebugSer(mess.nodeID);
						DebugSer(" dev ");
						DebugSerln(mess.devID);
#endif
						if (setACK) {
							DebugSerln(reply.devID);
							devices[i].read(&reply);
							txRadio(&reply);
						}
						//read for cmd 1
					} else if (mess.cmd == 1) {
						devices[i].read(&reply);
#ifdef DEBUG
						DebugSer("reading node ");
						DebugSer(reply.nodeID);
						DebugSer(" dev ");
						DebugSerln(reply.devID);
#endif
						txRadio(&reply);
					}
				}
			}
			//invalid device id in message
			if (!match) {
				reply.devID = 92;
				txRadio(&reply);
			}
		}
	}

    //check if any devices needs to transmit periodic info
    if (!updatesSent && wdtCounter % TXinterval == 0) {
        DebugSerln("Sending periodic updates");
        for (unsigned int i = 0; i < sizeof(devices) / sizeof(Device); i++) {
            if (devices[i].getSetTX()) {
                reply = DEFAULT_MSG;
                reply.devID = devices[i].getId();
                devices[i].read(&reply);
                txRadio(&reply);
            }
        }
        updatesSent = true;
    } else if(wdtCounter % TXinterval != 0) {
        updatesSent = false;
    }

    // Receive and process PIMA packagets
    //if (wdtCounter % 10 == 0) {
    //    DebugSer("PIMA");
        receivePIMAPacket();
    //}

}

void txRadio(Message * mess){
  DebugSer(" message n:");
  DebugSer(mess->nodeID);
  DebugSer(" devID:");
  DebugSer(mess->devID);
  DebugSer(" cmd:");
  DebugSer(mess->cmd);
  DebugSer(" intVal:");
  DebugSer(mess->intVal);
  DebugSer(" payload: ");
  DebugSer(mess->payLoad);
  DebugSerln(" sent...");
  if (!radio.sendWithRetry(GATEWAYID, mess, sizeof(*mess), RETRIES, ACK_TIME)){
    DebugSerln("No connection...");
  }
}

void readUptime(Message *mess){
  mess->intVal = wdtCounter / 60;
}

void readTXInt(Message *mess){
  mess->intVal = TXinterval;
}

void writeTXInt(const Message *mess){
  TXinterval = mess->intVal;
  if (TXinterval <10 && TXinterval !=0) TXinterval = 10;	// minimum interval is 10 seconds
}

void readRSSI(Message *mess){
  mess->intVal = radio.RSSI;
}

void readVoltage(Message *mess){
  long result;					// Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);					// Wait for Vref to settle
  ADCSRA |= _BV(ADSC);				// Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; 			// Back-calculate in mV
  mess->fltVal = float(result/1000.0);		// Voltage in Volt (float)
}

void readACK(Message *mess){
  setACK ? mess->intVal = 1 : mess->intVal = 0;
}

void writeACK(const Message *mess){
  mess->intVal ? setACK = true: setACK = false;
}

void readToggle(Message *mess){
  toggle ? mess->intVal = 1 : mess->intVal = 0;
}

void writeToggle(const Message *mess){
  mess->intVal ? toggle = true: toggle = false;
}

void sleep(){
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  sleep_enable();
//  sleep_bod_disable();
//  sleep_mode();
//  sleep_disable();
}

void watchdogSetup(void){
  cli();
  wdt_reset();
  WDTCSR |=(1<<WDCE) | (1<<WDE);
  //set for 1s
  WDTCSR = (1 <<WDIE) |(1<<WDP2) | (1<<WDP1);
  sei();
}

ISR(WDT_vect) // Watchdog timer interrupt.
{
  wdtCounter++;
}

uint8_t bcd2dec(uint8_t n) {
    return n - 6 * (n >> 4);
}
