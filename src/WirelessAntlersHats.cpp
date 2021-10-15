// **********************************************************************************
// Code for Radio City Music Hall Wireless Antlers Hats module
// **********************************************************************************
// Copyright 2021 Radio City Music Hall
// Contact: Michael Sauder, michael.sauder@msg.com

// **********************************************************************************
// Changelog:
// 
// **********************************************************************************

#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex

//#define NODEID       203  // node ID used for this unit
//#define NETWORKID    150
#define GATEWAY1     1
#define GATEWAY2     2
#define GATEWAY3     3
#define BROADCASTID  0

//#define FREQUENCY     RF69_915MHZ
//#define FREQUENCY_EXACT 915000000
//#define ENCRYPTKEY  "rcmhprodrcmhprod" //16-bytes or ""/0/null for no encryption
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define FLASH_ID      0xEF30  //ID for the 4Mbit Winbond W25X40CL flash chip

#define SERIAL_BAUD 115200
#define BLINKPERIOD 250
#define DEBUG_MODE true //uncomment to enable debug comments

//#define VERSION     1 // What version software are we running?
#define ANTLER_PIN  6 //PWM pin for controlling antler LEDs


// *****************************************************************************************************************************
// Setup battery monitoring
#define BATTERY_PIN     A7 //Analog pin for reading battery voltage
int analogBatteryValue;     //Analog value read on battery voltage divider pin BATTERY_PIN
float tempVoltage;          //temporary value
float actualBatteryVoltage; // calculated battery voltage
// *****************************************************************************************************************************

byte nodeID; // ID for this specific node.
byte codeVersion; // Code version stored in configuration
byte currentState; // What is the current state of this module?
bool antlerState; // What state are the antlers currently in?

SPIFlash flash(SS_FLASHMEM, FLASH_ID);

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char input = 0;
long lastPeriod = -1;

// struct for EEPROM config
struct configuration {
  byte frequency; // What family are we working in? Basically always going to be 915Mhz in RCMH.
  long frequency_exact; // The exact frequency we're operating at.
  byte isHW;
  byte nodeID;    // 8bit address (up to 255)
  byte networkID; // 8bit address (up to 255)
  byte gatewayID; // 8bit address (up to 255)
  char encryptionKey[16];
  byte state;     // Just in case we want to save a state setting.
  byte codeVersion; // What version code we're using
} CONFIG;

// struct for packets being sent to antler hats
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  state; // What state are we being told to go into?
  bool  antlerState; // Used if we want to overwrite pre-defined states
  bool  antlerStateUse; // Should we pay attention to the incoming Antler state?
  long  sleepTime; // In milliseconds. Used if we want to overwrite pre-defined states
  bool  sleepTimeUse; // Should we pay attention to the incoming sleep time?
} ToAntlersPayload;
ToAntlersPayload antlersPayload;

// struct for packets being sent to controllers
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  state; // What state Hat node is currently in
  bool  antlerState; // What state the antlers are currently in
  float vcc; // VCC read from battery monitor
  int   temperature; // Temperature of the radio
} ToControllersPayload;
ToControllersPayload controllersPayload;



//*************************************
// Setup                              *
//*************************************

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ANTLER_PIN, OUTPUT);

  EEPROM.setMaxAllowedWrites(10000);
  EEPROM.readBlock(0, CONFIG);
  
  nodeID = CONFIG.nodeID;
  codeVersion = CONFIG.codeVersion;
  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(1000);
  radio.initialize(CONFIG.frequency,CONFIG.nodeID,CONFIG.networkID);
//  radio.initialize(FREQUENCY,NODEID,NETWORKID);

  radio.setFrequency(CONFIG.frequency_exact); //set frequency to some custom frequency
  radio.encrypt(CONFIG.encryptionKey); //OPTIONAL

  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

  if (CONFIG.isHW) {
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  }

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");

  Serial.print("Starting node: "); Serial.println(nodeID);
  Serial.print("On network ID: "); Serial.println(CONFIG.networkID);
  Serial.print("Using encryption key: "); Serial.println(CONFIG.encryptionKey);
  Serial.print("On frequency: "); Serial.println(CONFIG.frequency_exact);
  Serial.print("Using high power mode: "); Serial.println(CONFIG.isHW);


  #ifdef BR_300KBPS
    radio.writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
    radio.writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
    radio.writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
    radio.writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
    radio.writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
    radio.writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
    radio.writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
  #endif
} // close setup()

//**************************************
// Calculate battery voltage  function *
//**************************************

float BatteryVoltage()
{
    analogBatteryValue = analogRead(BATTERY_PIN);
    tempVoltage = (float) analogBatteryValue / 1024.0 * 3.3;
     // Don't actually understand the exact math, but it's based on the resistor values (10M and 4.7M).
     // Could come back and play around with a constant for 4700.0/14700.0 instead of calculating. Calculating
     // likely more accurate?
     // actualBatteryVoltage = 1.0 / (4700.0 / (4700.0+10000.0)) * tempVoltage;
     actualBatteryVoltage = 1.0 / (4700.0 / 14700.0) * tempVoltage;
    return actualBatteryVoltage;
}

//*************************************
// Blink function                     *
//*************************************

void blinkLED(int blinkTime, int blinkNumber)
{
  // int counter = 0;
  // while (counter <= blinkNumber) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(blinkTime);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(blinkTime);
  //   counter++;
  // }
}

//*************************************
// Send to controllers function       *
//*************************************

void sendControllerPayload(){
  controllersPayload.nodeId = nodeID;
  controllersPayload.version = VERSION;
  controllersPayload.state = currentState;
  controllersPayload.antlerState = antlerState;
  controllersPayload.vcc = BatteryVoltage();
  controllersPayload.temperature = radio.readTemperature();

  radio.send(0, (const void*)(&controllersPayload), sizeof(controllersPayload));
} // close sendControllerPayload()

//*************************************
// Antlers on/off function            *
//*************************************

void antlers(bool state, int level = 0) // False to turn off antlers, true to turn on.
                                        // Level 0-255, 0 is off, anything between 1-254
                                        // will be dimmer but steady. 255 will blink at full.
{
  if (state) {
    antlerState=true;
    analogWrite(ANTLER_PIN, level);
  }
  else {
    antlerState=false;
    analogWrite(ANTLER_PIN, 0);
  }
} // close antlers()

//*************************************
// Loop                               *
//*************************************

void loop(){

  // Check for existing RF data
  if (radio.receiveDone()) {

   #ifdef DEBUG_MODE
      Serial.println();
      Serial.print("Got [");
      Serial.print(radio.SENDERID);
      Serial.print(':');
      Serial.print(radio.DATALEN);
      Serial.print("] > ");
      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i], HEX);
      Serial.println();
    #endif

    // Check for a new OTA sketch. If so, update will be applied and unit restarted.
    CheckForWirelessHEX(radio, flash, false);
    //Serial.println("Checked for OTA update");

    // Check if sender wanted an ACK
    if (radio.ACKRequested())
    {
      radio.sendACK();
      #ifdef DEBUG_MODE
        Serial.print(" - ACK sent");
      #endif
    }
 
    // Check if valid packet. In future perhaps add checking for different payload versions
    // if (radio.DATALEN != (uint8_t)sizeof(ToAntlersPayload)) {
    //   #ifdef DEBUG_MODE
    //     int bob = sizeof(radio.DATALEN);
    //     int joe = sizeof(ToAntlersPayload);
    //     Serial.println("Invalid payload received, not matching Payload struct!");
    //     Serial.print("Payload size was: "); Serial.println(bob);
    //     Serial.print("Payload size needs to be: "); Serial.println(joe);
    //   #endif
    // }
    // else
    // {

      antlersPayload = *(ToAntlersPayload*)radio.DATA; // We'll hope radio.DATA actually contains our struct and not something else

      #ifdef DEBUG_MODE
        Serial.println("Received data from node: ");
        Serial.println(antlersPayload.nodeId);
        Serial.println("Received data is payload version: ");
        Serial.println(antlersPayload.version);
        Serial.println("Received state is: ");
        Serial.println(antlersPayload.state);
        Serial.println("Received antler state is: ");
        Serial.println(antlersPayload.antlerState);
        Serial.println("Received sleep time is: ");
        Serial.println(antlersPayload.sleepTime);
      #endif

      //And now for the real meat, what should we be doing right now?

      if (antlersPayload.state != currentState) {
        switch (int(antlersPayload.state)) {
          case 1:
            // This is our deep sleep state
            // We want to tell the MCU and radio to go to sleep and, if necessary, implement a counter 
            // that goes longer than 8 seconds (default arduino limit).
            antlers(false,0);
            digitalWrite(LED_BUILTIN, LOW);
            currentState = 1;
            sendControllerPayload();
            #ifdef DEBUG_MODE
              Serial.println("Entered state 1");
            #endif
            break;
          case 2:
            // This is our wake up and pay some attention state
            // We want to shorten our sleep time to perhaps 1 second off, 20ms on? Experiment with on time.
            // Probably the state to go in with the handheld remotes.
            // Probably want to go back to sleep after say 2 minutes without a state change.
            antlers(false,0);
            digitalWrite(LED_BUILTIN, LOW);
            currentState = 2;
            sendControllerPayload();
            #ifdef DEBUG_MODE
              Serial.println("Entered state 2");
            #endif
            break;
          case 3:
            // This is our standby to do stuff really soon state. 
            // We want to stay on full time, but go back to sleep after 2 minutes without a state change.
            antlers(false,0);
            digitalWrite(LED_BUILTIN, LOW);
            currentState = 3;
            sendControllerPayload();
            #ifdef DEBUG_MODE
              Serial.println("Entered state 3");
            #endif
            break;
          case 4:
            // This is our GO GO GO state
            antlers(true,255);
            digitalWrite(LED_BUILTIN, HIGH);
            currentState = 4;
            sendControllerPayload();
            #ifdef DEBUG_MODE
              Serial.println("Entered state 4");
            #endif
            break;
          case 5:
            // This is the state where we experiment with keeping the antlers on
            // while sleeping the MCU and radio
            break;
          case 6:
            // This is a TBD state
            break;
          case 7:
            // Another TBD state. Perhaps for custom timings and antler states?
            break;
          case 8:
            // Another TBD state. Perhaps for custom timings and antler states?
            break;  
          case 9:
            // This is our programming state. Major power hog. Programming takes time, not sure yet if want to auto sleep
            antlers(0,0);
            digitalWrite(LED_BUILTIN, LOW);
            currentState = 9;
            sendControllerPayload();
            #ifdef DEBUG_MODE
              Serial.println("Entered state 9");
            #endif
            break;
       } // Close switch
      } // close if state change
    //} // close if valid payload
  } // close radio receive done

//*****************************************************************************************************************************
// Can play around with dimming the LEDs. If the antlers have analogWrite value of 255, they will blink at their brightest.
// If the analogWrite value is less than 255, they stop blinking, but you can control their brightness.
//
//          delay(2000);
//          analogWrite(RX_TOGGLE_PIN, 200);
//          delay(2000);
//          analogWrite(RX_TOGGLE_PIN, 150);
//        for(int i=0; i<255; i++){
//          analogWrite(RX_TOGGLE_PIN, i);
//          delay(5);
//        }
//        for(int i=255; i>0; i--){
//          analogWrite(RX_TOGGLE_PIN, i);
//          delay(5);
//        }
          
    //} // close valid payload
  //} // close radio.receiveDone()

  //  if ((int)(millis()/BLINKPERIOD) > lastPeriod)
  // {
  //   lastPeriod++;
  //   digitalWrite(LED_BUILTIN, lastPeriod%2);
  //   Serial.print("BLINKPERIOD ");Serial.println(BLINKPERIOD);
  // }

  // if (Serial.available() > 0) {
  //   Serial.println("Serial is available");
  //   input = Serial.parseInt();
  //   //int intInput = input - '0'

  //   if (input >= 1 && input <= 9) { //0-9
  //     Serial.print("\nSending state "); Serial.println(input);
  //     sendControllerPayload();
  //   } 
  // } // close serial

} // close loop()



