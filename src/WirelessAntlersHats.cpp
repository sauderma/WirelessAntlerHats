// **********************************************************************************
// Code for Radio City Music Hall Wireless Antlers Hats module
// **********************************************************************************
// Copyright 2021 Radio City Music Hall
// Contact: Michael Sauder, michael.sauder@msg.com

// **********************************************************************************
// Changelog:
// 
// **********************************************************************************

#include <RFM69_ATC.h>        // https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>        // https://github.com/lowpowerlab/RFM69
#include <RFM69.h>            // https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>         // https://github.com/lowpowerlab/spiflash
#include <EEPROMex.h>         // http://playground.arduino.cc/Code/EEPROMex
#include <SafeStringReader.h> // https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
#include <millisDelay.h>      // https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
#include <LowPower.h>         // https://github.com/rocketscream/Low-Power
//#include <time.h>

// ************************************************************************************
// The defines in this section are saved into EEPROM using the WirelessBaseTarget sketch. 
// Uncomment here to define custom values.
// TODO LOW PRIORITY: write code to check for prescense of custom #defines and use those
// instead of EEPROM values.
// *************************************************************************************

// #define NODEID       203  // This will typically be saved into the EEPROM by the WirelessBaseTarget sketch
#define NETWORKID    150  
#define GATEWAY1     1
#define GATEWAY2     2
#define FREQUENCY    RF69_915MHZ
#define FREQUENCY_EXACT 905500000 // Move away from the default exact 915Mhz, presumably less interference
#define ENCRYPTKEY  "rcmhprodrcmhprod" //16-bytes or ""/0/null for no encryption
#define IS_RFM69HW_HCW //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define VERSION     1 // What version software are we running?

// *****************************************************************************************

#define BROADCASTID  0 // Most or all Moteino documentation says to use 255 for broadcast. 255 IS WRONG!

#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define FLASH_ID      0xEF30  //ID for the 4Mbit Winbond W25X40CL flash chip

#define SERIAL_BAUD 115200
#define BLINKPERIOD 250
//#define DEBUG_MODE true //uncomment to enable debug comments

#define ANTLER_PIN  6 //PWM pin for controlling antler LEDs

// *****************************************************************************************************************************
// Setup battery monitoring
#define BATTERY_PIN     A7 //Analog pin for reading battery voltage
int analogBatteryValue;     //Analog value read on battery voltage divider pin BATTERY_PIN
float tempVoltage;          //temporary value
float actualBatteryVoltage; // calculated battery voltage
// *****************************************************************************************************************************

byte nodeID; // ID of this node.
byte codeVersion; // Code version stored in configuration
byte currentState; // Current state of this module
bool antlerState; // Current state of the antlers

SPIFlash flash(SS_FLASHMEM, FLASH_ID);

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

millisDelay sleepyTimer;

// struct for EEPROM config
struct configuration {
  byte codeVersion; // What version code we're using
  byte frequency; // What family are we working in? Basically always going to be 915Mhz in RCMH.
  long frequency_exact; // The exact frequency we're operating at.
  bool isHW; // Is this a high power radio?
  byte nodeID;    // 8bit address (up to 255)
  byte networkID; // 8bit address (up to 255)
  byte gatewayID1; // 8bit address (up to 255)
  byte gatewayID2; // 8bit address (up to 255)
  char encryptionKey[16];
  int state;     // Just in case we want to save a state setting.
} CONFIG;

// struct for packets being sent to antler hats
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  nodeState; // What state should node go into?
  byte  antlerState; // What state should the actual antlers to be in?
  long sleepTime; // In milliseconds. Used if we want to overwrite pre-defined states
} ToAntlersPayload;
ToAntlersPayload antlersPayload;

// struct for packets being sent to controllers
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  nodeState; // What state Hat node is currently in
  byte  antlerState; // What state the antlers are currently in
  float vcc; // VCC read from battery monitor
  int   temperature; // Temperature of the radio
} ToControllersPayload;
ToControllersPayload controllersPayload;

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
// Send to controllers function       *
//*************************************

void sendControllerPayload(){
  controllersPayload.nodeId = nodeID;
  controllersPayload.version = codeVersion;
  controllersPayload.nodeState = currentState;
  controllersPayload.antlerState = antlerState;
  controllersPayload.vcc = BatteryVoltage();
  controllersPayload.temperature = radio.readTemperature();

  // TODO MEDIUM-HIGH PRIORITY
  // Implement a random timing of the send command so that all 85 nodes announcing state changes
  // don't drown each other out. I believe tranmission takes roughly 5ms per node.
  // Try basing on nodeID

  radio.send(GATEWAY1, (const void*)(&controllersPayload), sizeof(controllersPayload));
  #ifdef DEBUG_MODE
    Serial.println("Sent an RF packet");
  #endif

} // close sendControllerPayload()

//*********************************************************
// Antlers on/off function                                *
// False to turn off antlers, true to turn on.            *
// Level 0-255, 0 is off, anything between 1-254          *
// will be dimmer but steady. 255 will blink at full.     *
//*********************************************************

void antlers(bool state, int level = 0)
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

//************************************
//  PowerDown function
//  sleepTime - 1 for 1 second, 8 for 8 seconds
//  wakeTime - in milliseconds
//************************************
void powerDown(int sleepTime = 8, int wakeTime = 50)
{
  radio.receiveDone(); // Radio must be in receive mode before sleeping.
  radio.sleep();
  if(currentState == 1) {
    #ifdef DEBUG_MODE
      Serial.println("Entering 8s deep sleep");
    #endif
    Serial.flush();
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF) ;
  } else if (currentState == 2 ) {
    #ifdef DEBUG_MODE
      Serial.println("Entering 8s normal sleep");
    #endif
    Serial.flush();
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    } else if (currentState == 3 ) {
    #ifdef DEBUG_MODE
      Serial.println("Entering 1s sleep");
    #endif
    Serial.flush();
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  
  } else if (currentState == 6 ) {
    #ifdef DEBUG_MODE
      Serial.println("Entering 8s sleep while antlers on");
    #endif
    Serial.flush();
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  sleepyTimer.start(wakeTime);

  #ifdef DEBUG_MODE
    Serial.println("Leaving Sleep...");
  #endif
} // close powerDown()


//*************************************
// Go to state                        *
//*************************************
void gotoState(int newstate)
{
      switch (newstate) {
      case 1:
        // This is our deep sleep state.
        // We want to tell the MCU and radio to go to sleep for 8 seconds, wake up for 50ms to check radio,
        // if nothing exciting go back to sleep.
        if (currentState != 1 ) {
          antlers(false,0);
          digitalWrite(LED_BUILTIN, LOW);
          currentState = 1;
          sendControllerPayload();
        }
        #ifdef DEBUG_MODE
          Serial.println("In state 1");
        #endif
        powerDown(8, 50);
        break;
      case 2:
        // This is our normal sleep state. The only difference from deep sleep is that we now send out an update
        // every 8 seconds.
        if (currentState != 2 ) {
          antlers(false,0);
          digitalWrite(LED_BUILTIN, LOW);
          currentState = 2;
        }
        #ifdef DEBUG_MODE
          Serial.println("In state 2");
        #endif
        sendControllerPayload();
        powerDown(8, 50);
        break;
      case 3:
        // This is our wake up and pay some attention state. Wake for 50ms, sleep for 1 second.
        // We will call this 12 seconds before we trigger the antlers ON.
        // TODO PRIORITY 2 - go back to sleep (state 1) after 2 minutes without receiving a state change.
        if (currentState != 3) {
          antlers(false, 0);
          digitalWrite(LED_BUILTIN, LOW);
          currentState = 3;
          sendControllerPayload();
        }
        powerDown(1, 50);
        #ifdef DEBUG_MODE
          Serial.println("In state 3");
        #endif
        break;
      case 4:
        // This is our standby to do stuff really soon state. We'll call this 2 seconds before we trigger the antlers ON.
        // We want to stay on full time, no sleep cycle.
        // TODO PRIORITY 2: go back to sleep after 2 minutes without a received state change.
        if (currentState != 4) {
          sleepyTimer.stop();
          antlers(false,0);
          digitalWrite(LED_BUILTIN, LOW);
          currentState = 4;
          sendControllerPayload();
        }
        #ifdef DEBUG_MODE
          Serial.println("Entered state 4");
        #endif
        break;
      case 5:
        // This is our GO GO GO state
        if (currentState !=5) {
          sleepyTimer.stop();
          antlers(true,255);
          digitalWrite(LED_BUILTIN, HIGH);
          currentState = 5;
          sendControllerPayload();
        }
        #ifdef DEBUG_MODE
          Serial.println("Entered state 5");
        #endif
        break;
      case 6:
        // This is our GO GO GO state but with the MCU and radio turned off after enabling the antlers
        if (currentState !=6 ) {
          sleepyTimer.stop();
          antlers(true,255);
          digitalWrite(LED_BUILTIN, HIGH);
          currentState = 6;
          sendControllerPayload();
        }
        #ifdef DEBUG_MODE
          Serial.println("Entered state 6");
        #endif
        powerDown(50);
        break;
      case 7:
        // Another TBD state. Perhaps for custom timings and antler states?
        break;
      case 8:
        // This is a query state (aka command) that does nothing but radio out the current state of affairs without making any changes.
        sendControllerPayload();
        break;  
      case 9:
        // This is our programming state. Major power hog. Programming takes time, not sure yet if want to auto sleep
        antlers(false,0);
        digitalWrite(LED_BUILTIN, LOW);
        flash.wakeup();
        currentState = 9;
        sendControllerPayload();
        #ifdef DEBUG_MODE
          Serial.println("Entered state 9");
        #endif
        break;
    } // Close switch
}


//*************************************
// Setup                              *
//*************************************

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ANTLER_PIN, OUTPUT);

  EEPROM.setMaxAllowedWrites(10000);
  EEPROM.readBlock(0, CONFIG); // Read in the configuration struct. Decided to use only the nodeID for now.
  
  // #ifdef USECUSTOM
  //   nodeID = NODEID; // nodeID and codeVersion are used in construction of every radio packet.
  //   codeVersion = VERSION;
  // #else
  nodeID = CONFIG.nodeID; // nodeID and codeVersion are used in construction of every radio packet.
  //codeVersion = CONFIG.codeVersion;
  codeVersion = VERSION;
  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(1000);

  radio.initialize(FREQUENCY,nodeID,NETWORKID);
  radio.setFrequency(FREQUENCY_EXACT);
  radio.encrypt(ENCRYPTKEY);
  //   //if (IS_RFM69HW_HCW) {
  //   //  radio.setHighPower();
  //   //}
  // #else
  // radio.initialize(CONFIG.frequency,CONFIG.nodeID,CONFIG.networkID);
  // radio.setFrequency(CONFIG.frequency_exact);
  // radio.encrypt(CONFIG.encryptionKey);

  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //Only for RFM69HW/HCW. Damage may occur if enabled on non-HW/HCW nodes. Antler boards are RFM69HCW.
  #endif

  // if (CONFIG.isHW) {
  //   radio.setHighPower(); //Only for RFM69HW/HCW. Damage may occur if enabled on non-HW/HCW nodes. Antler boards are RFM69HCW.
  // }

  // Having trouble with this for some elementary reason.
  // if (IS_RFM69HW_HCW) {
  //   radio.setHighPower(); //Only for RFM69HW/HCW. Damage may occur if enabled on non-HW/HCW nodes. Antler boards are RFM69HCW.
  // }

  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif
  
  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");

  flash.sleep();

  Serial.print("Starting node: "); Serial.println(nodeID);
  Serial.print("On network ID: "); Serial.println(NETWORKID);
  Serial.print("Using encryption key: "); Serial.println(ENCRYPTKEY);
  Serial.print("On frequency: "); Serial.println(FREQUENCY_EXACT);
  Serial.print("Using high power mode: "); Serial.println(IS_RFM69HW_HCW);


  #ifdef BR_300KBPS
    radio.writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
    radio.writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
    radio.writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
    radio.writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
    radio.writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
    radio.writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
    radio.writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
  #endif

  antlers(true, 255);
  delay (500);
  antlers(false, 0);

  currentState = 1;
  powerDown(8, 50);
} // close setup()



//*************************************
// Loop                               *
//*************************************

void loop(){

  if(sleepyTimer.justFinished()) {
    powerDown(50);
  }
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
    // KEEP THIS AT TOP OF receiveDone() LOOP.
    CheckForWirelessHEX(radio, flash, false);
    //Serial.println("Checked for OTA update");

    // Check if sender wanted an ACK. Doesn't mean we processed the packet correctly internally, just that we got it.
    if (radio.ACKRequested())
    {
      radio.sendACK();
      #ifdef DEBUG_MODE
        Serial.print(" - ACK sent");
      #endif
    }
 
    // Check if valid packet. In future perhaps add checking for different payload versions
    //
    // TODO HIGH PRIORITY - fix this, it's not working.
    //
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
      Serial.print("Received data from node: "); Serial.println(antlersPayload.nodeId);
      Serial.print("Received data is payload version: "); Serial.println(antlersPayload.version);
      Serial.print("Received state is: "); Serial.println(antlersPayload.nodeState);
      Serial.print("Received antler state is: "); Serial.println(antlersPayload.antlerState);
      Serial.print("Received sleep time is: "); Serial.println(antlersPayload.sleepTime);
    #endif
    
    //int nodeSleeper = 1;
    //nodeSleeper = rand() % 10 + 1;
    
    gotoState(antlersPayload.nodeState);


    // switch (antlersPayload.nodeState) {
    //   case 1:
    //     // This is our deep sleep state.
    //     // We want to tell the MCU and radio to go to sleep for 8 seconds, wake up for 50ms to check radio,
    //     // if nothing exciting go back to sleep.
    //     if (currentState != 1 ) {
    //       antlers(false,0);
    //       digitalWrite(LED_BUILTIN, LOW);
    //       currentState = 1;
    //       sendControllerPayload();
    //     }
    //     #ifdef DEBUG_MODE
    //       Serial.println("In state 1");
    //     #endif
    //     powerDown(8, 50);
    //     break;
    //   case 2:
    //     // This is our normal sleep state. The only difference from deep sleep is that we now send out an update
    //     // every 8 seconds.
    //     if (currentState != 2 ) {
    //       antlers(false,0);
    //       digitalWrite(LED_BUILTIN, LOW);
    //       currentState = 2;
          
    //     }
    //     #ifdef DEBUG_MODE
    //       Serial.println("In state 2");
    //     #endif
    //     sendControllerPayload();
    //     powerDown(8, 50);
    //     break;
    //   case 3:
    //     // This is our wake up and pay some attention state. We will call this 12 seconds before we trigger the antlers ON.
    //     // TODO PRIORITY 2 - go back to sleep (state 1) after 2 minutes without receiving a state change.
    //     if (currentState != 3) {
    //       antlers(false, 0);
    //       digitalWrite(LED_BUILTIN, LOW);
    //       currentState = 3;
    //       sendControllerPayload();
    //     }
    //     powerDown(1, 50);
    //     #ifdef DEBUG_MODE
    //       Serial.println("In state 3");
    //     #endif
    //     break;
    //   case 4:
    //     // This is our standby to do stuff really soon state. We'll call this 2 seconds before we trigger the antlers ON.
    //     // We want to stay on full time, no sleep cycle.
    //     // TODO PRIORITY 2: go back to sleep after 2 minutes without a received state change.
    //     if (currentState != 4) {
    //       sleepyTimer.stop();
    //       antlers(false,0);
    //       digitalWrite(LED_BUILTIN, LOW);
    //       currentState = 4;
    //       sendControllerPayload();
    //     }
    //     #ifdef DEBUG_MODE
    //       Serial.println("Entered state 4");
    //     #endif
    //     break;
    //   case 5:
    //     // This is our GO GO GO state
    //     if (currentState !=5) {
    //       sleepyTimer.stop();
    //       antlers(true,255);
    //       digitalWrite(LED_BUILTIN, HIGH);
    //       currentState = 5;
    //       sendControllerPayload();
    //     }
    //     #ifdef DEBUG_MODE
    //       Serial.println("Entered state 5");
    //     #endif
    //     break;
    //   case 6:
    //     // This is our GO GO GO state but with the MCU and radio turned off after enabling the antlers
    //     if (currentState !=6 ) {
    //       sleepyTimer.stop();
    //       antlers(true,255);
    //       digitalWrite(LED_BUILTIN, HIGH);
    //       currentState = 6;
    //       sendControllerPayload();
    //     }
    //     #ifdef DEBUG_MODE
    //       Serial.println("Entered state 6");
    //     #endif
    //     powerDown(50);
    //     break;
    //   case 7:
    //     // Another TBD state. Perhaps for custom timings and antler states?
    //     break;
    //   case 8:
    //     // This is a query state (aka command) that does nothing but radio out the current state of affairs without making any changes.
    //     sendControllerPayload();
    //     break;  
    //   case 9:
    //     // This is our programming state. Major power hog. Programming takes time, not sure yet if want to auto sleep
    //     antlers(false,0);
    //     digitalWrite(LED_BUILTIN, LOW);
    //     flash.wakeup();
    //     currentState = 9;
    //     sendControllerPayload();
    //     #ifdef DEBUG_MODE
    //       Serial.println("Entered state 9");
    //     #endif
    //     break;
    // } // Close switch



  } // close radio receive done
} // close loop()



