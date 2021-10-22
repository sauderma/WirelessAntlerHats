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

//#define NODEID       203  // node ID used for this unit
//#define NETWORKID    150  
//#define GATEWAY1     1
//#define GATEWAY2     2
//#define FREQUENCY    RF69_915MHZ
//#define FREQUENCY_EXACT 905500000 // Move away from the default exact 915Mhz, presumably less interference
//#define ENCRYPTKEY  "rcmhprodrcmhprod" //16-bytes or ""/0/null for no encryption
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//#define VERSION     1 // What version software are we running?

// *****************************************************************************************

#define BROADCASTID  0 // Most or all Moteino documentation says to use 255 for broadcast. 255 IS WRONG!


#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define FLASH_ID      0xEF30  //ID for the 4Mbit Winbond W25X40CL flash chip

#define SERIAL_BAUD 115200
#define BLINKPERIOD 250
#define DEBUG_MODE true //uncomment to enable debug comments


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

//char input = 0; // think this isn't used anymore
//long lastPeriod = -1; // think this isn't used anymore

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



//*************************************
// Setup                              *
//*************************************

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ANTLER_PIN, OUTPUT);

  EEPROM.setMaxAllowedWrites(10000);
  EEPROM.readBlock(0, CONFIG); // Read in the configuration struct
  
  nodeID = CONFIG.nodeID; // nodeID and codeVersion are used in construction of every radio packet.
  codeVersion = CONFIG.codeVersion; // This maybe saves reading from EEPROM each time? Maybe not.
  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(1000);
  radio.initialize(CONFIG.frequency,CONFIG.nodeID,CONFIG.networkID);
//  radio.initialize(FREQUENCY,NODEID,NETWORKID); // comment out above line and uncomment this line to use custom values

  radio.setFrequency(CONFIG.frequency_exact); // will need to change this line for custom value
  radio.encrypt(CONFIG.encryptionKey); // will need to change this line for custom value

  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

  if (CONFIG.isHW) {
    radio.setHighPower(); //Only for RFM69HW/HCW. Damage may occur if enabled on non-HW/HCW nodes. Antler boards are RFM69HCW.
  }

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");
  flash.sleep();

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
  // TODO LOW PRIORITY
  // fill this in if we want to blink the on-board LED in various patterns for easier troubleshooting
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

//************************************
//  PowerDown function
//************************************
void powerDown(int wakeTime){
  radio.sleep();
  if(currentState == 1){
    Serial.println("Entering 8s sleep");
    Serial.flush();
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF) ;
  } else {
    Serial.println("Entering 1s sleep");
    Serial.flush();
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  } 
  sleepyTimer.start(wakeTime);
  Serial.println("Leaving Sleep...");
}



//*************************************
// Loop                               *
//*************************************

void loop(){

  if(sleepyTimer.justFinished()){
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
      Serial.println("Received data from node: ");
      Serial.println(antlersPayload.nodeId);
      Serial.println("Received data is payload version: ");
      Serial.println(antlersPayload.version);
      Serial.println("Received state is: ");
      Serial.println(antlersPayload.nodeState);
      Serial.println("Received antler state is: ");
      Serial.println(antlersPayload.antlerState);
      Serial.println("Received sleep time is: ");
      Serial.println(antlersPayload.sleepTime);
    #endif

    // And now for the real meat, what should we be doing right now?
    // Overall we want to determine as quickly as possible if we need to do something, or if we can go back to sleep. 
    
    //int nodeSleeper = 1;
    //nodeSleeper = rand() % 10 + 1;
    

    //if (antlersPayload.nodeState != currentState) {
      switch (antlersPayload.nodeState) {
        case 1:
          // TODO HIGH PRIORITY
          // This is our deep sleep state.
          // We want to tell the MCU and radio to go to sleep for 8 seconds, wake up for 50ms (may need to play with this timing),
          // if nothing exciting go back to sleep. Note that the way this is currently structured I think this loop is WRONG, it's only reached
          // when a packet is received with a state change. What happens when you receive sleep packets being sent 30x/second for 10 minutes? Not sure,
          // think through the logic here...
          // Note that when waking up, the sketch will continue from the point in code where it went to sleep, it does NOT restart the loop.
          antlers(false,0);
          digitalWrite(LED_BUILTIN, LOW);
          analogWrite(ANTLER_PIN, 1);
          currentState = 1;
          sendControllerPayload();
          #ifdef DEBUG_MODE
            Serial.println("Entered state 1");
          #endif
          powerDown(50);
          // sleep radio
          // sleep MCU for 8 seconds
          // wake up radio, here or maybe a wakeup routine? Be sure not to end up in a loop that never exits, so that loop() runs properly.
          // ???
         //radio.sleep(); //replace with our sleep function
         #ifdef DEBUG_MODE
           Serial.println("I woke up");
         #endif
         
        //set the 50ms wait;
          break;
        case 2:
          // TODO HIGH PRIORITY
          // This is our wake up and pay some attention state. We will call this 10 seconds before we trigger the antlers ON.
          // We want to shorten our sleep time to perhaps 1 second off, 50ms on? Experiment with on time.
          // Probably the state to go in with the handheld remotes.
          // TODO PRIORITY 2 - Probably want to go back to sleep (state 1) after say 2 minutes without receiving a state change.
          antlers(false, 0);
          digitalWrite(LED_BUILTIN, LOW);
          currentState = 2;
          sendControllerPayload();
          powerDown(1000);
          #ifdef DEBUG_MODE
            Serial.println("Entered state 2");
          #endif
          // sleep radio
          // sleep MCU for 1 second
          // wake up radio, here or maybe a wakeup routine? Be sure not to end up in a loop that never exits, so that loop() runs properly.
          // ???
          radio.sleep();
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
          radio.setPowerLevel(RF69_MODE_RX);
          break;
        case 3:
          // TODO HIGH PRIORITY
          // This is our standby to do stuff really soon state. We'll call this 2 seconds before we trigger the antlers ON.
          // We want to stay on full time, no sleep cycle, but go back to sleep after 2 minutes without a received state change.
          sleepyTimer.stop();
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
          sleepyTimer.stop();
          antlers(true,255);
          digitalWrite(LED_BUILTIN, HIGH);
          currentState = 4;
          sendControllerPayload();
          #ifdef DEBUG_MODE
            Serial.println("Entered state 4");
          #endif
          break;
        case 5:
          // TODO MEDIUM PRIORITY
          // This is the state where we experiment with keeping the antlers on
          // while sleeping the MCU and radio. It might actually be easy.
          sleepyTimer.stop();
          antlers(true,255);
          digitalWrite(LED_BUILTIN, HIGH);
          currentState = 5;
          sendControllerPayload();
          #ifdef DEBUG_MODE
            Serial.println("Entered state 5");
          #endif
          powerDown(50);
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
    //} // close if state change
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



