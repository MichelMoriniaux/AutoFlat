/* 
 *  
 *  FlipFlat implementation
 *  based on Sweep example by BARRAGAN <http://barraganstudio.com>
 *  and Scott Fitzgerald http://www.arduino.cc/en/Tutorial/Sweep
 *  
 *  implements the Alnitak flipflat protocol to control a flat field EL screen cap 
 *  with a servo
 *  
 *  some eeprom code is borrowed from myfocuser2 project
 *  
 *  Michel Moriniaux 2019
*/

#include <Arduino.h>
#include <ArduinoQueue.h>
#include "AutoFlat.h"

//
// Start Config
//
// Here you input whether there is a Servo or not attached
#define SERVO               OFF           // 1 servo present, 0 no servo
// Should we store the cap position everytime it moves?
#define STOREPOS            OFF           // store the cap position in non-volatile memory
// define default servo positions
#define DEFAULTOPEN         58            // default position for servo open
#define DEFAULTCLOSED       160           // default position for servo closed
// End Config

// Figure out the type of board we are compiling for
#define NANO                0             // Arduino Nano
#define XIAO                1             // Seeeduino Xiao
#define S32C                2             // Seeduino ESP32-C3
#define PICOPD              3             // PicoPD board (RP2040)
#if defined(ARDUINO_AVR_NANO)
  #define BOARD               NANO          // We are running on an Arduino Nano board
#elif defined(__SAMD21__)
  #define BOARD               XIAO          // Good chance we are running on a Seeeduino XIAO board
#elif defined(ESP32)
  #define BOARD               S32C          // This is an ESP32C3 board
#endif
#if BOARD == XIAO
  #include <FlashStorage.h>
#elif BOARD == NANO
  #include <EEPROM.h>
#elif BOARD == S32C
  #include <Preferences.h>
#endif
#if SERVO == ON
  #include <Servo.h>
#endif

#if BOARD == XIAO
  #define LIGHTPIN            8            // Pin that controls the light
  #define SERVOPIN            5             // Pin to which the servo is connected to
  #define PWMFREQ             30000         //pwm frequency
#elif BOARD == S32C
  #define LIGHTPIN            8            // Pin that controls the light
  #define SERVOPIN            5             // Pin to which the servo is connected to
  #define LEDCHANNEL          0
  #define PWMFREQ             30000         //pwm frequency
  #define EEPROMSIZE          10240
#elif BOARD == NANO
  #define LIGHTPIN            3             // Pin that controls the light
  #define SERVOPIN            5             // Pin to which the servo is connected to
  #define EEPROMSIZE          1024
#endif
#if SERVO == ON
  #define FFVERSION           99            // FlipFlat ( we have a servo so the cover moves )
#else
  #define FFVERSION           19            // FlatMan ( only light )
#endif

char programVersion[] = "005";
char ProgramAuthor[]  = "MMX";

struct config_t {
  int validdata;                          // this is only needed for the nano to provide eeprom wear-levelling
#if STOREPOS == ON
  int capposition;                        // last cap position
#endif
  int closedcapposition;                  // servo position when the cap is closed
  int opencapposition;                    // servo position when the cap is open
  int lightbrightness;                    // preset lightbrightness
} autoflat;

int datasize;                             // will hold size of the struct autoflat
int nlocations;                           // number of storage locations available in EEPROM

String commands = "XSVYZJPMNOCQDLB";

int capposition = DEFAULTCLOSED;          // current cap position
int captargetposition = capposition;      // target cap position
int capstatus = CAPCLOSED;                // cap is open, moving or closed
int pos = DEFAULTCLOSED;                  // variable to store the servo position
int lightstatus = OFF;                    // light turned on or off
int motorstatus = OFF;                    // is the motor running or stopped
int currentaddr = 0;                      // will be address in eeprom of the data stored
int defaults = 0;                         // did we write defaults to the EEPROM
byte writenow = 0;                        // should we update values in eeprom
unsigned long lasteepromwrite = millis();

ArduinoQueue<String> intQueue(QUEUELENGTH); // receive serial queue of commands
String line;                              // buffer for serial data
char temp[10];

String osorzeros = "000";                 // nobody can agree wether the Alnitak protocol uses OOO or 000 (SGP uses 000, NINA OOO ) so lets adapt

#if SERVO == ON
Servo myservo;                            // create servo object to control a servo
#endif


// Clear the serial port from any garbage
void clearSerialPort() {
  while ( Serial.available() )
    Serial.read();
}


#if BOARD == XIAO
// This is needed on the Xiao to process serial events
void serialEventRun() {
  if (SerialUSB.available())
  {
    serialEvent();
  }
}
#endif


// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent() {
  // '>' starts the command, '\r' or '\n' ends the command, do not store these in the command buffer
  // read the command until the terminating character
  while (Serial.available() )
  {
    char inChar = Serial.read();
    switch ( inChar )
    {
      case '>':     // start
        line = "";
        break;
      case '\r':     // eoc
        intQueue.enqueue(String(line));
        break;
      case '\n':     // eoc
        intQueue.enqueue(String(line));
        break;
      default:      // anything else
        line = line + inChar;
        break;
    }
  }
}


// Move the servo to position pos. use this carefully as large movements can be instantaneous
// better to increment pos slowly 
void moveTo( int pos ) {
  DebugPrint(F("- Move to:"));
  DebugPrintln(pos);
  #if SERVO == ON
  myservo.write(pos);
  #endif
}


// Turn the light on
void lightOn() {
  DebugPrintln(F("- lightOn(): Turning light ON"));
#if BOARD == XIAO
  pwm(LIGHTPIN, PWMFREQ, map(autoflat.lightbrightness, 0, 255, 0, 1023));
#elif BOARD == S32C
  ledcWrite(LEDCHANNEL, autoflat.lightbrightness);
#elif BOARD == NANO
  if (autoflat.lightbrightness >= 255) {
    digitalWrite(LIGHTPIN, HIGH);
  } else {
    analogWrite(LIGHTPIN, autoflat.lightbrightness);
  }
#endif
  lightstatus = ON;
}


// Turn the light off
void lightOff() {
  DebugPrintln(F("- lightOff(): Turning light OFF"));
#if BOARD == XIAO
  pwm(LIGHTPIN, PWMFREQ, 0);
#elif BOARD == S32C
  ledcWrite(LEDCHANNEL, 0);
#elif BOARD == NANO
  digitalWrite(LIGHTPIN, LOW);
#endif
  lightstatus = OFF;
}


//
// EEPROM functions
//
#if BOARD == XIAO
FlashStorage(eeprom_storage, config_t);


void writeEEPROMNow() {
  DebugPrintln(F("- writeEEPROMNow(): writeEEPROMNow"));
  eeprom_storage.write(autoflat);         // update values in EEPROM
#ifdef DEBUG
  autoflat = eeprom_storage.read();
  DebugPrintln(F("- writeEEPROMNow(): autoflat values: "));
  printConfig();
#endif
  writenow = 0;
  lasteepromwrite = millis();
}


#elif BOARD == NANO
void writeEEPROMNow() {
  autoflat.validdata = 0;
  DebugPrint(F("- writeEEPROMNow(): updating valid flag at EEPROM addr: "));
  DebugPrintln(currentaddr);
  EEPROM.put(currentaddr, autoflat);      // update validdata values in EEPROM
  currentaddr += datasize;                // goto next free address and write data
  // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
  if ( currentaddr >= (nlocations * datasize) )
  {
    currentaddr = 0;
  }
  autoflat.validdata = VALIDDATAFLAG;
  DebugPrint(F("- writeEEPROMNow(): Writing new data at EEPROM addr: "));
  DebugPrintln(currentaddr);
  EEPROM.put(currentaddr, autoflat);      // write the new values in EEPROM
#ifdef DEBUG
  EEPROM.get(currentaddr, autoflat);
  DebugPrintln(F("- writeEEPROMNow(): autoflat values: "));
  printConfig();
#endif
  writenow = 0;
  lasteepromwrite = millis();
}

#elif BOARD == S32C
Preferences prefs;

void writeEEPROMNow() {
  DebugPrintln(F("- writeEEPROMNow(): writeEEPROMNow"));
#if STOREPOS == ON
  if (autoflat.capposition != prefs.getInt("CapPos"))
    prefs.putInt("CapPos", autoflat.capposition);
#endif
  if (autoflat.validdata != prefs.getInt("ValidData"))
    prefs.putInt("ValidData", autoflat.validdata);
  if (autoflat.closedcapposition != prefs.getInt("ClosedCap"))
    prefs.putInt("ClosedCap", autoflat.closedcapposition);
  if (autoflat.opencapposition != prefs.getInt("OpenCap"))
    prefs.putInt("OpenCap", autoflat.opencapposition);
  if (autoflat.lightbrightness != prefs.getInt("Bright"))
    prefs.putInt("Bright", autoflat.lightbrightness);
#ifdef DEBUG
  autoflat.validdata = prefs.getInt("ValidData");
  autoflat.opencapposition = prefs.getInt("OpenCap");
  autoflat.closedcapposition = prefs.getInt("ClosedCap");
  autoflat.lightbrightness = prefs.getInt("Bright");
  #if STOREPOS == ON
  autoflat.capposition = prefs.getInt("CapPos");
  #endif
  DebugPrintln(F("- writeEEPROMNow(): autoflat values: "));
  printConfig();
#endif
  writenow = 0;
  lasteepromwrite = millis();
}

#endif


// Check how long ago we wrote to the EEPROM as we do not want to write too often
void UpdateEEPROMCheck(void) {
  DebugPrintln(F("- UpdateEEPROMCheck(): Checking EEPROM "));
  unsigned long timenow = millis();
  if (((timenow - lasteepromwrite) > EEPROMWRITEINTERVAL) || (timenow < lasteepromwrite))
  {
    autoflat.validdata = VALIDDATAFLAG;
#if STOREPOS == ON
    autoflat.capposition = capposition;
#endif
    writeEEPROMNow();                         // update values in EEPROM
    DebugPrintln("- UpdateEEPROMCheck(): Config saved");
  } else {
  DebugPrintln(F("- UpdateEEPROMCheck(): Not saving now"));
  }
}


// Set default saves values
void setdefaults() {
  autoflat.validdata = VALIDDATAFLAG;
#if STOREPOS == ON
  autoflat.capposition = DEFAULTCLOSED;
#endif
  autoflat.closedcapposition = DEFAULTCLOSED;
  autoflat.opencapposition = DEFAULTOPEN;
  autoflat.lightbrightness = LIGHTMAX;
  writeEEPROMNow();                                   // update values in EEPROM
  defaults = 1;
}

// end EEPROM

//
// Serial COMMS
//
// Helper function
void SendPacket(char *str) {
  DebugPrint(F("- Send: "));
  DebugPrintln(str);
  Serial.print(str);
}


// Process serial Commands
void ser_comms() {
  if ( intQueue.isEmpty() )
    return;

  int cmdval;
  String receiveString = "";
  String WorkString = "";
  int paramval = 0;
  String replystr = "";

  receiveString = (String) intQueue.dequeue();
  String cmdstr = receiveString.substring(0, 1);
  cmdval = commands.indexOf(cmdstr);
  WorkString = receiveString.substring(1, 4);
  if (WorkString == "000" ) {
    osorzeros = "000";
  } else {
    osorzeros = "OOO";
  }
  DebugPrint(F("- ser_comms(): receive string="));
  DebugPrintln(receiveString);
  DebugPrint(F("- ser_comms(): cmdstr="));
  DebugPrintln(cmdstr);
  DebugPrint(F("- ser_comms(): cmdval="));
  DebugPrintln(cmdval);
  DebugPrint(F("- ser_comms(): WorkString="));
  DebugPrintln(WorkString);

  switch (cmdval)
  {
    // all the get go first followed by set
    case 0: // get position
      sprintf(temp,"*X%02d%03d\n", FFVERSION, capposition);
      SendPacket(temp);
      break;
    case 1: // get status
      //delay(100);
      sprintf(temp,"*S%02d%d%d%d\n", FFVERSION, motorstatus, lightstatus, capstatus);
      SendPacket(temp);
      #ifdef DEBUG
      printDebugStatus();
      #endif
      break;
    case 2: // get firmware version
      //delay(100);
      sprintf(temp,"*V%02d%s\n", FFVERSION, programVersion);
      SendPacket(temp);
      break;
    case 3: // get open position
      sprintf(temp,"*Y%02d%03d\n", FFVERSION, autoflat.opencapposition);
      SendPacket(temp);
      break;
    case 4: // get closed position
      sprintf(temp,"*Z%02d%03d\n", FFVERSION, autoflat.closedcapposition);
      SendPacket(temp);
      break;
    case 5: // get light brightness
      DebugPrint(F("- autoflat.lightbrightness="));
      DebugPrintln(autoflat.lightbrightness);
      sprintf(temp,"*J%02d%03d\n", FFVERSION, autoflat.lightbrightness);
      SendPacket(temp);
      break;
    case 6: // get the device id
      sprintf(temp,"*P%02d%s\n", FFVERSION, osorzeros.c_str());
      SendPacket(temp);
      break;
    // only the set commands are listed here as they do not require a response
    case 7: // set open position
      autoflat.opencapposition = (int)WorkString.toInt();
      writenow = 1;
      sprintf(temp,"*M%02d%03d\n", FFVERSION, autoflat.opencapposition);
      SendPacket(temp);
      break;
    case 8: // set closed position
      autoflat.closedcapposition = (int)WorkString.toInt();
      writenow = 1;
      sprintf(temp,"*N%02d%03d\n", FFVERSION, autoflat.closedcapposition);
      SendPacket(temp);
      break;
    case 9: // open the cap
      if ( capstatus != CAPOPEN )
      {
        lightOff();                                 // turn off the light if we are going to open the cap
        //openCap();
        captargetposition = autoflat.opencapposition;
      }
      sprintf(temp,"*O%02d%s\n", FFVERSION, osorzeros.c_str());
      SendPacket(temp);
      break;
    case 10: // close the cap
      if ( capstatus != CAPCLOSED )
      {
        //closeCap();
        captargetposition = autoflat.closedcapposition;
      }
      sprintf(temp,"*C%02d%s\n", FFVERSION, osorzeros.c_str());
      SendPacket(temp);
      break;
    case 11: // move to position
      captargetposition = (int)WorkString.toInt();
      sprintf(temp,"*Q%02d%03d\n", FFVERSION, captargetposition);
      SendPacket(temp);
      break;
    case 12: // turn off the EL panel
      if ( lightstatus == ON )
      {
        lightOff();
      }
      sprintf(temp,"*D%02d%s\n", FFVERSION, osorzeros.c_str());
      SendPacket(temp);
      break;
    case 13: // turn on the EL panel
      if ( lightstatus == OFF )
      {
        lightOn();
      }
      sprintf(temp,"*L%02d%s\n", FFVERSION, osorzeros.c_str());
      SendPacket(temp);
      break;
    case 14: // set brighness
      autoflat.lightbrightness = (int)WorkString.toInt();
      if ( lightstatus == ON )
      {
        lightOn();
      }
      sprintf(temp,"*B%02d%03d\n", FFVERSION, autoflat.lightbrightness);
      SendPacket(temp);
      break;
  }
}

#ifdef DEBUG
void printDebugStatus() {
  char buffer[100];
  sprintf(buffer, "curaddr %d, datasize: %d, nloc: %d, L: %d, M: %d, def: %d", currentaddr, datasize, nlocations, lightstatus, motorstatus, defaults);
  DebugPrintln(buffer);
}


void printConfig() {
  char buffer[100];
  #if STOREPOS == ON
  sprintf(buffer, "vflag: %d, open: %d, close: %d, bright: %d, pos: %d", autoflat.validdata, autoflat.opencapposition, autoflat.closedcapposition, autoflat.lightbrightness, autoflat.capposition);
  #else
  sprintf(buffer, "vflag: %d, open: %d, close: %d, bright: %d", autoflat.validdata, autoflat.opencapposition, autoflat.closedcapposition, autoflat.lightbrightness);
  #endif
  DebugPrintln(buffer);
}
#endif


void setup() {
  byte found = 0;

  Serial.begin(SERIALPORTSPEED);            // initialize serial port
  while (!Serial);
  #if BOARD == S32C
  delay(1000);
  #endif
  DebugPrintln(F("- setup(): Starting..."));
  clearSerialPort();                        // clear any garbage from serial buffer

  #if BOARD == NANO
  // Set the timer for LIGHTPIN to 31KHz
  //----- PWM frequency for D3 & D11 -----
  //Timer2 divisor = 2, 16, 64, 128, 512, 2048
  TCCR2B = TCCR2B & B11111000 | B00000001;    // 31KHz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // 3.9KHz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // 980Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // 490Hz (default)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // 245Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // 122.5Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // 30.6Hz
  #endif

  #if BOARD == XIAO
  // Turn off all leds on the Xiao board
  pinMode(PIN_LED_TXL, INPUT);
  pinMode(PIN_LED_RXL, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  #endif
  #if BOARD == XIAO || BOARD == NANO
  pinMode(LIGHTPIN, OUTPUT);
  #endif
  #if BOARD == S32C
  ledcSetup(LEDCHANNEL, PWMFREQ, 8);
  ledcAttachPin(LIGHTPIN, LEDCHANNEL);
  #endif
  
  lightOff();                               // init light pin and set to off
 
  #if SERVO == ON
  myservo.attach(SERVOPIN);                 // attaches the servo on pin 9 to the servo object
  capposition = myservo.read();             // get the current position of the servo
  #endif
  captargetposition = capposition;          // default for capposition is DEFAULTCLOSED

  // initialize the EEPROM stuff
  datasize = sizeof( autoflat );
  #if BOARD == S32C
  prefs.begin("Flat", false);
  #endif
  #if BOARD == NANO
  nlocations = EEPROM.length() / datasize;
  DebugPrint(F("- setup(): EEPROM cells: "));
  DebugPrintln(nlocations);
  #endif

  found = 0;
  #if BOARD == XIAO
  autoflat = eeprom_storage.read();

  if ( autoflat.validdata == VALIDDATAFLAG )   // check to see if the data is valid
  {
    DebugPrintln(F("- setup(): Found valid config"));
    found = 1;
  }
  #elif BOARD == NANO
  // start at 0 if not found later
  for (int lp1 = 0; lp1 < nlocations; lp1++ )
  {
    int addr = lp1 * datasize;
    EEPROM.get( addr, autoflat );
    if ( autoflat.validdata == VALIDDATAFLAG ) // check to see if the data is valid
    {
      currentaddr = addr;                      // data was erased so write some default values
      found = 1;
      DebugPrint(F("- setup(): Found valid config at address: "));
      DebugPrintln(currentaddr);
      break;
    }
  }
  #elif BOARD == S32C
  if (prefs.getInt("ValidData") == VALIDDATAFLAG) {
    DebugPrintln(F("- setup(): Found valid config"));
    found = 1;
    autoflat.opencapposition = prefs.getInt("OpenCap");
    autoflat.closedcapposition = prefs.getInt("ClosedCap");
    autoflat.lightbrightness = prefs.getInt("Bright");
    #if STOREPOS == ON
    autoflat.capposition = prefs.getInt("CapPos");
    #endif

  }
  #endif
  if ( found == 0 )
  {
    // set defaults because not found
    DebugPrintln(F("- setup(): no valid config found, writing defaults to EEPROM "));
    setdefaults();                   // set defaults because not found
  }
  // configure with what we now know
#if STOREPOS == ON
  captargetposition = autoflat.capposition;
#endif
  if ( capposition == autoflat.opencapposition )
    capstatus = CAPOPEN;
  else if ( capposition == autoflat.closedcapposition )
    capstatus = CAPCLOSED;
  else
    capstatus = CAPMOVING;
}


void loop() {
  static byte MainStateMachine = State_Idle;

#if BOARD == S32C
  serialEvent();
#endif

  ser_comms();
  
  switch (MainStateMachine)
  {
    case State_Idle:
      if (capposition != captargetposition)
      {
        MainStateMachine = State_InitMove;
        DebugPrint(F("- loop(): Idle => InitMove Target "));
        DebugPrintln(captargetposition);
        DebugPrint(F("- Current "));
        DebugPrintln(capposition);
      }
      else
      {
        if ( writenow == 1 )
        {
          UpdateEEPROMCheck();
        }
      }
      break;

    case State_InitMove:
      capstatus = CAPMOVING;
      motorstatus = ON;
      MainStateMachine = State_Moving;
      DebugPrintln(F("- loop(): => State_Moving#"));
      break;

    case State_Moving:
      if ( capposition == captargetposition )      // must come first else cannot halt
      {
        MainStateMachine = State_FinishedMove;
        DebugPrint(F("- loop(): => State_FinishedMove#"));
      }
      else
      {
        if ( captargetposition - capposition > 0 )
          capposition++;
        else
          capposition--;
        moveTo(capposition);
      }
      break;

    case State_FinishedMove:
      if ( capposition == autoflat.closedcapposition )
        capstatus = CAPCLOSED;
      if ( capposition == autoflat.opencapposition )
        capstatus = CAPOPEN;
      motorstatus = OFF;
      MainStateMachine = State_Idle;
      DebugPrintln(F("- loop(): => State_Idle#"));
      break;

    default:
      MainStateMachine = State_Idle;
      break;
  }

#ifdef LOOPTIMETEST
  DebugPrint(F("- loop(): Loop End ="));
  DebugPrintln(millis());
#endif

}
