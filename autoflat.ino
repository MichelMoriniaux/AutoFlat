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


#define NANO                0             // Arduino Nano
#define XIAO                1             // Seeduino Xiao
#define ON                  1             // EL panel or motor is on
#define OFF                 0             // EL panel or motor is off

// Board config
#define SERVO               OFF           // 1 servo present, 0 no servo
#define BOARD               NANO          // NANO or XIAO
#define STOREPOS            OFF           // store the cap position in non-volatile memory

// includes
#include <Arduino.h>
#include <myQueue.h>                      //  By Steven de Salas
#if BOARD == XIAO
#include <FlashStorage.h>
#endif
#if BOARD == NANO
#include <myEEPROM.h>                     // needed for EEPROM
#include <myeepromanything.h>             // needed for EEPROM
#endif
#if SERVO == ON
#include <Servo.h>
#endif

#define DEFAULTOPEN         58            // default position for servo open
#define DEFAULTCLOSED       160           // default position for servo closed
#define SERVOPIN            5             // Pin to which the servo is connected to
#if BOARD == XIAO
#define LIGHTPIN            A1            // Pin that controls the light
#endif
#if BOARD == NANO
#define LIGHTPIN            3             // Pin that controls the light
#endif
#define LIGHTMAX            255           // maximum light brightness
#define CAPOPEN             2             // cap is open
#define CAPCLOSED           1             // cap is closed
#define CAPMOVING           0             // servo is moving
#define EEPROMSIZE          1024          // ATMEGA328P 1024 EEPROM
#define EEPROMWRITEINTERVAL 10000L        // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
#define VALIDDATAFLAG       99            // valid eeprom data flag
#define SERIALPORTSPEED     9600          // 9600, 14400, 19200, 28800, 38400, 57600
#define QUEUELENGTH         20            // number of commands that can be saved in the serial queue
#define PWMFREQ             20000         //pwm frequency
#define EOFSTR              '\n'
//  StateMachine definition
#define State_Idle              0
#define State_InitMove          1
#define State_Moving            3
#define State_FinishedMove      5

// ----------------------------------------------------------------------------------------------------------
// DEBUGGING
// ----------------------------------------------------------------------------------------------------------
// do not change - leave this commented out
//#define DEBUG 1
//#define LOOPTIMETEST 1

#if BOARD == XIAO
#define MYSERIAL                  SerialUSB
#endif
#if BOARD == NANO
#define MYSERIAL                  Serial
#endif
#ifdef DEBUG                                          //Macros are usually in all capital letters.
#define DebugPrint(...) MYSERIAL.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugPrintln(...) MYSERIAL.println(__VA_ARGS__) //DPRINTLN is a macro, debug print with new line
#else
#define DebugPrint(...)                               //now defines a blank line
#define DebugPrintln(...)                             //now defines a blank line
#endif

struct config_t {
  int validdata;                          // if this is 99 then data is valid
#if STOREPOS == ON
  int capposition;                        // last cap position
#endif
  int closedcapposition;                  // servo position when the cap is closed
  int opencapposition;                    // servo position when the cap is open
  int lightbrightness;                    // preset lightbrightness
} autoflat;

int datasize = sizeof( autoflat );        // will hold size of the struct autoflat
int nlocations = EEPROMSIZE / datasize;   // number of storage locations available in EEPROM

char programVersion[] = "001";
char ProgramAuthor[]  = "MMX";
String commands = "XSVYZJPMNOCQDLB";

int capposition = DEFAULTCLOSED;          // current cap position
int captargetposition = capposition;      // target cap position
int capstatus = CAPCLOSED;                // cap is open, moving or closed
int lightstatus = OFF;                    // light turned on or off
int motorstatus = OFF;                    // is the motor running or stopped
int currentaddr = 0;                      // will be address in eeprom of the data stored
byte writenow = 0;                        // should we update values in eeprom
unsigned long lasteepromwrite = millis();

Queue<String> queue(QUEUELENGTH);         // receive serial queue of commands
String line;                              // buffer for serial data
char temp[10];

String osorzeros = "000";                 // nobody can agree wether the Alnitak protocol uses OOO or 000 (SGP uses 000, NINA OOO ) so lets adapt

#if SERVO == ON
Servo myservo;                            // create servo object to control a servo
// twelve servo objects can be created on most boards
#endif

int pos = DEFAULTCLOSED;                              // variable to store the servo position


void clearSerialPort()
{
  while ( MYSERIAL.available() )
    MYSERIAL.read();
}


#if BOARD == XIAO
void serialEventRun()
{
  if (SerialUSB.available())
  {
    serialEvent();
  }
}
#endif


// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent()
{
  // '>' starts the command, '\r' or '\n' ends the command, do not store these in the command buffer
  // read the command until the terminating character
  while (MYSERIAL.available() )
  {
    char inChar = MYSERIAL.read();
    switch ( inChar )
    {
      case '>':     // start
        line = "";
        break;
      case '\r':     // eoc
        queue.push(String(line));
        break;
      case '\n':     // eoc
        queue.push(String(line));
        break;
      default:      // anything else
        line = line + inChar;
        break;
    }
  }
}


// unused
void openCap()
{
  if ( lightstatus )
    lightOff();
  moveTo(autoflat.opencapposition);
  capstatus = CAPOPEN;
}


// unused
void closeCap()
{
  int prelightstatus = lightstatus;
  if (lightstatus)
    lightOff();
  moveTo(autoflat.closedcapposition);
  capstatus = CAPOPEN;
  // if (prelightstatus)
  lightOn();
}


void moveTo( int pos )
{
  DebugPrint(F("- to:"));
  DebugPrintln(pos);
  #if SERVO == ON
  myservo.write(pos);
  #endif
}


void lightOn()
{
  if (autoflat.lightbrightness >= 255) {
    pinMode(LIGHTPIN, OUTPUT);
    digitalWrite(LIGHTPIN, HIGH);
  } else {
#if BOARD == NANO
    analogWrite(LIGHTPIN, autoflat.lightbrightness);
#endif
#if BOARD == XIAO
    pwm(LIGHTPIN, PWMFREQ, map(autoflat.lightbrightness, 0, 255, 0, 1023));
#endif
  }
  lightstatus = ON;
}


void lightOff()
{
  pinMode(LIGHTPIN, OUTPUT);
  digitalWrite(LIGHTPIN, LOW);
  lightstatus = OFF;
}


// EEPROM stuff
#if BOARD == XIAO
FlashStorage(eeprom_storage, config_t);


void writeEEPROMNow()
{
  eeprom_storage.write(autoflat);       // update values in EEPROM
  writenow = 0;
  lasteepromwrite = millis();
}
#endif


#if BOARD == NANO
void writeEEPROMNow()
{
  autoflat.validdata = 0;
  DebugPrintln(F("- writing EEPROM "));
  EEPROM_writeAnything(currentaddr, autoflat);       // update validdata values in EEPROM
  currentaddr += datasize;                           // goto next free address and write data
  // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
  if ( currentaddr >= (nlocations * datasize) )
  {
    currentaddr = 0;
  }
  autoflat.validdata = VALIDDATAFLAG;
  EEPROM_writeAnything(currentaddr, autoflat);       // write the new values in EEPROM
  DebugPrintln(F("- autoflat values: "));
  DebugPrintln(autoflat.validdata);
#if STOREPOS == ON
  DebugPrintln(autoflat.capposition);
#endif
  DebugPrintln(autoflat.lightbrightness);
  DebugPrintln(autoflat.opencapposition);
  DebugPrintln(autoflat.closedcapposition);
  writenow = 0;
  lasteepromwrite = millis();
}
#endif


void UpdateEEPROMCheck(void)
{
  unsigned long timenow = millis();
  if (((timenow - lasteepromwrite) > EEPROMWRITEINTERVAL) || (timenow < lasteepromwrite))
  {
    autoflat.validdata = VALIDDATAFLAG;
#if STOREPOS == ON
    autoflat.capposition = capposition;
#endif
    writeEEPROMNow();                         // update values in EEPROM
    DebugPrint("Config saved");
  }
}


void setdefaults()
{
  autoflat.validdata = VALIDDATAFLAG;
#if STOREPOS == ON
  autoflat.capposition = DEFAULTCLOSED;
#endif
  autoflat.closedcapposition = DEFAULTCLOSED;
  autoflat.opencapposition = DEFAULTOPEN;
  autoflat.lightbrightness = LIGHTMAX;
  writeEEPROMNow();                                   // update values in EEPROM
}


// MYSERIAL COMMS
void SendPacket(char *str)
{
  DebugPrint(F("- Send: "));
  DebugPrintln(str);
  MYSERIAL.print(str);
}


// Serial Commands
void ser_comms()
{
  if ( queue.count() == 0 )
    return;

  int cmdval;
  String receiveString = "";
  String WorkString = "";
  int paramval = 0;
  String replystr = "";

  receiveString = (String) queue.pop();
  String cmdstr = receiveString.substring(0, 1);
  cmdval = commands.indexOf(cmdstr);
  WorkString = receiveString.substring(1, 4);
  if (WorkString == "000" ) {
    osorzeros = "000";
  } else {
    osorzeros = "OOO";
  }
  DebugPrint(F("- receive string="));
  DebugPrintln(receiveString);
  DebugPrint(F("- cmdstr="));
  DebugPrintln(cmdstr);
  DebugPrint(F("- cmdval="));
  DebugPrintln(cmdval);
  DebugPrint(F("- WorkString="));
  DebugPrintln(WorkString);

  switch (cmdval)
  {
    // all the get go first followed by set
    case 0: // get position
      sprintf(temp,"*X99%03d\n", capposition);
      SendPacket(temp);
      break;
    case 1: // get status
      //delay(100);
      sprintf(temp,"*S99%d%d%d\n", motorstatus, lightstatus, capstatus);
      SendPacket(temp);
      break;
    case 2: // get firmware version
      //delay(100);
      sprintf(temp,"*V99%s\n", programVersion);
      SendPacket(temp);
      break;
    case 3: // get open position
      sprintf(temp,"*Y99%03d\n", autoflat.opencapposition);
      SendPacket(temp);
      break;
    case 4: // get closed position
      sprintf(temp,"*Z99%03d\n", autoflat.closedcapposition);
      SendPacket(temp);
      break;
    case 5: // get light brightness
      DebugPrint(F("- autoflat.lightbrightness="));
      DebugPrintln(autoflat.lightbrightness);
      sprintf(temp,"*J99%03d\n", autoflat.lightbrightness);
      SendPacket(temp);
      break;
    case 6: // get the device id
      sprintf(temp,"*P99%s\n", osorzeros.c_str());
      SendPacket(temp);
      break;
    // only the set commands are listed here as they do not require a response
    case 7: // set open position
      autoflat.opencapposition = (int)WorkString.toInt();
      writenow = 1;
      sprintf(temp,"*M99%03d\n", autoflat.opencapposition);
      SendPacket(temp);
      break;
    case 8: // set closed position
      autoflat.closedcapposition = (int)WorkString.toInt();
      writenow = 1;
      sprintf(temp,"*N99%03d\n", autoflat.closedcapposition);
      SendPacket(temp);
      break;
    case 9: // open the cap
      if ( capstatus != CAPOPEN )
      {
        lightOff();                                 // turn off the light if we are going to open the cap
        //openCap();
        captargetposition = autoflat.opencapposition;
      }
      sprintf(temp,"*O99%s\n", osorzeros.c_str());
      SendPacket(temp);
      break;
    case 10: // close the cap
      if ( capstatus != CAPCLOSED )
      {
        //closeCap();
        captargetposition = autoflat.closedcapposition;
      }
      sprintf(temp,"*C99%s\n", osorzeros.c_str());
      SendPacket(temp);
      break;
    case 11: // move to position
      captargetposition = (int)WorkString.toInt();
      sprintf(temp,"*Q99%03d\n", captargetposition);
      SendPacket(temp);
      break;
    case 12: // turn off the EL panel
      if ( lightstatus == ON )
      {
        lightOff();
      }
      sprintf(temp,"*D99%s\n", osorzeros.c_str());
      SendPacket(temp);
      break;
    case 13: // turn on the EL panel
      if ( lightstatus == OFF )
      {
        lightOn();
      }
      sprintf(temp,"*L99%s\n", osorzeros.c_str());
      SendPacket(temp);
      break;
    case 14: // set brighness
      autoflat.lightbrightness = (int)WorkString.toInt();
      if ( lightstatus == ON )
      {
        lightOn();
      }
      sprintf(temp,"*B99%03d\n", autoflat.lightbrightness);
      SendPacket(temp);
      break;
  }
}


void setup() {
  byte found = 0;

  //----- PWM frequency for D3 & D11 -----
  //Timer2 divisor = 2, 16, 64, 128, 512, 2048
  #if BOARD == NANO
  TCCR2B = TCCR2B & B11111000 | B00000001;    // 31KHz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // 3.9KHz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // 980Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // 490Hz (default)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // 245Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // 122.5Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // 30.6Hz
  #endif

  MYSERIAL.begin(SERIALPORTSPEED);            // initialize serial port
  clearSerialPort();                        // clear any garbage from serial buffer
  lightOff();                               // init light pin and set to off
 
  #if SERVO == ON
  myservo.attach(SERVOPIN);                 // attaches the servo on pin 9 to the servo object
  capposition = myservo.read();             // get the current position of the servo
  #endif
  captargetposition = capposition;          // default for capposition is DEFAULTCLOSED

  // initialize the EEPROM stuff
  found = 0;
  #if BOARD == XIAO
  autoflat = eeprom_storage.read();

  if ( autoflat.validdata == VALIDDATAFLAG )   // check to see if the data is valid
  {
    found = 1;
  }
  #endif
  #if BOARD == NANO
  // start at 0 if not found later
  for (int lp1 = 0; lp1 < nlocations; lp1++ )
  {
    int addr = lp1 * datasize;
    EEPROM_readAnything( addr, autoflat );
    if ( autoflat.validdata == VALIDDATAFLAG ) // check to see if the data is valid
    {
      currentaddr = addr;                      // data was erased so write some default values
      found = 1;
      break;
    }
  }
  #endif
  if ( found == 0 )
  {
    // set defaults because not found
    DebugPrintln(F("- writing defaults to EEPROM "));
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

  ser_comms();
  
  switch (MainStateMachine)
  {
    case State_Idle:
      if (capposition != captargetposition)
      {
        MainStateMachine = State_InitMove;
        DebugPrint(F("- Idle => InitMove Target "));
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
      DebugPrintln(F("- => State_Moving#"));
      break;

    case State_Moving:
      if ( capposition == captargetposition )      // must come first else cannot halt
      {
        MainStateMachine = State_FinishedMove;
        DebugPrint(F("- => State_FinishedMove#"));
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
      DebugPrintln(F("- => State_Idle#"));
      break;

    default:
      MainStateMachine = State_Idle;
      break;
  }

#ifdef LOOPTIMETEST
  DebugPrint(F("- Loop End ="));
  DebugPrintln(millis());
#endif

}
