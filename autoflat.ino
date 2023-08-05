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
#include <myQueue.h>                      //  By Steven de Salas
#include <FlashStorage.h>
#include <Servo.h>

#define DEFAULTOPEN         58            // default position for servo open
#define DEFAULTCLOSED       160           // default position for servo closed
#define SERVOPIN            5             // Pin to which the servo is connected to
#define LIGHTPIN            A1             // Pin that controls the light
#define ON                  1             // EL panel or motor is on
#define OFF                 0             // EL panel or motor is off
#define LIGHTMAX            255           // maximum light brightness
#define CAPOPEN             2             // cap is open
#define CAPCLOSED           1             // cap is closed
#define CAPMOVING           0             // servo is moving
#define EEPROMSIZE          1024          // ATMEGA328P 1024 EEPROM
#define EEPROMWRITEINTERVAL 10000L        // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
#define VALIDDATAFLAG       99            // valid eeprom data flag
#define SERIALPORTSPEED     9600         // 9600, 14400, 19200, 28800, 38400, 57600
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

#ifdef DEBUG                                          //Macros are usually in all capital letters.
#define DebugPrint(...) SerialUSB.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugPrintln(...) SerialUSB.println(__VA_ARGS__) //DPRINTLN is a macro, debug print with new line
#else
#define DebugPrint(...)                               //now defines a blank line
#define DebugPrintln(...)                             //now defines a blank line
#endif

struct config_t {
  int validdata;                          // if this is 99 then data is valid
  int capposition;                        // last cap position
  int closedcapposition;                  // servo position when the cap is closed
  int opencapposition;                    // servo position when the cap is open
  int lightbrightness;                    // preset lightbrightness
} autoflat;

char programVersion[] = "001";
char ProgramAuthor[]  = "MMX";
String commands = "XSVYZJPMNOCQDLB";

int capposition;                          // current cap position
int captargetposition;                    // target cap position
int capstatus;                            // cap is open, moving or closed
int lightstatus;                          // light turned on or off
int motorstatus;                          // is the motor running or stopped
int currentaddr;                          // will be address in eeprom of the data stored
byte writenow;                            // should we update values in eeprom

Queue<String> queue(QUEUELENGTH);         // receive serial queue of commands
String line;                              // buffer for serial data
char temp[10];
unsigned long lasteepromwrite;

String osorzeros = "000";                 // nobody can agree wether the Alnitak protocol uses OOO or 000 (SGP uses 000, NINA OOO ) so lets adapt

Servo myservo;                            // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;                              // variable to store the servo position

void clearSerialPort()
{
  while ( SerialUSB.available() )
    SerialUSB.read();
}

void serialEventRun()
{
  if (SerialUSB.available())
  {
    serialEvent();
  }
}

// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent()
{
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character
  while (SerialUSB.available() )
  {
    char inChar = SerialUSB.read();
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

void openCap()
{
  if ( lightstatus )
    lightOff();
  moveTo(autoflat.opencapposition);
  capstatus = CAPOPEN;
}

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
  DebugPrint(pos);
  DebugPrint(EOFSTR);
  myservo.write(pos);
}

void lightOn()
{
  if (autoflat.lightbrightness >= 255) {
    pinMode(LIGHTPIN, OUTPUT);
    digitalWrite(LIGHTPIN, HIGH);
  } else {
    //analogWrite(LIGHTPIN, autoflat.lightbrightness);
    pwm(LIGHTPIN, PWMFREQ, map(autoflat.lightbrightness, 0, 255, 0, 1023));
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
FlashStorage(eeprom_storage, config_t);

void writeEEPROMNow()
{
  eeprom_storage.write(autoflat);       // update values in EEPROM
  writenow = 0;
}

void UpdateEEPROMCheck(void)
{
  unsigned long timenow = millis();
  if (((timenow - lasteepromwrite) > EEPROMWRITEINTERVAL) || (timenow < lasteepromwrite))
  {
    lasteepromwrite     = timenow ;           // update the timestamp
    autoflat.validdata = VALIDDATAFLAG;
    autoflat.capposition = capposition;
    writeEEPROMNow();                         // update values in EEPROM
    DebugPrint("Config saved");
  }
}

void setdefaults()
{
  autoflat.validdata = VALIDDATAFLAG;
  autoflat.capposition = DEFAULTCLOSED;
  autoflat.closedcapposition = DEFAULTCLOSED;
  autoflat.opencapposition = DEFAULTOPEN;
  autoflat.lightbrightness = LIGHTMAX;
  writeEEPROMNow();                                   // update values in EEPROM
}

// SERIAL COMMS
void SendPacket(char *str)
{
  DebugPrint(F("- Send: "));
  DebugPrint(str);
  DebugPrint(EOFSTR);
  SerialUSB.print(str);
}

// Serial Commands
//void ser_comms(String receiveString)
void ser_comms()
{
  int cmdval;
  String receiveString = "";
  String WorkString = "";
  int paramval = 0;
  String replystr = "";

  if ( queue.count() == 0 )
    return;
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
  DebugPrint(receiveString);
  DebugPrint(EOFSTR);
  DebugPrint(F("- cmdstr="));
  DebugPrint(cmdstr);
  DebugPrint(EOFSTR);
  DebugPrint(F("- cmdval="));
  DebugPrint(cmdval);
  DebugPrint(EOFSTR);
  DebugPrint(F("- WorkString="));
  DebugPrint(WorkString);
  DebugPrint(EOFSTR);

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
  int datasize;                             // will hold size of the struct autoflat - 4 bytes
  int nlocations;                           // number of storage locations available in EEPROM
  byte found;

  //----- PWM frequency for D3 & D11 -----
  //Timer2 divisor = 2, 16, 64, 128, 512, 2048
  //TCCR2B = TCCR2B & B11111000 | B00000001;    // 31KHz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // 3.9KHz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // 980Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // 490Hz (default)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // 245Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // 122.5Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // 30.6Hz
  SerialUSB.begin(SERIALPORTSPEED);            // initialize serial port
  clearSerialPort();                        // clear any garbage from serial buffer
  pinMode(LIGHTPIN, OUTPUT);
 
  myservo.attach(SERVOPIN);                 // attaches the servo on pin 9 to the servo object
  capposition = myservo.read();    // get the current position of the servo
  captargetposition = capposition;
  
  capstatus = CAPMOVING;
  if ( capposition == autoflat.opencapposition )
    capstatus = CAPOPEN;
  if ( capposition == autoflat.closedcapposition )
    capstatus = CAPCLOSED;

  // initialize the EEPROM stuff
  datasize = sizeof( autoflat );
  found = 0;                        // start at 0 if not found later
  autoflat = eeprom_storage.read();
  if ( autoflat.validdata == VALIDDATAFLAG )   // check to see if the data is valid
  {
    found = 1;
  }
  if ( found == 0 )
  {
    setdefaults();                   // set defaults because not found
  }


}

String str;

void loop() {
  static byte MainStateMachine = State_Idle;

  if ( queue.count() >= 1 )                 // check for serial command
  {
    ser_comms();
  }
  /*if ( SerialUSB.available()) {
    DebugPrint(F("- received data "));
    str = SerialUSB.readStringUntil('\r');
    DebugPrint(F("- have a string "));
    ser_comms(str.substring(1));
  }*/
  
  switch (MainStateMachine)
  {
    case State_Idle:
      if (capposition != captargetposition)
      {
        MainStateMachine = State_InitMove;
        DebugPrint(F("- Idle => InitMove Target "));
        DebugPrint(captargetposition);
        DebugPrint(EOFSTR);
        DebugPrint(F("- Current "));
        DebugPrint(capposition);
        DebugPrint(EOFSTR);
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
      DebugPrint(F("- => State_Moving#"));
      DebugPrint(EOFSTR);
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
      DebugPrint(F("- => State_Idle#"));
      DebugPrint(EOFSTR);
      break;

    default:
      MainStateMachine = State_Idle;
      break;
  }

#ifdef LOOPTIMETEST
  DebugPrint(F("- Loop End ="));
  DebugPrint(millis());
  DebugPrint(EOFSTR);
#endif

}
