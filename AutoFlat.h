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

#define ON                  1             // EL panel or motor is on
#define OFF                 0             // EL panel or motor is off
#define LIGHTMAX            255           // maximum light brightness
#define CAPOPEN             2             // cap is open
#define CAPCLOSED           1             // cap is closed
#define CAPMOVING           0             // servo is moving
#define EEPROMWRITEINTERVAL 10000L        // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
#define VALIDDATAFLAG       9999          // valid eeprom data flag
#define SERIALPORTSPEED     9600          // 9600, 14400, 19200, 28800, 38400, 57600
#define QUEUELENGTH         20            // number of commands that can be saved in the serial queue
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
  #define DebugPrint(...) Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DebugPrintln(...) Serial.println(__VA_ARGS__) //DPRINTLN is a macro, debug print with new line
#else
  #define DebugPrint(...)                               //now defines a blank line
  #define DebugPrintln(...)                             //now defines a blank line
#endif
