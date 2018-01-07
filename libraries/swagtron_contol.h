#ifndef SWAGTRON_CONTROL_H
#define SWAGTRON_CONTROL_H

/**************************************************************************
 * Include Files
 **************************************************************************/
#include <logger_config.h>
/**************************************************************************
 * Manifest Constants
 **************************************************************************/
#define LOG_SPEED 500000
#define LOG_SERIAL Serial
#define MAX_RUN_TIME 1000*4 // 8000 is not exceeding but too fast
#define SPEED_LIMIT 20  //20 is lower than 40, 80, 120 (1 sec). almost no motion on 10

#define TURN_TIME_LIMIT 4000
#define TURN_SPEED_LIMIT 96 //calibrated for 3600/circle

#define SHORT_PRESS_DUR_MS 500

#define MOTOR_BAUD 52600
#define MOTOR_CONFIG SERIAL_9N1
#define LEFT_MOTOR Serial1
#define RIGHT_MOTOR Serial3

#define ONBOARD_LED 13
#define POWER_PIN 14
#define POWER_STATUS_PIN 2

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
* Static members definitions
 **************************************************************************/
 //motor controls
const uint8_t GYRO_FRAME_LENGTH = 11;
const uint8_t GYRO_CONTACT_CLOSED_BYTE = 85;
const uint8_t GYRO_CONTACT_OPENED_BYTE = 170;
const uint16_t GYRO_FRAME_END = 256;
const uint16_t start_pattern1[GYRO_FRAME_LENGTH] = {
   0,0,0,0,GYRO_CONTACT_OPENED_BYTE,
   0,0,0,0,0,GYRO_FRAME_END
};
const int pattern1repeat = 19;
const uint16_t start_pattern2[GYRO_FRAME_LENGTH] = {
   0,0,0,0,GYRO_CONTACT_OPENED_BYTE,
   88,88,0,0,0,GYRO_FRAME_END
};
const int pattern2repeat = 1;
const uint16_t start_pattern3[GYRO_FRAME_LENGTH] = {
   252,255,252,255,GYRO_CONTACT_OPENED_BYTE,
   88,88,0,0,0,GYRO_FRAME_END
};
const int pattern3repeat = 3;
const uint16_t stop_pattern[GYRO_FRAME_LENGTH] = {
   0,0,0,0,GYRO_CONTACT_OPENED_BYTE,
   88,88,0,0,0,GYRO_FRAME_END
};
const uint16_t run_pattern[GYRO_FRAME_LENGTH] = {
   0,0,0,0,GYRO_CONTACT_CLOSED_BYTE,
   88,88,0,0,0,GYRO_FRAME_END
};
const int warmuprepeat = 7;

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 *  Class Declarations
 **************************************************************************/
 /**
  * Class that controls hoverboard.
  */
 class Hoverboard
 {
   public:
     typedef enum {
      CMD_OFF = 0,
      CMD_ON,
      CMD_FORWARD,
      CMD_BACKWARD,
      CMD_LEFT,
      CMD_RIGHT,
      CMD_UNKNOWN
    } EHoverboardCmd;

    volatile EHoverboardCmd _e_cmd = CMD_UNKNOWN;
    volatile EHoverboardCmd _e_pcmd = CMD_UNKNOWN;
    int leftspeedtarget = 0;
    int rightspeedtarget = 0;
    bool forward, turn, motion, idlegap;
    uint16_t runtime; //how long to move forward or back
    elapsedMillis runTimer = 0; //Is reset everytime Set is called.
    void control(void);
    void init(void);
    Hoverboard();

  private:
    typedef enum {
     POWER_OFF = 0,
     POWER_ON,
     POWERING_ON,
     POWERING_OFF,
     RUNNING,
     OUT_OF_ENUM_STATE
    } EHoverboardState;

    static Hoverboard* _instance;
    volatile EHoverboardState _e_state = OUT_OF_ENUM_STATE;
    IntervalTimer _timer;
    static void timerIt(void);
    void powerOn(void);
    void powerOnAsync(void);
    void powerOff(void);
    void powerOffAsync(void);
    static void powerOffIt(void);
    void startsignals(void); //This starts sending gyro signals.
    void run(void); //This function is called repetedly to control speed
    void write9bit(uint16_t leftmotor_byte, uint16_t rtmotor_byte);
    int left_speed, rt_speed; //actual speed set for motors
 };


 /**
  * Class that listen hoverboard commands over UART to control it.
  */
 class CmdListener
 {
   public:
     CmdListener();
     void init(void);
     void pollCmd(void);
     void emergencyStop(void);

   private:
     void handleCommand(String cmd, int arg1, int arg2);
 };

#endif /** SWAGTRON_CONTROL_H **/
