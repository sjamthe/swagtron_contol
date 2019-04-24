/******************************************************************************
 * @file    swagtron_control
 * @author  Shirish Jamthe
 * @date    1/1/2018
 *
 * @brief Swagtron Hoverboard Control.
 *  
 *  This controller listens to commands coming on the Serial (usb connection)
 *  of teensy 3.2 board
 *  The command is modeled after commands in https://github.com/OpHaCo/hoverbot
 *
 * Revision History:
 * https://github.com/sjamthe/swagtron_control.git
 * 
 * LICENSE :
 * swagtron_control (c) by Shirish Jamthe
 * project_name is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/
#include "libraries/swagtron_contol.h"

/*****************************************
 * CONTROL cmd format
 * 
 * 'ON' = POWER_ON
 * 'OFF' = POWER_OFF
 * 'F' <msecs> (how many seconds to move forward) cannot be more than MAX_RUN_TIME)
 * 'B' <msecs> (how many seconds to move backword) cannot be more than MAX_RUN_TIME)
 * 'L' <degree*10> turn left
 * 'R' <degree*10> rurn right
 ****************************************/

 /**************************************************************************
  * Static variables definitions
  **************************************************************************/
static CmdListener _cmdListener;
Hoverboard _p_hoverboard;
Hoverboard* Hoverboard::_instance = NULL;
bool lognow = true;

void setup() {
   LOG_SERIAL.begin(LOG_SPEED);
   LOG_INIT_STREAM(LOG_LEVEL, &LOG_SERIAL);
   delay(2000);

   /** Wait 1s to get setup logs */
   _cmdListener.init();
   _p_hoverboard.init();
}

void loop() {  
  _cmdListener.pollCmd();
  _p_hoverboard.control();
}


CmdListener::CmdListener()
{
  //Constructor of class, to initialize variables.
}

void CmdListener::init(void)
{
  //init is called from setup, should be only once.
  LOG_INFO_LN("starting swagtron cmd listener ...");
}

/*
 * pollCmd: is called from the loop(), reads for incoming control commands.
 */
void CmdListener::pollCmd(void)
{
  String inString = "";    // string to hold input
  String cmd = "";
  int args = 0, arg1=0, arg2=0;
  bool minus = false;
   //LOG_INFO_LN("pollCmd");
   while (Serial.available() > 0) {
    int inChar = Serial.read();
    //Serial.print(inChar);
    if (isAlpha(inChar) || isdigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    } else {
      switch(inChar) {
        case '-':
          minus = true;
         break;
        case ' ':
        case '\n':
          //Serial.println(inString);
          if(args == 0) {
            cmd = inString;
            inString = "";
            args++;
          } else if (args == 1) {
            arg1 = inString.toInt();
            if(minus)
              arg1 *= -1;
            inString = "";
            args++;
          } else if (args == 2) {
            arg2 = inString.toInt();
            if(minus)
              arg2 *= -1;
            inString = "";
            args++;
          } else {
            inString = "";
            minus = false;
            args = 0; arg1 = 0; arg2 = 0;
            cmd = "";
          }
          if(inChar == '\n') {
            handleCommand(cmd, arg1, arg2);
            inString = "";
            minus = false;
            args = 0; arg1 = 0; arg2 = 0;
            cmd = ""; 
          }
          break; 
          
        default:
          LOG_INFO_LN("Invalid command will be skipped");
          Serial.println(cmd);
          inString = "";
          minus = false;
          args = 0; arg1 = 0; arg2 = 0;
          cmd = "";
      }
    }
  }
}

void CmdListener::handleCommand(String cmd, int arg1, int arg2)
{
  bool prev_motion = _p_hoverboard.motion;
  
  if(cmd == "ON" || cmd == "on") {
    LOG_INFO_LN("Powering On");
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_ON;
    return;
  } else if (cmd == "OFF" || cmd == "off") {
    LOG_INFO_LN("Powering Off");
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_OFF;
    return;
  } else if(cmd == "F" || cmd == "f") {
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_FORWARD;
    _p_hoverboard.forward = true;
    _p_hoverboard.motion = true;
    _p_hoverboard.turn = false;
    _p_hoverboard.idlegap = false;
    _p_hoverboard.runtime = arg1; //how long to run
    LOG_INFO_LN("Moving Forward by %d",arg1);
  } else if(cmd == "B" || cmd == "b") {
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_BACKWARD;
    _p_hoverboard.forward = false;
    _p_hoverboard.motion = true;
    _p_hoverboard.turn = false;
    _p_hoverboard.idlegap = false;
    _p_hoverboard.runtime = arg1; //how long to run
    LOG_INFO_LN("Moving Backward by %d",arg1);
  } else if(cmd == "L" || cmd == "l") {
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_LEFT;
    _p_hoverboard.forward = true;
    _p_hoverboard.motion = true;
    _p_hoverboard.turn = true;
    _p_hoverboard.idlegap = false;
    _p_hoverboard.runtime = arg1; //how long to run
    LOG_INFO_LN("Turn Left by %d",arg1);
  } else if(cmd == "R" || cmd == "r") {
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_RIGHT;
    _p_hoverboard.forward = false;
    _p_hoverboard.motion = true;
    _p_hoverboard.turn = true;
    _p_hoverboard.idlegap = false;
    _p_hoverboard.runtime = arg1; //how long to run
    LOG_INFO_LN("Turn Right by %d",arg1);
  } else {
    _p_hoverboard._e_cmd = _p_hoverboard.CMD_UNKNOWN;
  }

  //Don't reset timer of cmd is same as cmd as it was issued within MAX_RUN_TIME
  if (_p_hoverboard.turn) {
    _p_hoverboard.runTimer = 0;
    _p_hoverboard.hallCounter = 0;   
  } else {
    if(prev_motion && _p_hoverboard.runTimer < MAX_RUN_TIME && 
        _p_hoverboard._e_cmd == _p_hoverboard._e_pcmd) {
      //don't reset if we were moving in the same way
      LOG_DEBUG_LN("%d - timer not rest at %d for cmd %d",micros(), _p_hoverboard.runTimer, _p_hoverboard._e_cmd);
    } else {
      _p_hoverboard.runTimer = 0;
      _p_hoverboard.hallCounter = 0;
    }
  }
  _p_hoverboard._e_pcmd = _p_hoverboard._e_cmd;
}

//hoverboard class
Hoverboard::Hoverboard()
{
  //constructor class.
  _instance = this;
}

void Hoverboard::init(void)
{
  //called only once from setup()
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(POWER_STATUS_PIN, INPUT); 
  pinMode(POWER_PIN, OUTPUT);
  pinMode(HALL_PIN, INPUT);

  LEFT_MOTOR.begin(MOTOR_BAUD, MOTOR_CONFIG);
  RIGHT_MOTOR.begin(MOTOR_BAUD, MOTOR_CONFIG);
  //delay(1000);
  if(digitalRead(POWER_STATUS_PIN) == HIGH) {
    //power should not be on. turn it off
     LOG_INFO_LN("Power is On. Turning it off");
    _e_state = POWER_ON;
    powerOff();
  } else {
    _e_state = POWER_OFF;
    LOG_INFO_LN("Power is off.");
  }
  LOG_INFO_LN("Hoverboard init done.");
}

void Hoverboard::hallPinCounter() {
  // read the pushbutton input pin:
  hallPinState = digitalRead(HALL_PIN);

  // compare the buttonState to its previous state
  if (hallPinState != lastHallPinState) {
    // if the state has changed, increment the counter
    if (hallPinState == LOW) {
      hallCounter++;
      if(hallCounter > 0) {
        hallMicroDiff = micros()-hallMicros;
        LOG_INFO_LN("hallCounter: %d - micros %d",hallCounter, hallMicroDiff);
        hallMicros = micros();
      }
    } 

  }
  // save the current state as the last state, 
  //for next time through the loop
  lastHallPinState = hallPinState;
}

void Hoverboard::powerOn(void)
{
  if(_e_state == POWER_OFF || _e_state == OUT_OF_ENUM_STATE )
  {
    powerOnAsync();
  }
} 

void Hoverboard::powerOnAsync(void)
{
  motion = false; 
  _e_state = POWERING_ON;
    
  digitalWrite(POWER_PIN, HIGH);
  //LOG_DEBUG_LN("Is power_pin high?");
  
  //The timer helps hold the pin high for SHORT_PRESS_DUR_MS milliseconds
  _timer.begin(Hoverboard::timerIt, SHORT_PRESS_DUR_MS*1000); 
}

void Hoverboard::powerOff(void)
{
  if(_e_state != POWER_OFF)
  {
    powerOffAsync();
    while(_e_state != POWER_OFF)
    {
      /** TODO : IDLE control */
    }
    LOG_DEBUG_LN("power off complete");
  } 
}

void Hoverboard::powerOffAsync(void)
{
  if(_e_state != POWER_OFF || _e_state != POWERING_OFF)
  {
    motion = false;
    _e_state = POWERING_OFF;
    digitalWrite(POWER_PIN, HIGH);
    /** POWER OFF detected on rising edge => increasing press duration makes power off
     * detected later */
    _timer.begin(Hoverboard::timerIt, SHORT_PRESS_DUR_MS*1000); 
  }
}

//POWER_PIN is held high by timer for both power on and off. 
void Hoverboard::timerIt(void)
{
  LOG_DEBUG_LN("In timer");
  _instance->_timer.end();
  if(_instance->_e_state == POWERING_ON && digitalRead(POWER_STATUS_PIN) == HIGH)
  {
    digitalWrite(POWER_PIN, LOW);
    LOG_DEBUG_LN("%d - Hoverboard turned on",micros());
    _instance->_e_state = POWER_ON;
    digitalWrite(ONBOARD_LED, HIGH);
  }
  else if(_instance->_e_state == POWERING_OFF)
  {
    digitalWrite(POWER_PIN, LOW);
    while(digitalRead(POWER_STATUS_PIN) == HIGH) {
      delay(500);
      LOG_DEBUG_LN("%d - Waiting for POWER_STATUS_PIN to go low...",micros());
    }
    _instance->_e_state = POWER_OFF;
    LOG_DEBUG_LN("%d - Hoverboard turned off",micros());
    digitalWrite(ONBOARD_LED, LOW);
  }
}

void Hoverboard::control(void)
{
  // This function should send commands to each motor in a loop
  if(_e_cmd == CMD_ON)
  {
    powerOn();
  }
  else if(_e_cmd == CMD_OFF) {
    powerOff();
  }
  else if(_e_state == POWER_ON) {
    startsignals();
  }
  else if(_e_state == RUNNING) {
    run();
  }
  _e_cmd = _p_hoverboard.CMD_UNKNOWN;
}

//start sending initial signals to motors. this is called after power is turned on.
void Hoverboard::startsignals(void)
{
  //start bit
  write9bit(0, 0); 
  //pattern1
  for(uint8_t i = 0; i < pattern1repeat; i++) {
    for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++) {
      write9bit(start_pattern1[index], start_pattern1[index]);
    }  
  }
  //pattern2
  for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++) {
      write9bit(start_pattern2[index], start_pattern2[index]);
  } 
  //pattern3
  for(uint8_t i = 0; i < pattern3repeat; i++) {
    for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++) {
      write9bit(start_pattern3[index], start_pattern3[index]);
    }  
  }
  //warmup/idle pattern
  for(uint8_t i = 0; i < warmuprepeat; i++) {
    for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++) {
      write9bit(stop_pattern[index], stop_pattern[index]); 
    }
  }
  _e_state = RUNNING;
  lognow = true;
  LOG_INFO_LN("%d - Warmup done, motors are idle", micros());
}

//if power if on this function has to send signals all to motors all the time.
void Hoverboard::run(void)
{
  if(motion) {
    //measure movement from hallpin
    hallPinCounter();
    //emergency stop
    if(runTimer > MAX_RUN_TIME && !turn) { 
      LOG_INFO_LN("Exceeded %d ms in MAX_RUN_TIME. Stopping hoverboard",MAX_RUN_TIME);
      motion = false;
    }
      //if we are turning we need to stop turning after TURN_TIME_LIMIT
    if(turn && runTimer > TURN_TIME_LIMIT) { 
      LOG_INFO_LN("Exceeded %d ms in TURN_TIME_LIMIT. Stopping hoverboard",TURN_TIME_LIMIT);
      motion = false;
      turn = false;
    }
    /*if(motion && runTimer > runtime) {
      motion = false;
    }*/
    //test runtime as runHallCounter
    if(motion && hallCounter >= runtime) {
      motion = false;
    }
    //introduce idlegap here, to keep constant speed we should run not more than 
    //2000 ms then idle for 1000 ms
    /*if(int(runTimer/500)%3 == 2 && !turn) {
      idlegap = true; //was true
    } else {
      idlegap = false;
    }*/
  }
  
  if(turn) {
    if(forward) {
      left_speed = TURN_SPEED_LIMIT;
      rt_speed = TURN_SPEED_LIMIT; //same speeds to turn
    } else {
      left_speed = 65535-TURN_SPEED_LIMIT;
      rt_speed = 65535-TURN_SPEED_LIMIT;
    }
  } else {
    int limit = SPEED_LIMIT;
    // testing throttling for arbitrary 100ms
    if(hallMicroDiff < 100000 ) {
      limit = 5;
    }
    else if(hallMicroDiff < 60000 ) {
      limit = 1;
    }
    if(forward) {
      left_speed = limit;
      rt_speed = 65535 - limit;
    } else {
      left_speed = 65535 - limit;
      rt_speed = limit;
    }
  }
  //for idlegap speeds are 0
  if(idlegap || !motion) {
    left_speed = 0;
    rt_speed = 0;
  }
  
  uint16_t left_writeval, rt_writeval;
  for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++)
  {
    //replace first 4 bytes with rampup/rampdown speed based on direction
    if(index == 0 || index == 2) {
      left_writeval = (uint8_t)(left_speed & 0xff);
      rt_writeval = (uint8_t)(rt_speed & 0xff);
    }
    else if (index == 1 || index == 3) {
      left_writeval = (uint8_t)(left_speed >> 8 & 0xff);
      rt_writeval = (uint8_t)(rt_speed >> 8 & 0xff);
    }
    else { 
        if(motion) {
          left_writeval = run_pattern[index];
          rt_writeval = run_pattern[index];
        } else {
          left_writeval = stop_pattern[index];
          rt_writeval = stop_pattern[index];
        }
    }
    write9bit(left_writeval, rt_writeval);
  }
  if(lognow) {
    LOG_INFO_LN("%d - started 1st signal", micros()); 
    lognow = false;
  }
}

void Hoverboard::write9bit(uint16_t left_val, uint16_t rt_val) {
  LEFT_MOTOR.write9bit(left_val);
  RIGHT_MOTOR.write9bit(rt_val);
  //LOG_INFO_LN("%d",rt_val);
}
