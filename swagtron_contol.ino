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
 * 'SET <sp1> <sp2> = Set speed where <sp1> and <sp2> are int between -10 and 10
 *                    sp1 is for left wheel and sp2 is for rt. + is forward - is reverse
 ****************************************/

 /**************************************************************************
  * Static variables definitions
  **************************************************************************/
static CmdListener _cmdListener;
Hoverboard _p_hoverboard;
Hoverboard* Hoverboard::_instance = NULL;

void setup() {
   LOG_SERIAL.begin(LOG_SPEED);
   LOG_INIT_STREAM(LOG_LEVEL, &LOG_SERIAL);
   /** Wait 1s to get setup logs */
   delay(1000);
   _cmdListener.init();
   _p_hoverboard.init();
}

void loop() {  
  _cmdListener.pollCmd();
  _p_hoverboard.control();
  if(_p_hoverboard.runTimer > MAX_RUN_TIME) { 
    _cmdListener.emergencyStop();
  }
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
  if(cmd == "ON" || cmd == "on") {
    LOG_INFO_LN("Powering On");
    _p_hoverboard.powerontarget = true;
    return;
  } else if (cmd == "OFF" || cmd == "off") {
    LOG_INFO_LN("Powering Off");
    _p_hoverboard.powerontarget = false;
    return;
  } else if(cmd == "SET" || cmd == "set") {
    _p_hoverboard.leftspeedtarget = arg1;
    _p_hoverboard.rightspeedtarget = arg2;
    _p_hoverboard.runTimer = 0; //Reset timer
    LOG_INFO_LN("Setting speed to left (%d), right (%d)",_p_hoverboard.leftspeedtarget,
                  _p_hoverboard.rightspeedtarget);
  }
}

void CmdListener::emergencyStop()
{
  if(_p_hoverboard.leftspeedtarget != 0 || _p_hoverboard.rightspeedtarget != 0) {
    LOG_INFO_LN("Exceeded %d ms in MAX_RUN_TIME. Stopping hoverboard",MAX_RUN_TIME);
    _p_hoverboard.leftspeedtarget = 0;
    _p_hoverboard.rightspeedtarget = 0;
    _p_hoverboard.runTimer = 0;
  }
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
  //attachInterrupt(POWER_STATUS_PIN, Hoverboard::powerOffIt, RISING); 
  LEFT_MOTOR.begin(MOTOR_BAUD, MOTOR_CONFIG);
  RIGHT_MOTOR.begin(MOTOR_BAUD, MOTOR_CONFIG);
  if(POWER_STATUS_PIN == HIGH) {
    _e_state = POWER_ON;
  } else {
    _e_state = POWER_OFF;
  }
    
  LOG_INFO_LN("Hoverboard init done.");
}

void Hoverboard::powerOn(void)
{
  if(_e_state == POWER_OFF)
  {
    powerOnAsync();
    /*while(_e_state != POWER_ON)
    {
      /** TODO : IDLE control *
      LOG_DEBUG_LN("idle for power on state %d",_e_state);
    }*/
  }
  else
  {
    //LOG_DEBUG_LN("already powered on %d",_e_state);
  }
} 

void Hoverboard::powerOnAsync(void)
{
  leftspeedtarget = 0.0;
  rightspeedtarget = 0.0; 
  _e_state = POWERING_ON;
    
  digitalWrite(POWER_PIN, HIGH);
  LOG_DEBUG_LN("Is power_pin high?");
  
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
  } 
  else
  {
    LOG_DEBUG_LN("already powered off");
  }
}

void Hoverboard::powerOffAsync(void)
{
  if(_e_state != POWER_OFF || _e_state != POWERING_OFF)
  {
    leftspeedtarget = 0.0;
    rightspeedtarget = 0.0;
    _e_state = POWERING_OFF;
    digitalWrite(POWER_PIN, HIGH);
    /** POWER OFF detected on rising edge => increasing press duration makes power off
     * detected later */
    _timer.begin(Hoverboard::timerIt, SHORT_PRESS_DUR_MS*1000); 
  }
}

//POWER_PIN is held low by timer for both power on and off. 
void Hoverboard::timerIt(void)
{
  LOG_DEBUG_LN("In timer");
  if(_instance->_e_state == POWERING_ON)
  {
    //temp work around till we connect power pin
    while(digitalRead(POWER_STATUS_PIN) == LOW) {
      LOG_DEBUG_LN("Turn on the hoverboard...");
     delay(1000);
    }
    //end workaround
    digitalWrite(POWER_PIN, LOW);
    if(digitalRead(POWER_STATUS_PIN) == HIGH)
    {
      LOG_DEBUG_LN("Hoverboard turned on");
      _instance->_e_state = POWER_ON;
      digitalWrite(ONBOARD_LED, HIGH);
      _instance->startsignals();
    }
  }
  else if(_instance->_e_state == POWERING_OFF)
  {
    //temp work around till we connect power pin
    while(digitalRead(POWER_STATUS_PIN) == HIGH) {
      LOG_DEBUG_LN("Turn off the hoverboard...");
      delay(1000);
    }
    _instance->_e_state = POWER_OFF;
    LOG_DEBUG_LN("Hoverboard turned off");
    //end workaround
    digitalWrite(POWER_PIN, LOW);
    digitalWrite(ONBOARD_LED, LOW);
    /** Hoverbot not yet stopped - it is stopped on falling edge
     * => POWER OFF will be detected in POWER_OFF it */
  }
  //if motors are still running we cannot power off, do we need to check this?

  _instance->_timer.end();
}

void Hoverboard::powerOffIt(void)
{
  //detachInterrupt(POWER_STATUS_PIN); 
  if(POWER_STATUS_PIN == LOW)
  {
    _instance->_e_state = POWER_OFF;
  } 
  //attachInterrupt(POWER_STATUS_PIN, Hoverboard::powerOffIt, RISING); 
}

void Hoverboard::control(void)
{
  // This function should send commands to each motor in a loop
  if(powerontarget == true && (_e_state != POWER_ON && _e_state != POWERING_ON))
  {
    powerOn();
  }
  else if(powerontarget == false && (_e_state != POWER_OFF && _e_state != POWERING_OFF)) {
    powerOff();
  }
  else if(_e_state == POWER_ON){
    run();
  }
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
  LOG_INFO_LN("gyro turned on");

  //Warmup - is this dependent on direction? test out then simplify
  left_speed = 0;
  rt_speed = 0;
  
  int left_change, rt_change;
  //ramup/down depends on direction of motion.
  //for left motor > 0 is forward < 0 is backward
  if(leftspeedtarget > 0) {
    left_change = 1;
  } else {
    left_change = -1;
  }
  if(rightspeedtarget > 0) {
    rt_change = 1;
  } else {
    rt_change = -1;
  }

  uint16_t left_writeval, rt_writeval;
  for(uint8_t i = 0; i < warmuprepeat; i++) {
    left_speed += left_change;
    rt_speed += rt_change;
    for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++)
    {
      //reverse rt_motor direction by multiplying by -1
      if(index == 0 || index == 2) {
         left_writeval = (uint8_t)(left_speed & 0xff);
         rt_writeval = (uint8_t)(-1*rt_speed & 0xff);
        write9bit(left_writeval, rt_writeval);
      }
      else if (index == 1 || index == 3) {
         left_writeval = (uint8_t)(left_speed >> 8 & 0xff);
         rt_writeval = (uint8_t)(-1*rt_speed >> 8 & 0xff);
        write9bit(left_writeval, rt_writeval);
      }
      else {
        write9bit(stop_pattern[index], stop_pattern[index]); 
      }
    }
  }
  left_speed = 0;
  rt_speed = 0;
  LOG_INFO_LN("Warmup done");
  
}

void Hoverboard::run(void)
{
  //ramup/down depends on direction of motion.
  //for left motor > 0 is forward < 0 is backward
  if(leftspeedtarget >= 0 && left_speed < SPEED_LIMIT*256 && left_speed < leftspeedtarget*128) {
    left_speed += 1;
  } 
  else if (leftspeedtarget <= 0 && left_speed > -1*SPEED_LIMIT*256 && left_speed > leftspeedtarget*128) {
    left_speed -= 1;
  }
  //for rt motor it is reverse of left.
  if(rightspeedtarget >= 0 && rt_speed < SPEED_LIMIT*256 && rt_speed < rightspeedtarget*128) {
    rt_speed += 1;
  } 
  else if (rightspeedtarget <= 0 && rt_speed > -1*SPEED_LIMIT*256 && rt_speed > rightspeedtarget*128) {
    rt_speed -= 1;
  }
  uint16_t left_writeval, rt_writeval;
  for(uint8_t index = 0; index < GYRO_FRAME_LENGTH; index++)
  {
    //replace first 4 bytes with rampup/rampdown speed based on direction
    if(index == 0 || index == 2) {
      left_writeval = (uint8_t)(left_speed & 0xff);
      rt_writeval = (uint8_t)(-1*rt_speed & 0xff);
      write9bit(left_writeval, rt_writeval);
    }
    else if (index == 1 || index == 3) {
      left_writeval = (uint8_t)(left_speed >> 8 & 0xff);
      rt_writeval = (uint8_t)(-1*rt_speed >> 8 & 0xff);
      write9bit(left_writeval, rt_writeval);
    }
    else { 
        if(left_speed == 0) {
          left_writeval = stop_pattern[index];
        } else {
          left_writeval = run_pattern[index];
        }
        if(rt_speed == 0) {
          rt_writeval = stop_pattern[index];
        } else {
          rt_writeval = run_pattern[index];
        }
        write9bit(left_writeval, rt_writeval); 
      }
  }  
}

void Hoverboard::write9bit(uint16_t left_val, uint16_t rt_val) {
  LEFT_MOTOR.write9bit(left_val);
  RIGHT_MOTOR.write9bit(rt_val);
  LOG_INFO_LN("%d",left_val);
}
