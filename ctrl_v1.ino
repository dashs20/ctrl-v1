/*
                             tttt                            lllllll                              1111111   
                          ttt:::t                            l:::::l                             1::::::1   
                          t:::::t                            l:::::l                            1:::::::1   
                          t:::::t                            l:::::l                            111:::::1   
    ccccccccccccccccttttttt:::::ttttttt   rrrrr   rrrrrrrrr   l::::l      vvvvvvv           vvvvvvv1::::1   
  cc:::::::::::::::ct:::::::::::::::::t   r::::rrr:::::::::r  l::::l       v:::::v         v:::::v 1::::1   
 c:::::::::::::::::ct:::::::::::::::::t   r:::::::::::::::::r l::::l        v:::::v       v:::::v  1::::1   
c:::::::cccccc:::::ctttttt:::::::tttttt   rr::::::rrrrr::::::rl::::l         v:::::v     v:::::v   1::::l   
c::::::c     ccccccc      t:::::t          r:::::r     r:::::rl::::l          v:::::v   v:::::v    1::::l   
c:::::c                   t:::::t          r:::::r     rrrrrrrl::::l           v:::::v v:::::v     1::::l   
c:::::c                   t:::::t          r:::::r            l::::l            v:::::v:::::v      1::::l   
c::::::c     ccccccc      t:::::t    ttttttr:::::r            l::::l             v:::::::::v       1::::l   
c:::::::cccccc:::::c      t::::::tttt:::::tr:::::r           l::::::l             v:::::::v     111::::::111
 c:::::::::::::::::c      tt::::::::::::::tr:::::r           l::::::l              v:::::v      1::::::::::1
  cc:::::::::::::::c        tt:::::::::::ttr:::::r           l::::::l               v:::v       1::::::::::1
    cccccccccccccccc          ttttttttttt  rrrrrrr           llllllll                vvv        111111111111
*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Included libraries
#include <Wire.h>
#include <sbus.h>
#include "MPU6050.h"

// Time variables instantiated
float dt;
unsigned long current_time, prev_time;

// Radio commands instantiated
float throttle_cmd;
float roll_cmd;
float pitch_cmd;
float yaw_cmd;
float switch_cmd;

// Error variables instantiated
float curr_roll_err;
float curr_pitch_err;
float curr_yaw_err;
float prev_roll_err = 0;
float prev_pitch_err = 0;
float prev_yaw_err = 0;

// Integral term variables instantiated
float curr_roll_i = 0;
float curr_pitch_i = 0;
float curr_yaw_i = 0;

// PID values
float roll_p = 0.30;
float roll_i = 0.001;
float roll_d = 0.30;
float pitch_p = 0.30;
float pitch_i = 0.001;
float pitch_d = 0.30;
float yaw_p = 0.3;
float yaw_i = 0.001;
float yaw_d = 0.2;

// SbusRx object instantiated - member of Boulderflight class
bfs::SbusRx sbus_rx(&Serial2);
bfs::SbusData data;

// IMU variables declared, MPU object instantiated
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
MPU6050 mpu6050;
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_SCALE GYRO_FS_SEL_1000

/* Declare pins (for ctrl v1 FC, pin assignments are as follows:
/
/ Motor 1: pin 3, back left
/ Motor 2: pin 4, front left
/ Motor 3: pin 5, front right
/ Motor 4: pin 6, back right
/
*/
const int m1Pin = 3;
const int m2Pin = 4;
const int m3Pin = 5;
const int m4Pin = 6;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

// Max rates
float min_throttle = 10; // minimum throttle allowed when drone is armed.
float max_pitch = 650; // max pitch rate (deg/s)
float max_yaw = 650;
float max_roll = 650;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Primary Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/                             /
/     Main                    /
/         Setup               /
/             Function        /
/                             /
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void setup() 
{
  // Begin serial communication at a baud rate of 115200
  Serial.begin(115200);
  delay(500);

  // RX setup
  sbus_rx.Begin();

  // IMU setup
  IMUinit();
  delay(20);

  // ESC Setup (set pins to output PWM signal)
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/                             /
/         Main                /
/             Loop            /
/                             /
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void loop() 
{
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  
  dt = (current_time - prev_time)/1000000.0;
  
  // get RX data from pilot and scale them to max rates
  get_rx();
  
  // Obtain IMU data
  getIMUdata();

  // Compute State Error
  get_error();

  // Compute motor commands
  if(switch_cmd > 90)
  { // If arm switch is flipped, populate mN_command_PWM variables with autopilot signals.
    get_motor_commands();
  }
  else
  { // Oneshot125 minimum pulsewidth <=> zero throttle
    m1_command_PWM = 125;
    m2_command_PWM = 125;
    m3_command_PWM = 125;
    m4_command_PWM = 125;
  }
  // Send PWM signals to the pins.
  commandMotors();
  
  // Save previous errors for integral of error
  prev_roll_err = curr_roll_err;
  prev_pitch_err = curr_pitch_err;
  prev_yaw_err = curr_yaw_err;

  // Optional debug printing

  /*
  Serial.print("yaw: ");
  Serial.print(yaw_cmd);
  Serial.print("throttle: ");
  Serial.print(throttle_cmd);
  Serial.print("pitch: ");
  Serial.print(pitch_cmd);
  Serial.print("roll: ");
  Serial.print(roll_cmd);
  Serial.print("switch: ");
  Serial.println(switch_cmd);
  */

  // Determine headroom
  int ping = micros();
  
  // Main loop delay
  loopRate(2000);
  
  int pong = micros();
  float headroom = pong - ping;
  Serial.println(headroom); // Print headroom (in microseconds) to serial monitor
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~ PID function ~~~ /
/
/ Function:
/   PID controller
/
/ Inputs:
/ - curr_err (currnet error, deg/s) error between current pilot input and drone state
/ - perv_err (previous error, deg/s) same as above, but for previous time step
/ - p_gain (unitless) proportional gain for PID controller
/ - d_gain (unitless) derivative gain for PID controller
/ - i_gain (unitless) intergal gain for PID controller
/ - curr_i (deg) integral of error signal. Integral of deg/s -> deg
/
/ Output(s):
/ - signal (unitless) output signal from PID controller. Typically for commanding motors.
*/
float PID(float curr_err,float prev_err,float p_gain,float d_gain, float i_gain, float curr_i)
{
  return p_gain * curr_err + d_gain * (curr_err - prev_err) + i_gain * curr_i;
}

/*~~~ Motor Command Function ~~~ /
/
/ Function:
/   Orchestrate generation of curated PWM signal to motors. Generates a signal suitable for oneshot125 ESCs.
/   Oneshot125 signals are PWM signals that range from 125-250 microseconds, and scales with throttle.
/
/ Inputs:
/ - none (generates PID signal from error, which is the difference between the IMU's signal and the user's input)
/
/ Output(s):
/ - none (effects motor command PWM variables)
*/

void get_motor_commands()
{

  // step 1 is to compute the roll, pitch and yaw PID commands based on all the errors.
  float auto_roll = PID(curr_roll_err,prev_roll_err,roll_p,roll_d,roll_i,curr_roll_i);
  float auto_pitch = PID(curr_pitch_err,prev_pitch_err,pitch_p,pitch_d,pitch_i,curr_pitch_i);
  float auto_yaw = PID(curr_yaw_err,prev_yaw_err,yaw_p,yaw_d,yaw_i,curr_yaw_i);

  // save current integral value to previous intergal value variable
  curr_roll_i += curr_roll_err;
  curr_pitch_i += curr_pitch_err;
  curr_yaw_i += curr_yaw_err;

  // step 2 is to mix them together into commands.
  float auto_m1 = throttle_cmd - auto_roll + auto_pitch + auto_yaw; // back left
  float auto_m2 = throttle_cmd - auto_roll - auto_pitch - auto_yaw; // front left
  float auto_m3 = throttle_cmd + auto_roll - auto_pitch + auto_yaw; // front right
  float auto_m4 = throttle_cmd + auto_roll + auto_pitch - auto_yaw; // back right

  // step 3 is to constrain them between 0 and 100 percent for each motor
  auto_m1 = diy_constrain(auto_m1,min_throttle,100.0);
  auto_m2 = diy_constrain(auto_m2,min_throttle,100.0);
  auto_m3 = diy_constrain(auto_m3,min_throttle,100.0);
  auto_m4 = diy_constrain(auto_m4,min_throttle,100.0);

  // step 4 is to translate them into oneshot_125 language, and send it!
  m1_command_PWM = 125 + auto_m1 * 125.0/100.0;
  m2_command_PWM = 125 + auto_m2 * 125.0/100.0;
  m3_command_PWM = 125 + auto_m3 * 125.0/100.0;
  m4_command_PWM = 125 + auto_m4 * 125.0/100.0;
  
}

/*~~~ Constrain Function ~~~ /
/
/ Function:
/   Constrain a given variable to ensure it's within expected bounds
/
/ Inputs:
/ - var (unitless) variable to constrain
/ - mini (unitless) lower bound
/ - maxi (unitless) upper bound
/
/ Output(s):
/ - constrained value
*/
float diy_constrain(float var, float mini, float maxi)
{
  if(var > maxi)
  {
    var = maxi;
  }

  if(var < mini)
  {
    var = mini;
  }

  return var;
}

/*~~~ Get Error Function ~~~ /
/
/ Function:
/   Populate global error variables with
/
/ Inputs:
/ - nothing
/
/ Output(s):
/ - nothing
*/
void get_error()
{
  curr_roll_err = GyY - roll_cmd;
  curr_pitch_err = GyX - pitch_cmd;
  curr_yaw_err = GyZ - yaw_cmd;
}

/*~~~ Get Receiver Data Function ~~~ /
/
/ Function:
/   Populates "data" variable - member of bfs class
/
/ Inputs:
/ - nothing
/
/ Output(s):
/ - nothing
*/

void get_rx()
{
  // obtain raw signal data
  if (sbus_rx.Read()) 
  {
    data = sbus_rx.data();
  }

  // populate cmd variables with scaled values (0-100)
  roll_cmd = (data.ch[1] - 173) * 100.0/1637.0;
  pitch_cmd = (data.ch[2] - 173) * 100.0/1637.0;
  throttle_cmd = (data.ch[0] - 173) * (100.0-min_throttle)/1637.0 + min_throttle;
  yaw_cmd = (data.ch[3] - 173) * 100.0/1637.0;
  switch_cmd = (data.ch[4] - 173) * 100.0/1637.0;

  // scale yaw, pitch and roll cmd to match desired max rates
  yaw_cmd = (yaw_cmd - 50) * max_yaw/50.0;
  pitch_cmd = (pitch_cmd - 50) * max_pitch/50.0;
  roll_cmd = (roll_cmd - 50) * max_roll/50.0;

  yaw_cmd = diy_constrain(yaw_cmd,-1*max_yaw,max_yaw);
  pitch_cmd = diy_constrain(pitch_cmd,-1*max_pitch,max_pitch);
  roll_cmd = diy_constrain(roll_cmd,-1*max_roll,max_roll);
  
  throttle_cmd = diy_constrain(throttle_cmd,0.0,100.0);

}

/*~~~ Loop Rate Function ~~~ /
/
/ Function:
/   To regulate loop rate of main loop.
/   
/ Inputs:
/ - freq (hz) looprate
/
/ Output(s):
/ - none
*/

void loopRate(int freq) 
{
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) 
  {
    checker = micros();
  }
}

/*~~~ Command Motors ~~~ /
/
/ Function:
/   Construct PWM commands, and send them to the motor pins.
/   
/ Inputs:
/ - none (uses mN_command_PWM variables to generate PWM signal.)
/
/ Output(s):
/ - none
*/

void commandMotors() 
{
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 4 ) 
  { //Keep going until final (4th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) 
    {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) 
    {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) 
    {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) 
    {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
  }
}

/*~~~ Get IMU Data ~~~ /
/
/ Function:
/   Get, and scale, data from IMU. Populate global GyX, GyY, and GyZ variables with rates (deg/s)
/
/ Inputs:
/ - none
/
/ Output(s):
/ - none
*/

void getIMUdata() 
{
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  GyX = GyX/32.8;
  GyY = GyY/32.8;
  GyZ = GyZ/32.8;
}

/*~~~ Initialize IMU ~~~ /
/
/ Function:
/   Populates "data" variable - member of bfs class
/
/ Inputs:
/ - nothing
/
/ Output(s):
/ - nothing
*/

void IMUinit() 
{
  //DESCRIPTION: Initialize IMU
  Wire.begin();
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
  mpu6050.initialize();
    
  if (mpu6050.testConnection() == false) 
  {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  //From the reset state all registers should be 0x00, so we should be at
  //max sample rate with digital low pass filter(s) off.  All we need to
  //do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
}
