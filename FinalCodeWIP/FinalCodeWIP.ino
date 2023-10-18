// Libraries
#include "RPi_Pico_TimerInterrupt.h"
#include "RP2040_PWM.h"
#include <Servo.h>

// PINS
#define PIN_J0_SERVO  0
#define PIN_J1_SERVO  2
#define PIN_J2_SERVO  10
#define PIN_J3_SERVO  6
#define PIN_J4_SERVO  8
#define PIN_J5_SERVO  4
#define PIN_MUX_CTRL  16
#define PIN_RELAY     17
#define PIN_MUX1      26
#define PIN_MUX2      27
#define PIN_MUX3      28

// PARAMETERS
#define PWM_FREQUENCY         250   // [Hz]
#define SMALL_SERVO_MIN_PWM   500   // [us]
#define SMALL_SERVO_MAX_PWM   2500  // [us]
#define SMALL_SERVO_MIN_ANGLE 0     // [Degrees]
#define SMALL_SERVO_MAX_ANGLE 180   // [Degrees]
#define SAMPLING_FREQUENCY    100   // [Hz]
#define PWM_QUICK_TEST        false // [true / false]
#define MAX_RECORDING_TIME    15    // [s]
#define ADC_RES               12    // [bit]
#define POTENTIOMETER_TEST    false // [true / false]
#define RAMP_MODE             true  // [true / false]
#define SHOWCASE_MODE         true  // [true / false]

// CALCULATED PARAMETERS
const int sample_period = 1000000 / SAMPLING_FREQUENCY;
const int playback_period = sample_period / 10;
const int max_samples = MAX_RECORDING_TIME * SAMPLING_FREQUENCY;

// CONFIGURABLE PARAMETERS
float playback_speed = 1.00; // Range from 0.1 to 10

// Variables
float   pulse_width         = 0.00;
float   duty_cycle          = 0.00;
float   period              = 1000000.00 / PWM_FREQUENCY;

// Min and Max potentiometer positions for all of the joints
int j0_min, j0_max, j1_min, j1_max, j2_min, j2_max, j3_min, j3_max, j4_min, j4_max, j5_min, j5_max;

// Serial Communication Variables
String  serial_message      = "";             // String for holding the Serial message
bool    new_message         = false;          // Flag for if there is a new message 

// System variables
bool    calibration_done    = false;          // Flag for keeping the info if the calibration procedure is done or not
volatile int mux_position   = 1;

// Angle recordings
int j0_pos[max_samples] = {};
int j1_pos[max_samples] = {};
int j2_pos[max_samples] = {};
int j3_pos[max_samples] = {};
int j4_pos[max_samples] = {};
int j5_pos[max_samples] = {};

// Recording variables
volatile bool   record_flag         = false;
volatile int    sample_num          = 0;     
volatile bool   recorded_data       = false;

// Playback variables
volatile bool playback_flag         = false;
volatile int playback_trigger = 0;
volatile int playback_counter = 0;

// Ramp variables
int j0_goal, j1_goal, j2_goal, j3_goal, j4_goal, j5_goal;
int J0, J1, J2, J3, J4, J5; 

// Process variables
bool    first               = false;

// Objects / Instances
RP2040_PWM* PWM_Instance[6];  // PWM used for the Servos
RPI_PICO_Timer Timer_Sampling(0);
RPI_PICO_Timer Timer_Playback(1);
RPI_PICO_Timer Timer_Ramp(2);

// FUNCTIONS

// Function for toggling the MUX
void MUX(int pos)
{
  if (pos == 1)
  {
    digitalWrite(PIN_MUX_CTRL, HIGH);
    return;
  }
  else if (pos == -1)
  {
    digitalWrite(PIN_MUX_CTRL, LOW);
    return;
  }
  Serial.println("ERROR - MUX Function - Parameters pos needs to be either 1 or -1");
  Serial.print("pos: ");
  Serial.println(pos);
  return;
}

// Function for clearing out all Arrays
void ClearArr()
{
  for (int i = 0; i <= max_samples; i++)
  {
    j0_pos[i] = 90;
    j1_pos[i] = 90;
    j2_pos[i] = 90;
    j3_pos[i] = 90;
    j4_pos[i] = 90;
    j5_pos[i] = 90;
  }
  return;
}

// Function for updating the Ramp parameters
void UpdateMotors(int a0, int a1, int a2, int a3, int a4, int a5)
{
  if(a0 >= 0 && a0 <= 180) j0_goal = a0;
  if(a1 >= 0 && a1 <= 180) j1_goal = a1;
  if(a2 >= 0 && a2 <= 180) j2_goal = a2;
  if(a3 >= 0 && a3 <= 180) j3_goal = a3;
  if(a4 >= 0 && a4 <= 180) j4_goal = a4;
  if(a5 >= 0 && a5 <= 180) j5_goal = a5;
  return;
}

void UpdateMotor(int joint, int angle)
{
  if (joint < 0 || joint > 5) return;
  if (angle < 0 || angle > 180) return;
  if (joint == 0) j0_goal = angle;
  if (joint == 1) j1_goal = angle;
  if (joint == 2) j2_goal = angle;
  if (joint == 3) j3_goal = angle;
  if (joint == 4) j4_goal = angle;
  if (joint == 5) j5_goal = angle;
  return;
}

// This function plots all of the potentiometer readings
void PotentiometerTest()
{
  while(true)
  {
    digitalWrite(PIN_MUX_CTRL, LOW);
    delayMicroseconds(5);
    Serial.print("Joint0:");
    Serial.print(analogRead(PIN_MUX1));
    Serial.print(",Joint2:");
    Serial.print(analogRead(PIN_MUX2));
    Serial.print(",Joint4:");
    Serial.print(analogRead(PIN_MUX3));
    digitalWrite(PIN_MUX_CTRL, HIGH);
    delayMicroseconds(5);
    Serial.print(",Joint1:");
    Serial.print(analogRead(PIN_MUX1));
    Serial.print(",Joint3:");
    Serial.print(analogRead(PIN_MUX2));
    Serial.print(",Joint5:");
    Serial.println(analogRead(PIN_MUX3));
    delay(20);
  }
}

// This function converts from raw ADC values into Servo angle values
void ConvertArrays()
{
  Serial.println("Starting the Array conversion process");
  // Raw ADC values go between 0 and 4086, different range for each potentiometer in that range
  // We want to scale that range, up to 0 to 180 degrees
  for (int i = 0; i < max_samples; i++)
  {
    // Joint 0
    j0_pos[i] = map(j0_pos[i], j0_min, j0_max, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE);
    if (j0_pos[i] > 180)  j0_pos[i] = 180;
    if (j0_pos[i] < 0)    j0_pos[i] = 0;
    // Joint 1
    j1_pos[i] = map(j1_pos[i], j1_min, j1_max, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE);
    if (j1_pos[i] > 180)  j1_pos[i] = 180;
    if (j1_pos[i] < 0)    j1_pos[i] = 0;
    // Joint 2
    j2_pos[i] = map(j2_pos[i], j2_min, j2_max, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE);
    if (j2_pos[i] > 180)  j2_pos[i] = 180;
    if (j2_pos[i] < 0)    j2_pos[i] = 0;
    // Joint 3
    j3_pos[i] = map(j3_pos[i], j3_min, j3_max, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE);
    if (j3_pos[i] > 180)  j3_pos[i] = 180;
    if (j3_pos[i] < 0)    j3_pos[i] = 0;
    // Joint 4
    j4_pos[i] = map(j4_pos[i], j4_min, j4_max, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE);
    if (j4_pos[i] > 180)  j4_pos[i] = 180;
    if (j4_pos[i] < 0)    j4_pos[i] = 0;
    // Joint 5
    j5_pos[i] = map(j5_pos[i], j5_min, j5_max, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE);
    if (j5_pos[i] > 180)  j5_pos[i] = 180;
    if (j5_pos[i] < 0)    j5_pos[i] = 0;
  }
  Serial.println("Array conversion process complete");
  return;
}

// Function for controlling the Servo motors
void ServoMotor(int joint, int angle)
{
  // The joint parameter can range between 0 and 5
  if (joint < 0 || joint > 5)
  {
    Serial.println("ERROR - ServoMotor Function - This joint doesn't exist");
    Serial.print("Joint requested: ");
    Serial.println(joint);
    return;
  }
  // The Angle can range between 0 and 180 degrees
  if (angle < 0 || angle > 180)
  {
    Serial.println("ERROR - ServoMotor Function - This angle is out of range");
    Serial.print("Requested angle: ");
    Serial.println(angle);
    return;
  }
  // At this point, all of the parameters are clear

  // We calculate the pulse width based on the requested angle and motor parameters
  pulse_width = map(angle, SMALL_SERVO_MIN_ANGLE, SMALL_SERVO_MAX_ANGLE, SMALL_SERVO_MIN_PWM, SMALL_SERVO_MAX_PWM);
  // We calculate the duty cycle based on the pulse width and the frequency
  duty_cycle = 100 * pulse_width / period;
  // Sending out the PWM
  switch(joint)
  {
    case 0: 
    {
      PWM_Instance[joint]->setPWM(PIN_J0_SERVO, PWM_FREQUENCY, duty_cycle);
      return;
    }
    case 1: 
    {
      PWM_Instance[joint]->setPWM(PIN_J1_SERVO, PWM_FREQUENCY, duty_cycle);
      return;
    }
    case 2: 
    {
      PWM_Instance[joint]->setPWM(PIN_J2_SERVO, PWM_FREQUENCY, duty_cycle);
      return;
    }
    case 3: 
    {
      PWM_Instance[joint]->setPWM(PIN_J3_SERVO, PWM_FREQUENCY, duty_cycle);
      return;
    }
    case 4: 
    {
      PWM_Instance[joint]->setPWM(PIN_J4_SERVO, PWM_FREQUENCY, duty_cycle);
      return;
    }
    case 5: 
    {
      PWM_Instance[joint]->setPWM(PIN_J5_SERVO, PWM_FREQUENCY, duty_cycle);
      return;
    }
    default: Serial.println("ERROR - ServoMotor Function - Unknown Error in Switch Case statement");
  }
}

// Function for controlling all motors at once
void RobotMotors(int ang0, int ang1, int ang2, int ang3, int ang4, int ang5)
{
  // This is just a shortcut function for calling the ServoMotor function 6 times at once
  ServoMotor(0, ang0);
  ServoMotor(1, ang1);
  ServoMotor(2, ang2);
  ServoMotor(3, ang3);
  ServoMotor(4, ang4);
  ServoMotor(5, ang5);
  return;
}

// This function is used for quickly testing out all of the PWM Channels
void QuickPWMTest()
{
  // This function shouldn't be done with the power provided to the motors
  digitalWrite(PIN_RELAY, LOW);
  // The function uses blocking code, and should only be called in debugging cases and disabled afterwards
  // It switches the PWM from 500us to 1500us to 2500us
  while(true)
  {
    // Set all Motors to 0 Degrees
    ServoMotor(0, 0);
    ServoMotor(1, 0);
    ServoMotor(2, 0);
    ServoMotor(3, 0);
    ServoMotor(4, 0);
    ServoMotor(5, 0);
    delay(1000);
    // Set all Motors to 90 Degrees
    ServoMotor(0, 90);
    ServoMotor(1, 90);
    ServoMotor(2, 90);
    ServoMotor(3, 90);
    ServoMotor(4, 90);
    ServoMotor(5, 90);
    delay(1000);
    // Set all Motors to 0 Degrees
    ServoMotor(0, 180);
    ServoMotor(1, 180);
    ServoMotor(2, 180);
    ServoMotor(3, 180);
    ServoMotor(4, 180);
    ServoMotor(5, 180);
    delay(1000);
  }
}

// Function for checking if there is any Serial data
void CheckSerial()
{
  if (Serial.available() <= 0) return;
  serial_message = "";
  new_message = true;
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') serial_message = serial_message + c;
  }
  return;
}


// Function used for calibrating the robot
void Calibrate()
{
  // Calibration procedure includes getting all of the motor to their max positions, and record the potentiometer reading
  // This should be done in such a way to not put a lot of straing on the robot segments
  // The angles should go like this:
  // Step 1: J0 0 J1 0 J2 180 J3 180 J4 180 J5 0
  // Step 2: J0 180 J1 180 J2 0 J3 0 J4 0 J5 180

  // We need to first put the robot into a safe position and turn the Relay ON
  RobotMotors(90, 90, 90, 90, 90, 90);
  digitalWrite(PIN_RELAY, HIGH);
  // We also need to give some time to the robot to reach this position
  delay(3000);

  Serial.println("Starting Calibration Procedure");

  // Step 1
  // Step 1: J0 0 J1 0 J2 180 J3 180 J4 180 J5 0
  RobotMotors(0, 0, 180, 180, 180, 0);
  // We need to give adequate time for all of the motors to reach this angle
  delay(5000);
  // After that, we can record all of the potentiometer readings
  int p0, p1, p2, p3, p4, p5;
  int sample_size = 10;
  // We sample the potentiometers sample_size number of times
  for (int i = 0; i < sample_size; i++)
  {
    // We need to switch the MUX before reading alternating values
    digitalWrite(PIN_MUX_CTRL, LOW);
    // Give it a bit of time to switch
    delay(5);
    // In this state, we can read p0, p2, and p4
    p0 += analogRead(PIN_MUX1);
    p2 += analogRead(PIN_MUX2);
    p4 += analogRead(PIN_MUX3);
    // Now we can switch the MUX to the other position and read the other 3 values
    digitalWrite(PIN_MUX_CTRL, HIGH);
    // Give it a bit of time to switch
    delay(5);
    // In this state, we can read p1, p3, and p5
    p1 += analogRead(PIN_MUX1);
    p3 += analogRead(PIN_MUX2);
    p5 += analogRead(PIN_MUX3);
    // Give a bit of delay between readings
  }
  // In this Step 1, we've recorded j0_min, j1_min, j2_max, j3_max, j4_max, j5_min
  j0_min = p0 / sample_size;
  j1_min = p1 / sample_size;
  j2_max = p2 / sample_size;
  j3_max = p3 / sample_size;
  j4_max = p4 / sample_size;
  j5_min = p5 / sample_size;
  // Reseting all of the variables needed for Step 2
  p0 = 0;
  p1 = 0;
  p2 = 0;
  p3 = 0;
  p4 = 0;
  p5 = 0;
  
  // Step 2
  // Step 2: J0 180 J1 180 J2 0 J3 0 J4 0 J5 180
  RobotMotors(180, 180, 0, 0, 0, 180);
  // We need to give adequate time for all of the motors to reach this angle
  delay(5000);
  // We sample the potentiometers sample_size number of times
  for (int i = 0; i < sample_size; i++)
  {
    // We need to switch the MUX before reading alternating values
    digitalWrite(PIN_MUX_CTRL, LOW);
    // Give it a bit of time to switch
    delay(5);
    // In this state, we can read p0, p2, and p4
    p0 += analogRead(PIN_MUX1);
    p2 += analogRead(PIN_MUX2);
    p4 += analogRead(PIN_MUX3);
    // Now we can switch the MUX to the other position and read the other 3 values
    digitalWrite(PIN_MUX_CTRL, HIGH);
    // Give it a bit of time to switch
    delay(5);
    // In this state, we can read p1, p3, and p5
    p1 += analogRead(PIN_MUX1);
    p3 += analogRead(PIN_MUX2);
    p5 += analogRead(PIN_MUX3);
    // Give a bit of delay between readings
  }
  // In this Step 1, we've recorded j0_max, j1_max, j2_min, j3_min, j4_min, j5_max
  j0_max = p0 / sample_size;
  j1_max = p1 / sample_size;
  j2_min = p2 / sample_size;
  j3_min = p3 / sample_size;
  j4_min = p4 / sample_size;
  j5_max = p5 / sample_size;

  // We can now return the robot to the default upright position
  RobotMotors(90, 90, 90, 90, 90, 90);
  // We also need to give some time to the robot to reach this position
  delay(3000);
  // Now we turn OFF the power to the motors
  digitalWrite(PIN_RELAY, LOW);
  
  // With that the calibration procedure is complete
  Serial.println("Calibration procedure complete");
  
  return;
  
}

// Function for decoding the Serial message coming from the user
void DecodeSerialMessage(String msg)
{
  /*
   *  THIS IS THE LIST OF THE CURRENTLY AVAILABLE COMMANDS FOR THE ROBOT ARM
   *  C0  - STOP ALL          - Kills the relay power and sets the PWM on all motors to 0% duty cycle
   *  C1  - DISABLE MOTOR PWM - Disables all motor PWM-s, drops the duty cycle to 0%
   *  C2  - TURN RELAY ON     - This function turns on the RELAY
   *  C3  - SET MOTOR ANGLE   - Set an angle on a specific motor
   *  C4  - CALIBRATION       - Let the robot run through a calibration procedure
   *  C5  - START RECORDING   - Start recording potentiometer angles
   *  C6  - STOP RECORDING    - Stop the current recording
   *  C7  - SET ALL JOINTS    - Send angle joint command to all joints
   *  C8  - PLAYBACK START    - Playback the last recording from 0
   *  C9  - PLAYBACK STOP     - Stop the playback     
   *  C10 - PLAYBACK RESET    - Start the playback again from 0
   *  C11 - PLAYBACK SPEED    - Configure the playback speed
   */

   // Command 0 [C0] - Stop All Command
   if (msg.startsWith("C0") == true)
   {
    Serial.println("Received command: C0 - Turning Relay OFF - Killing all motors");
    // Turning the Felay OFF
    digitalWrite(PIN_RELAY, LOW);
    // Killing the PWM for all motors
    PWM_Instance[0]->setPWM(PIN_J0_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[1]->setPWM(PIN_J1_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[2]->setPWM(PIN_J2_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[3]->setPWM(PIN_J3_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[4]->setPWM(PIN_J4_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[5]->setPWM(PIN_J5_SERVO, PWM_FREQUENCY, 0.00);
    // With this, the call of this function is complete
    return;
   }

   // Command 1 [C1] - Disable Motor PWM
   if (msg.startsWith("C1") == true)
   {
    Serial.println("Received command: C1 - Disable all motor PWM");
    // Killing the PWM for all motors
    PWM_Instance[0]->setPWM(PIN_J0_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[1]->setPWM(PIN_J1_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[2]->setPWM(PIN_J2_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[3]->setPWM(PIN_J3_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[4]->setPWM(PIN_J4_SERVO, PWM_FREQUENCY, 0.00);
    PWM_Instance[5]->setPWM(PIN_J5_SERVO, PWM_FREQUENCY, 0.00);
    // With this, the call of this function is complete
    return;
   }

   // Command 2 [C2] - Turn Relay ON
   if (msg.startsWith("C2") == true)
   {
    Serial.println("Received command: C2 - Turning Relay ON");
    // Turning the Felay ON
    digitalWrite(PIN_RELAY, HIGH);
    // With this, the call of this function is complete
    return;
   }

   // Command 3 [C3] - Set angle on a specific motor
   // Command format: C3 J X A Y - Where X is the Joint Number and Y is the Angle value
   if (msg.startsWith("C3") == true)
   {
    Serial.println("Received command: C3 - Set Motor Angle");
    // We need to decode this function call
    int mot_pos, mot, ang_pos, ang;
    // Extracting the location of the designators within the command
    mot_pos = msg.indexOf("J");
    ang_pos = msg.indexOf("A");
    // Extracting the actual number values from the command
    mot = msg.substring(mot_pos + 1, ang_pos).toInt();
    ang = msg.substring(ang_pos + 1).toInt();
    Serial.print("Joint number: ");
    Serial.println(mot);
    Serial.print("Angle: ");
    Serial.println(ang);
    if(RAMP_MODE == true)
    {
      // If in ramp mode, update the parameters
      UpdateMotor(mot, ang);
    }
    else
    {
      // Using these values we call the ServoMotor function
      ServoMotor(mot, ang);
    }
    // With this, the call of this function is complete
    return;
   }
   
   // Command 4 [C4] - Calibrate Robot
   if (msg.startsWith("C4") == true)
   {
    Serial.println("Received command: C4 - Calibrate Robot");
    // Switch the flag to false before starting the calibration procedure
    calibration_done = false;
    // Start the calibration procedure
    Calibrate();
    // Switch the flag to true after the calibration has been complete
    calibration_done = true;
    // Printing out the recorded parameters
    // With this, the call of this function is complete
    return;
   }

   // Command 5 [C5] - Start Recording
   if (msg.startsWith("C5") == true)
   {
    Serial.println("Received command: C5 - Start Recording");
    // We can start recording only if the robot has previously been calibrated
    if (calibration_done == false)
    {
      Serial.println("Can't start recording because the robot hasn't been calibrated yet");
      return;
    }
    if (record_flag == true)
    {
      Serial.println("Recording has already been started");
      return;
    }
    if (playback_flag == true)
    {
      Serial.println("There is an active playback, stop it first before trying to record");
      return;
    }
    // We need to clear out all of the variables for recording and reset the sample_num
    ClearArr();
    sample_num  = 0;
    // Switch the recording flag to true
    Serial.println("Changing the record flag to true");
    record_flag = true;
    // With this, the call of this function is complete
    return;
   }

   // Command 6 [C6] - Stop Recording
   if (msg.startsWith("C6") == true)
   {
    Serial.println("Received command: C6 - Stop Recording");
    // Checking if there is a recording to stop
    if (record_flag == false)
    {
      Serial.println("The recording hasn't been started yet, nothing to stop");
      return;
    }
    Serial.println("Converting the arrays into angles");
    // Convert the default arrays into servo angles
    ConvertArrays();
    // Switch the recording flag to false
    record_flag = false;
    // Switch the recorded_data flag to true
    recorded_data = true;
    // With this, the call of this function is complete
    return;
   }

   // Command 7 [C7] - Send angle to all Joints
   // Command format: C7 A0 X0 A1 X1 A2 X2 A3 X3 A4 X4 A5 X5
   if (msg.startsWith("C7") == true)
   {
    Serial.println("Received command: C7 - Send angle to all joints");
    // We need to extract the data from the message
    int a0_pos, a1_pos, a2_pos, a3_pos, a4_pos, a5_pos, a0, a1, a2, a3, a4, a5; 
    // Positions of all markers in the String
    a0_pos = msg.indexOf("A0");
    a1_pos = msg.indexOf("A1");
    a2_pos = msg.indexOf("A2");
    a3_pos = msg.indexOf("A3");
    a4_pos = msg.indexOf("A4");
    a5_pos = msg.indexOf("A5");
    // Extracting the value
    a0 = msg.substring(a0_pos + 2, a1_pos).toInt();
    a1 = msg.substring(a1_pos + 2, a2_pos).toInt();
    a2 = msg.substring(a2_pos + 2, a3_pos).toInt();
    a3 = msg.substring(a3_pos + 2, a4_pos).toInt();
    a4 = msg.substring(a4_pos + 2, a5_pos).toInt();
    a5 = msg.substring(a5_pos + 2).toInt();
    // Printing out what we parsed from the command
    Serial.println("Angles extracted from the message: ");
    Serial.print("A0: ");
    Serial.println(a0);
    Serial.print("A1: ");
    Serial.println(a1);
    Serial.print("A2: ");
    Serial.println(a2);
    Serial.print("A3: ");
    Serial.println(a3);
    Serial.print("A4: ");
    Serial.println(a4);
    Serial.print("A5: ");
    Serial.println(a5);
    // Send all of those angles to the Robot
    if (RAMP_MODE == true)
    {
      UpdateMotors(a0, a1, a2, a3, a4, a5);
    }
    else
    {
      RobotMotors(a0, a1, a2, a3, a4, a5);
    }
    // With this, the call of this function is complete
    return;
   }

   // Command 8 [C8] - Start Playback
   if (msg.startsWith("C8") == true)
   {
    Serial.println("Received command: C8 - Start Playback");
    // Checking if we have a valid robot calibration
    if (calibration_done == false)
    {
      Serial.println("Robot hasn't been calibrated - unable to playback");
      return;
    }
    // Checking if there is any recorded data to playback
    if (recorded_data == false)
    {
      Serial.println("There isn't any recorded data to playback");
      return;
    }
    // Checking if we're in an active recording
    if (record_flag == true)
    {
      Serial.println("System currently in recording mode, can't start playback during recording");
      return;
    }
    // After all of those checks, we can switch the playback flag to true
    playback_flag = true;
    // With this, the call of this function is complete
    return;
   }

   // Command 9 [C9] - Stop Playback
   if (msg.startsWith("C9") == true)
   {
    Serial.println("Received command: C9 - Stop Playback");
    // Checking if there is a playback to stop
    if (playback_flag == false)
    {
      Serial.println("The playback hasn't been started yet, nothing to stop");
      return;
    }
    // Switch the recording flag to false
    playback_flag = false;
    // With this, the call of this function is complete
    return;
   }

   

   // If we're here, that command is unknown
   Serial.println("ERROR - DecodeSerialMessage Function - Unknown Command");
}

// TIMER FUNCTIONS

// Function for handling the sampling timer
bool TimerHandlerSampling(struct repeating_timer *t)
{
  (void) t;
  
  if (first == true && record_flag == true)
  {
    Serial.println("Starting to record");
    first = false;
  }
  
  if (record_flag == true)
  {
    if (sample_num > max_samples)
    {
      Serial.println("Recording finished due to reacing sample limit");
      Serial.println("Converting Arrays");
      // Convert the default arrays into servo angles
      ConvertArrays();
      record_flag = false;
      recorded_data = true;
      first = true;
      return true;
    }
    // Set the multiplexer to position 0
    digitalWrite(PIN_MUX_CTRL, LOW);
    delayMicroseconds(5);
    // We can now read position from these joints: J0 J2 J4
    j0_pos[sample_num] = analogRead(PIN_MUX1);
    j2_pos[sample_num] = analogRead(PIN_MUX2);
    j4_pos[sample_num] = analogRead(PIN_MUX3);
    // Set the multiplexer to position 1
    digitalWrite(PIN_MUX_CTRL, HIGH);
    delayMicroseconds(5);
    // We can now read position from these joints: J1 J3 J5
    j1_pos[sample_num] = analogRead(PIN_MUX1);
    j3_pos[sample_num] = analogRead(PIN_MUX2);
    j5_pos[sample_num] = analogRead(PIN_MUX3);
    // Increase the sample_num counter
    sample_num++;
  }

  return true;
}

// Function for handling the playback timer
bool TimerHandlerPlayback(struct repeating_timer *t)
{
  (void) t;
  if (first == true && playback_flag == true)
  {
    // We need to put the robot into the starting position first
    RobotMotors(90, 90, 90, 90, 90, 90);
    // Turning the power ON
    digitalWrite(PIN_RELAY, HIGH);
    Serial.println("Starting playback");
    first = false;
  }
  if (playback_flag == true)
  {
    playback_trigger++;
    if (playback_trigger >= (1 / playback_speed) * 10)
    {
      playback_trigger = 0;
      RobotMotors(j0_pos[playback_counter], j1_pos[playback_counter], j2_pos[playback_counter], j3_pos[playback_counter], j4_pos[playback_counter], j5_pos[playback_counter]);
      playback_counter++;
      if(playback_counter >= max_samples) playback_counter = 0;
    }
  }

  return true;
}

// Function for handling the ramp timer
bool TimerHandlerRamp(struct repeating_timer *t)
{
  (void) t;

  if (RAMP_MODE == true)
  {
    // Joint 0
    if (j0_goal != J0)
    {
      // This means that there was a request to change the angle on this joint
      if (j0_goal > J0)
      {
        J0++;
      }
      else
      {
        J0--;
      }
    }

    // Joint 1
    if (j1_goal != J1)
    {
      // This means that there was a request to change the angle on this joint
      if (j1_goal > J1)
      {
        J1++;
      }
      else
      {
        J1--;
      }
    }

    // Joint 2
    if (j2_goal != J2)
    {
      // This means that there was a request to change the angle on this joint
      if (j2_goal > J2)
      {
        J2++;
      }
      else
      {
        J2--;
      }
    }

    // Joint 3
    if (j3_goal != J3)
    {
      // This means that there was a request to change the angle on this joint
      if (j3_goal > J3)
      {
        J3++;
      }
      else
      {
        J3--;
      }
    }

    // Joint 4
    if (j4_goal != J4)
    {
      // This means that there was a request to change the angle on this joint
      if (j4_goal > J4)
      {
        J4++;
      }
      else
      {
        J4--;
      }
    }

    // Joint 5
    if (j5_goal != J5)
    {
      // This means that there was a request to change the angle on this joint
      if (j5_goal > J5)
      {
        J5++;
      }
      else
      {
        J5--;
      }
    }   

    RobotMotors(J0, J1, J2, J3, J4, J5);  
  }

  return true;
}

void setup() {
  
  // Starting the Serial
  Serial.begin(115200);
  //Serial.println("Staring the program");

  // Setting up the pins
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_MUX_CTRL,   OUTPUT);
  pinMode(PIN_J0_SERVO,   OUTPUT);
  pinMode(PIN_J1_SERVO,   OUTPUT);
  pinMode(PIN_J2_SERVO,   OUTPUT);
  pinMode(PIN_J3_SERVO,   OUTPUT);
  pinMode(PIN_J4_SERVO,   OUTPUT);
  pinMode(PIN_J5_SERVO,   OUTPUT);
  pinMode(PIN_MUX1,       INPUT);
  pinMode(PIN_MUX2,       INPUT);
  pinMode(PIN_MUX3,       INPUT);

  // Disabling the power to the servo motors
  digitalWrite(PIN_RELAY, LOW);

  // ADC Resolution
  analogReadResolution(ADC_RES);

  // Clearing all of the Arrays - This sets them to the default position of 90
  ClearArr();

  // Setting up PWM
  // Joint 0
  PWM_Instance[0] = new RP2040_PWM(PIN_J0_SERVO, PWM_FREQUENCY, 0.00f);
  PWM_Instance[0]->setPWM();
  // Joint 1
  PWM_Instance[1] = new RP2040_PWM(PIN_J1_SERVO, PWM_FREQUENCY, 0.00f);
  PWM_Instance[1]->setPWM();
  // Joint 2
  PWM_Instance[2] = new RP2040_PWM(PIN_J2_SERVO, PWM_FREQUENCY, 0.00f);
  PWM_Instance[2]->setPWM();
  // Joint 3
  PWM_Instance[3] = new RP2040_PWM(PIN_J3_SERVO, PWM_FREQUENCY, 0.00f);
  PWM_Instance[3]->setPWM();
  // Joint 4
  PWM_Instance[4] = new RP2040_PWM(PIN_J4_SERVO, PWM_FREQUENCY, 0.00f);
  PWM_Instance[4]->setPWM();
  // Joint 5
  PWM_Instance[5] = new RP2040_PWM(PIN_J5_SERVO, PWM_FREQUENCY, 0.00f);
  PWM_Instance[5]->setPWM();

  // Checking to see if the Quick PWM Test Function is enabled
  // IF YOU WANT YOUR CODE TO RUN DISABLE THIS FUNCTION IN THE PARAMETERS BY SWITCHING PWM_QUICK TEST TO FALSE
  if (PWM_QUICK_TEST == true) QuickPWMTest();

  // Checking to see if the Potentiometer Test Function is enabled
  // IF YOU WANT YOUR CODE TO RUN DISABLE THIS FUNCTION IN THE PARAMETERS BY SWITCHING POTENTIOMETER_TEST TO FALSE
  if (POTENTIOMETER_TEST == true) PotentiometerTest();

  // Starting up the Timers
  // Interval in microsecs
  if (Timer_Sampling.attachInterruptInterval(sample_period, TimerHandlerSampling))
  {
    Serial.print(F("Starting Sampling Timer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set Sampling Timer. Select another freq. or timer"));

  // Interval in microsecs
  if (Timer_Playback.attachInterruptInterval(playback_period, TimerHandlerPlayback))
  {
    Serial.print(F("Starting Playback Timer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set Playback Timer. Select another freq. or timer"));

  // Interval in microsecs
  if (Timer_Ramp.attachInterruptInterval(10000, TimerHandlerRamp))
  {
    Serial.print(F("Starting Ramp Timer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set Ramp Timer. Select another freq. or timer"));

  if (RAMP_MODE == true)
  {
    Serial.println("Ramp mode is enabled");
    J0      = 90;
    J1      = 90;
    J2      = 90;
    J3      = 90;
    J4      = 90;
    J5      = 90;
    j0_goal = 90;
    j1_goal = 90;
    j2_goal = 90;
    j3_goal = 90;
    j4_goal = 90;
    j5_goal = 90;
  }

  // Turn the Relay ON by default in Showcase mode
  if (SHOWCASE_MODE == true) digitalWrite(PIN_RELAY, HIGH);
  
}

void loop() {

  if (SHOWCASE_MODE == true)
  {
    if (RAMP_MODE == true)
    {
      UpdateMotors(90, 85, 90, 90, 90, 90);
      delay(3000);
      UpdateMotors(0, 85, 90, 90, 90, 90);
      delay(3000);
      UpdateMotors(180, 85, 90, 90, 90, 90);
      delay(3000);
      UpdateMotors(0, 85, 90, 0, 90, 90);
      delay(3000);
      UpdateMotors(0, 85, 90, 180, 90, 90);
      delay(3000);
    }
  }
  
  if (new_message == false)
  {
    CheckSerial();
  }
  else
  {
    new_message = false;
    Serial.println(serial_message);
    DecodeSerialMessage(serial_message);
  }
}
