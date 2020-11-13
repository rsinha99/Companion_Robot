#include "MotorControl.h"
#include "Ultrasound.h"

/* TODO */
// Set up a Queue for receiving serial commands


// Ping sensor pins
const int center_echo_pin = 8;
const int center_trig_pin = 9;
const int left_echo_pin   = A1;
const int left_trig_pin   = A0;
const int right_echo_pin  = 12;
const int right_trig_pin  = 11;

// Motor pins
const int LeftMotorA_pin    = A3;
const int RightMotorA_pin   = 6;
const int LeftMotorB_pin    = A4;
const int RightMotorB_pin   = 7;
const int LeftMotorPWM_pin  = 5;
const int RightMotorPWM_pin = 10;
const int LeftEncoder_pin = 2;
const int RightEncoder_pin = 3;

// OOP Initializations
MotorControl leftMotor  (LeftMotorA_pin,  LeftMotorB_pin,  LeftMotorPWM_pin);
MotorControl rightMotor (RightMotorA_pin, RightMotorB_pin, RightMotorPWM_pin);
Ultrasound leftUltrasound   (left_echo_pin, left_trig_pin);
Ultrasound centerUltrasound (center_echo_pin, center_trig_pin);
Ultrasound rightUltrasound  (right_echo_pin, right_trig_pin);

//constants
const byte default_PWM = 40;
byte right_PWM = default_PWM - 5; //adjustment for initial PWM; proportional control makes this value irrelevant later on
byte left_PWM = default_PWM;
const int STOP_DISTANCE_CENTER = 25; // cm
const int STOP_DISTANCE_SIDE   = 25; // cm

// enum for Direction
enum Directions {forward, backward, left, right, halt};
// 0 - forward, 1 - backwards, 2 - left, 3 - right, 4 - halt

enum States {calibration, obstacle, distance, following, waiting};
// calibration - calibrate motor power, obstacle - basic obstacle avoidance
// distance - specify distance to travel and turning angles
// following - no specified distance; turns while moving forward at different turning speeds
// waiting - robot is halted; awaiting commands

// State Serial Code Variables
const byte distance_code = B00000001;
const byte following_code = B00000010;
const byte calibration_code = B00000011;
const byte halt_code = B00000000;

//globals for motor control
Directions currDir = forward;
Directions prevDir = halt;
States state = calibration;
byte commandValue = 0;

//globals for crash avoidance
boolean dangerCenter = false;
boolean dangerLeft   = false;
boolean dangerRight  = false;
boolean dangerDetected = false;
long leftDistance = 0, centerDistance = 0, rightDistance = 0;

// Variables for the encoder
int count = 0;
int calibration_count = 0;
int error = 0;
int kp = 80;
int rightTicks = 0;
int leftTicks = 0;
int distanceTicks = 0;

void setup() {
  Serial.begin(9600); // Set baud-rate
  while(!Serial) {
    ; // wait for serial to connect
  }
  attachInterrupt(digitalPinToInterrupt(RightEncoder_pin), tickRight, CHANGE); //unfortunately, we cannot include the interrupt in the MotorControl class
  attachInterrupt(digitalPinToInterrupt(LeftEncoder_pin), tickLeft, CHANGE);
  go_stop(); // Guarantee that both motors are not moving at start
  set_speed();
  delay(500);
}

void loop() {
  byte = Serial.read(1);
  Serial.print("Received: " + byte);
}

// Commented out so that I can test the serial
//void loop() {
//  count++;
//
//  processCommand(); // Check Serial for command and process it
//
//  // Check the state of the robot: Obstacle, Distance, Angle
//  switch (state) {
//    case calibration:
//      // Use Proportional Feedback and Encoder readings to match the right motor's speed to the left motor.
//      // In this case, left motor is the master and right motor is the slave.
//      Serial.print("left ticks: "); Serial.println(leftTicks);
//      Serial.print("right ticks: "); Serial.println(rightTicks);
//      if (count >= 15) {
//        error = leftTicks - rightTicks;
//        right_PWM += error / kp;
//        set_speed();
//        leftTicks = 0;
//        rightTicks = 0;
//        count = 0;
//        error = 0;
//        calibration_count++;
//        Serial.println("Calibrating Motors");
//      }
//      if (calibration_count >= 10) {
//        state = waiting;
//        calibration_count = 0;
//      }
//      break;
//    case distance:
//      centerDistance = centerUltrasound.getDistance();
//      if (centerDistance <= STOP_DISTANCE_CENTER) {
//        updateDir(halt);
//      } else {
//        if (distanceTicks >= 15000) {
//          updateDir(halt);
//          delay(1000);
//          distanceTicks = 0;
//        } else {
//          updateDir(forward);
//        }
//      }
//      if (distanceTicks >= 3400) {
//        updateDir(halt);
//        delay(5000);
//        distanceTicks = 0;
//        state = distance;
//      } else {
//        updateDir(left);
//      }
//      break;
//    default:
//      updateDir(halt);
//  }
//
//}

// Process command coming from the Serial Port. The behavior changes depending
// on what state the robot is in. The serial can also be used to change states.
// Commands are a single byte. Command codes are listed below:
// States:
//    Distance    0000 0001
//    Following   0000 0010
//    Calibration 0000 0011
//    Waiting     0000 0000
// Directions:
//    Forward     11XX XXXX
//    Backward    001X XXXX
//    Left        01XX XXXX
//    Right       10XX XXXX
// When in Distance mode, the X'd values represent distances and in-place turn angles.
// When in following mode, the X'd values represent turning speeds (difference in
// power betewen motors). This only applies for left and right directions.
void processCommand() {
  if (Serial.available() > 0) {
    byte commandByte = Serial.read();
    bool isStateChange = commandByte >> 4 == 0;
    if (isStateChange) {
      switch (commandByte) {
        case 0:
          state = waiting;
          break;
        case 1:
          state = distance;
          break;
        case 2:
          state = following;
          break;
        case 3:
          state = calibration;
          break;
      }
    } else {
      switch (commandByte >> 6) {
        case 0: //backwards
          commandValue = commandByte & B00011111;
          updateDir(backward);
          break;
        case 1: //left
          commandValue = commandByte & B00111111;
          updateDir(left);
          break;
        case 2: //right
          commandValue = commandByte & B00111111;
          updateDir(right);
          break;
        case 3: //forward
          commandValue = commandByte & B00111111;
          updateDir(forward);
          break;
        
      }
    }
  }
}

// Takes a centimeter input and outputs a distance tick count
// About 80 encoder ticks per centimeter
int go_distance(int distance) {
  go_forward();
  distanceTicks = 0;
  updateDir(forward);

}

// Call this function to tell robot to recalibrate motor powers to drive straight
void calibrate_motors() {
  calibration_count = 0;
  state = calibration;
}


//check sensors for new obstacles
void updatePingData(void *pvParameters) {
  while (1) {
    //Serial.println("Reading Ping Data");
    // Get distances
    leftDistance   = leftUltrasound.getDistance();
    centerDistance = centerUltrasound.getDistance();
    rightDistance  = rightUltrasound.getDistance();
    //    Serial.print("left cm: "); Serial.print(leftDistance);
    //    Serial.println();
    //    Serial.print("center cm: "); Serial.print(centerDistance);
    //    Serial.println();
    //    Serial.print("right cm: "); Serial.print(rightDistance);
    //    Serial.println();

    // Update danger booleans
    dangerLeft   = leftDistance   <= STOP_DISTANCE_SIDE;
    dangerCenter = centerDistance <= STOP_DISTANCE_CENTER;
    dangerRight  = rightDistance  <= STOP_DISTANCE_SIDE;
    dangerDetected = dangerCenter || dangerRight || dangerLeft;

  }
}

void updateDir(Directions newDir) {
  prevDir = currDir;
  currDir = newDir;
  if (state == distance) {
    respondToCurrDir();
  }
  
}

void respondToCurrDir() {
  // Only need to act on the currDir value if it's different from the prevDir
  if (currDir != prevDir) {
    if (currDir == forward)
      go_forward();
    else if (currDir == left)
      go_left();
    else if (currDir == right)
      go_right();
    else if (currDir == backward)
      go_backward();
    else if (currDir == halt)
      go_stop();
    else
      Serial.println("Error in respondToCurrDir() - currDir not found");
  }
}

// We use the left wheel to measure distance
void tickLeft() {
  leftTicks++;
  distanceTicks++;
}

void tickRight() {
  rightTicks++;
}

void set_speed() {
  leftMotor.setPWM(left_PWM);
  rightMotor.setPWM(right_PWM);
}

void go_stop() {
  leftMotor.halt();
  rightMotor.halt();
}

void go_forward() {
  set_speed();
  leftMotor.forward();
  rightMotor.backward();
}

void go_backward() {
  set_speed();
  leftMotor.backward();
  rightMotor.forward();
}

void go_left() {
  set_speed();
  leftMotor.backward();
  rightMotor.backward();
}

void go_right() {
  set_speed();
  leftMotor.forward();
  rightMotor.forward();
}
