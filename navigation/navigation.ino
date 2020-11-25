#include "MotorControl.h"
#include "Ultrasound.h"
#include <cppQueue.h>

/* TODO
    To find things I still need to do, use ctrl-F to find all instances of "TODO" in the code
*/

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

enum Modes {calibration, obstacle, distance, following, waiting};
// calibration - calibrate motor power, obstacle - basic obstacle avoidance
// distance - specify distance to travel and turning angles
// following - no specified distance; turns while moving forward at different turning speeds
// waiting - robot is halted; awaiting commands

//globals for motor control
Directions currDir = forward;
Directions prevDir = halt;
Modes mode = waiting;
boolean isExecuting = false;

long leftDistance = 0, centerDistance = 0, rightDistance = 0;

// Variables for the encoder
int calibration_count = 0;
int error = 0;
int kp = 80;
int rightTicks = 0;
int leftTicks = 0;
int distanceTicks = 0;

// Variables for Serial Communication
int q_size = 20;
Queue q(sizeof(byte), q_size, FIFO);  // Queue to store commands from Nvidia
byte command;

void setup() {
  Serial.begin(9600); // Set baud-rate
  while (!Serial) {
    ; // wait for serial to connect
  }
  go_stop(); // Guarantee that both motors are not moving at start
  set_speed(right_PWM, left_PWM);
  delay(500);
}

// TODO: What prevents the last instruction from constantly repeating if no new command is received?
void loop() {
  acceptCommand();
  switch (mode) {
    case calibration:
      calibrate_motors();
      break;
    case distance:
      if (!q.isEmpty()) {
        q.pop(&command);  //store popped byte into variable value
        isExecuting = true;
        //Serial.write(command);
      } else {
        isExecuting = false;
      }
      break;
    case following:
      break;
    case waiting: // just here to make it clear that I did not forget about this mode. It just does nothing by default.
      isExecuting = false;
      break;
  }
  if (isExecuting) {
    processCommand(command);
    isExecuting = false;
  }
}

// Accept Command. Read serial; add command to queue if in Distance Mode and the instructions is not a System Instruction
void acceptCommand() {
  while (Serial.available() > 0) {
    command = Serial.read();
    bool isSystemCommand = command >> 4 == 1;
    if (isSystemCommand) {
      processCommand(command);
    } else if (mode == distance) {
      q.push(&command);
    } else {
      isExecuting = true; // command does not go to Queue; processes immediately
      break;
    }
  }
}

// Process command. This can be either from the Serial Port or the Command Queue.
// The behavior changes depending on what mode the robot is in. The serial can
// also be used to change modes. Commands are a single byte composed of
// Instruction Bits and Value Bits.
//
// Command codes are listed below:x
// Modes (0000):
//    Distance      0000 0001
//    Following     0000 0010
//    Calibration   0000 0011
//    Waiting       0000 0000
// System (0001): (These are not placed in the Queue)
//    E-Halt Robot  0001 0000 (and Clear Queue)
//    Clear Queue   0001 0001
// Directions:
//    Backward    001X XXXX
//    Left        01XX XXXX
//    Right       10XX XXXX
//    Forward     11XX XXXX
// When in Distance mode, the X'd values represent distances and in-place turn angles.
// When in following mode, the X'd values represent turning speeds (difference in
// power betewen motors). This only applies for left and right directions.
//
// Note: To change modes immediately, make sure to clear the Queue first
void processCommand(byte command) {
  int commandValue;
  bool isModeChange = command >> 4 == 0;
  bool isSystemCommand = command >> 4 == 1;
  if (isModeChange) {
    switch (command & B00001111) {
      case 0: // The robot only enters this mode after calibration
        mode = waiting;
        break;
      case 1:
        mode = distance;
        break;
      case 2:
        mode = following;
        q.clean();
        break;
      case 3:
        mode = calibration;
        break;
    }
  } else if (isSystemCommand) {
    switch (command & B00001111) {
      case 0: // Halt the robot immediately
        q.clean(); // clear Queue so that no other commands run
        isExecuting = false;
        updateDir(halt);
        respondToCurrDir();
        break;
      case 1: // Clear Queue (not sure why we would do this without halting, though)
        q.clean();
        isExecuting = false;
        break;
    }
  } else {
    // TODO Modify these according to the design in the Google Slide
    // Following Mode and Distance Mode make these do different things
    // Maybe have a go_distance(command) function and a follow(command) function
    switch (command >> 6) {
      case 0: //backwards
        commandValue = command & B00011111;

        break;
      case 1: //left
        commandValue = command & B00111111;

        break;
      case 2: //right
        commandValue = command & B00111111;

        break;
      case 3: //forward
        commandValue = command & B00111111;

        break;

    }
  }
}

// TODO: Complete this function
// TODO: Find the number of ticks per the angle of rotation
// Takes a centimeter input and outputs a distance tick count
// About 80 encoder ticks per centimeter
int go_distance(int distance) {
  go_forward();
  distanceTicks = 0;
  updateDir(forward);

}

// Call this function to tell robot to recalibrate motor powers to drive straight
// Modifies the right_PWM value
//
// TODO: Send back a Serial message if calibration succeeds/fails (if obstacle gets in way)
void calibrate_motors() {
  boolean halted = false;

  updateDir(halt); // stop the robot
  respondToCurrDir();
  delay(500);
  // Turn on the interrupt to detect encoder ticks
  attachInterrupt(digitalPinToInterrupt(RightEncoder_pin), tickRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftEncoder_pin), tickLeft, CHANGE);


  updateDir(forward); // robot starts moving
  respondToCurrDir();
  delay(200); // Give robot chance to get up to speed

  calibration_count = 0;
  leftTicks = 0;
  rightTicks = 0;

  // Use Proportional Feedback and Encoder readings to match the right motor's speed to the left motor.
  // In this case, left motor is the master and right motor is the slave.
  while (1) {
    centerDistance = centerUltrasound.getDistance();
    if (centerDistance <= STOP_DISTANCE_CENTER) {
      updateDir(halt);
      respondToCurrDir();
      // turn off the interrupts, so that they don't interfere with the Serial
      detachInterrupt(digitalPinToInterrupt(RightEncoder_pin));
      detachInterrupt(digitalPinToInterrupt(LeftEncoder_pin));
      //TODO: Send Serial Message back to Nvidia stating a halt
      Serial.write(B00000001);
      break;
    }

    if (calibration_count >= 15) {
      error = leftTicks - rightTicks;
      int adjustment = error / kp;
      right_PWM += adjustment;
      set_speed(right_PWM, left_PWM);
      delay(100);

      leftTicks = 0;
      rightTicks = 0;
      calibration_count = 0;
      error = 0;
      if (adjustment >= -1 && adjustment <= 1) {
        break;
      }
    }
    calibration_count++;
  }

  updateDir(halt); // stop the robot!
  respondToCurrDir();
  mode = waiting;
  // turn off the interrupts, so that they don't interfere with the Serial
  detachInterrupt(digitalPinToInterrupt(RightEncoder_pin));
  detachInterrupt(digitalPinToInterrupt(LeftEncoder_pin));
  // TODO: Send a Serial Message to Nvidia stating successful calibration
  Serial.write(B00000001);

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

  }
}

void updateDir(Directions newDir) {
  prevDir = currDir;
  currDir = newDir;
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

// Sets motor PWM values
void set_speed(byte new_right_PWM, byte new_left_PWM) {
  leftMotor.setPWM(new_left_PWM);
  rightMotor.setPWM(new_right_PWM);
}

void go_stop() {
  leftMotor.halt();
  rightMotor.halt();
}

void go_forward() {
  leftMotor.forward();
  rightMotor.backward();
}

void go_backward() {
  leftMotor.backward();
  rightMotor.forward();
}

void go_left() {
  leftMotor.backward();
  rightMotor.backward();
}

void go_right() {
  leftMotor.forward();
  rightMotor.forward();
}
