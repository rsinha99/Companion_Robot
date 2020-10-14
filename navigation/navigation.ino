#include "MotorControl.h"
#include "Ultrasound.h"

//constants
const byte default_PWM = 40;
byte right_PWM = default_PWM - 5; //adjustment for initial PWM; proportional control makes this value irrelevant later on
byte left_PWM = default_PWM;
const int STOP_DISTANCE_CENTER = 25; // cm
const int STOP_DISTANCE_SIDE   = 20; // cm

// enum for Direction
enum Directions {forward, backward, left, right, halt};
// 0 - forward, 1 - backwards, 2 - left, 3 - right, 4 - halt

// enum for type of following/avoidance
enum Following {person, roam, command};
// 0 - person, 1 - roam

enum States {obstacle, distance, turning};


//globals for motor control
Directions currDir = forward;
Directions prevDir = halt;
States state = distance;
Following followType = roam; // Default to roam for fun

//globals for crash avoidance
boolean dangerCenter = false;
boolean dangerLeft   = false;
boolean dangerRight  = false;
boolean dangerDetected = false;
long leftDistance = 0, centerDistance = 0, rightDistance = 0;
boolean lastCommandFromSerial = false;

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

// Variables for the encoder
int count = 0;
int error = 0;
int kp = 80;
int rightTicks = 0;
int leftTicks = 0;
int distanceTicks = 0;

void setup() {
  Serial.begin(9600); // Set baud-rate
  attachInterrupt(digitalPinToInterrupt(RightEncoder_pin), tickRight, CHANGE); //unfortunately, we cannot include the interrupt in the MotorControl class
  attachInterrupt(digitalPinToInterrupt(LeftEncoder_pin), tickLeft, CHANGE);
  go_stop(); // Guarantee that both motors are not moving at start
  set_speed();
  delay(500);
  go_forward();
}

void loop() {
  count++;

  // Use Proportional Feedback and Encoder readings to match the right motor's speed to the left motor.
  // In this case, left motor is the master and right motor is the slave.
  Serial.print("left ticks: "); Serial.println(leftTicks);
  Serial.print("right ticks: "); Serial.println(rightTicks);
  if (count >= 20) {
    error = leftTicks - rightTicks;
    right_PWM += error / kp;
    set_speed();
    leftTicks = 0;
    rightTicks = 0;
    count = 0;
    error = 0;
    Serial.println("Hello");
  }

  // Check the state of the robot: Obstacle, Distance, Angle
  switch (state) {
    case distance:
      centerDistance = centerUltrasound.getDistance();
      if (centerDistance <= STOP_DISTANCE_CENTER) {
        updateDir(halt);
      } else {
        if (distanceTicks >= 15000) {
          updateDir(halt);
          delay(1000);
          distanceTicks = 0;
          state = turning;
        } else {
          updateDir(forward);
        }
      }
      break;
    case turning:
      if (distanceTicks >= 3400) {
        updateDir(halt);
        delay(5000);
        distanceTicks = 0;
        state = distance;
      } else {
        updateDir(left);
      }
      break;
    default:
      updateDir(halt);
  }

}

// Takes a centimeter input and outputs a distance tick count
int go_distance(int distance) {

}

// Updates robot's movement state to the currDir
void respondToCurrDir() {
  // TODO - if lastCommand was not from serial then don't check the currDir != prevDir

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

//check the orders string for new commands
void updateOrders(void *pvParameters) {
  while (1) {
    if (Serial.available() > 0) {
      char incomingCharacter = Serial.read();
      Serial.print("I received: ");
      Serial.println(incomingCharacter);
      switch (incomingCharacter) {
        case '\n': // Disregard newlines
          break;
        case 'f':
          prevDir = currDir;
          currDir = forward;
          lastCommandFromSerial = true;
          break;
        case 'l':
          prevDir = currDir;
          currDir = left;
          lastCommandFromSerial = true;
          break;
        case 'r':
          prevDir = currDir;
          currDir = right;
          lastCommandFromSerial = true;
          break;
        case 'b':
          prevDir = currDir;
          currDir = backward;
          lastCommandFromSerial = true;
          break;
        case 'h':
          prevDir = currDir;
          currDir = halt;
          lastCommandFromSerial = true;
          break;
        case 'p':
          followType = person;
          break;
        case 'a':
          followType = roam;
          break;
        case 'c':
          followType = command;
          break;
        default:
          Serial.println("ERROR: Bad Message from Serial - character was not expected");
          break;
      }
    }
  }
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
  respondToCurrDir();
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
