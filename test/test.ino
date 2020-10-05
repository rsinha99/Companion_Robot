#include <Arduino_FreeRTOS.h>
#include "MotorControl.h"
#include "Ultrasound.h"

//constants
byte PWM = 60;
const int STOP_DISTANCE_CENTER = 15; // cm
const int STOP_DISTANCE_SIDE   = 10; // cm

// enum for Direction
enum Directions {forward, backward, left, right, halt};
// 0 - forward, 1 - backwards, 2 - left, 3 - right, 4 - halt

// enum for type of following/avoidance
enum Following {person, roam};
// 0 - person, 1 - roam


//globals for motor control
Directions currDir = halt;
Directions prevDir = halt;
Following followType = roam; // Default to roam for fun

//globals for crash avoidance
boolean dangerCenter = false;
boolean dangerLeft   = false;
boolean dangerRight  = false;
boolean dangerDetected = false;
long leftDistance = 0, centerDistance = 0, rightDistance = 0;
boolean lastCommandFromSerial = false;

//prototypes for RTOS tasks
void updateOrders  (void *pvParameters);
void updatePingData(void *pvParameters);
void driveACR      (void *pvParameters);

// Ping sensor pins
const int center_echo_pin = 10;
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
const int RightMotorPWM_pin = 3;

// OOP Initializations
MotorControl leftMotor  (LeftMotorA_pin,  LeftMotorB_pin,  LeftMotorPWM_pin);
MotorControl rightMotor (RightMotorA_pin, RightMotorB_pin, RightMotorPWM_pin);
Ultrasound leftUltrasound   (left_echo_pin, left_trig_pin);
Ultrasound centerUltrasound (center_echo_pin, center_trig_pin);
Ultrasound rightUltrasound  (right_echo_pin, right_trig_pin);

void setup() {
  Serial.begin(9600); // Set baud-rate
  xTaskCreate(driveACR,       (const portCHAR *) "Driving",         128, NULL, 1, NULL); // Priority 1
  xTaskCreate(updateOrders,   (const portCHAR *) "Updating Orders", 128, NULL, 2, NULL); // Priority 2
  xTaskCreate(updatePingData, (const portCHAR *) "Updating Pings",  128, NULL, 3, NULL); // Priority 3
  go_stop(); // Guarantee that both motors are not moving at start
  set_speed(PWM, PWM); // Kind-of useless since we set it on every movement command (see below)
}

// This is supposed to be empty (lets RTOS run uninterupted)
void loop() {
}

void set_speed(const int left_speed, const int right_speed) {
  leftMotor.setPWM(left_speed);
  rightMotor.setPWM(right_speed);
}

void go_stop() {
  leftMotor.halt();
  rightMotor.halt();
}

void go_forward() {
  set_speed(PWM, PWM);
  leftMotor.forward();
  rightMotor.backward();
}

void go_backward() {
  set_speed(PWM - 3, PWM); // Compensate for difference that occurs when backing up
  leftMotor.backward();
  rightMotor.forward();
}

void go_left() {
  set_speed(PWM, PWM);
  leftMotor.backward();
  rightMotor.backward();
}

void go_right() {
  set_speed(PWM, PWM);
  leftMotor.forward();
  rightMotor.forward();
}

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
        default:
          Serial.println("ERROR: Bad Message from Serial - character was not expected");
          break;
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

//check sensors for new obstacles
void updatePingData(void *pvParameters) {
  while (1) {
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

    vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}

//drives the robot accoring to the last received order
void driveACR(void *pvParameters) {
  while (1) {

    // Print logs below... uncomment what's needed
    // ---------------------------------------------
    //Serial.print("L: "); Serial.print(leftDistance);Serial.print("      C: "); Serial.print(centerDistance);Serial.print("      R: "); Serial.println(rightDistance);
    //Serial.print("Current Direction : "); Serial.println(currDir);
    //Serial.print("Avoid enabled : "); Serial.println(avoidObstaclesEnabled);

    // Two variations of obstacle avoidance...
    //  1) Person following - just halt on obstacles
    //  2) Free Roam

    if (followType == person) {
      // Person following responds to serial commands, but ignores them and halts when we're in danger.
      // EXCEPT left, right, and halt commands (which we should let through)...

      if (currDir == left || currDir == right || currDir == halt) {
        // Current direction is left/right/halt, so lets let the robot move that way
        respondToCurrDir();
      } else {
        lastCommandFromSerial = false;
        go_stop();
      }
    } else if (followType == roam) {
      // Free Roam

      // Check where the danger is...

      if (centerDistance <= STOP_DISTANCE_CENTER) { // If Danger at center...
        // Check whether we should go left or right
        if (leftDistance <= rightDistance) { // Right has more room than left, so go right.
          lastCommandFromSerial = false;
          prevDir = currDir;
          currDir = right;
          respondToCurrDir();
        }
        else { // Left has more room than right, so go left.
          lastCommandFromSerial = false;
          prevDir = currDir;
          currDir = left;
          respondToCurrDir();
        }
      } else if (leftDistance <= STOP_DISTANCE_SIDE) { // Issue @ left, so go right
        lastCommandFromSerial = false;
        prevDir = currDir;
        currDir = right;
        respondToCurrDir();
      } else if (rightDistance <= STOP_DISTANCE_SIDE) { // Issue @ right, so go left
        lastCommandFromSerial = false;
        prevDir = currDir;
        currDir = left;
        respondToCurrDir();
      } else {
        lastCommandFromSerial = false;
        prevDir = currDir;
        currDir = forward;
        respondToCurrDir();
      }
    } else { // Neither followType matched... not possible
      Serial.println("ERROR: followType had no match");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
