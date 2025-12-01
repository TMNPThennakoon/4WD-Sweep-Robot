/*
 * 4WD Sumo-Style Border-Avoid & Obstacle-Pushing Robot
 * 
 * Author: T.M.N.P.Thennakoon
 * Repository: https://github.com/TMNPThennakoon/4WD-Sweep-Robot
 * License: MIT
 * 
 * This Arduino sketch controls a 4WD robot with:
 * - Border detection using IR line sensors
 * - Obstacle detection using HC-SR04 ultrasonic sensor
 * - Two operational modes: ROAM and PUSH
 */

#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// ------------ Ultrasonic Sensor (HC-SR04) ------------
#define TRIGGER_PIN A1
#define ECHO_PIN    A0
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ------------ IR Line Sensors (DIGITAL) ---------------
#define IR_FL A4    // Front Left  (down-facing)
#define IR_FR A5    // Front Right (down-facing)

// ------------ Motors (Adafruit Motor Shield v1) -------
// motor1 + motor2 = LEFT side
// motor3 + motor4 = RIGHT side
// m4,m1 = front, m3,m2 = back
AF_DCMotor motor1(1);  // Front Left
AF_DCMotor motor2(2);  // Back Left
AF_DCMotor motor3(3);  // Back Right
AF_DCMotor motor4(4);  // Front Right

// ------------ Servo (optional, not really used) ------
Servo scanner;
#define SERVO_PIN 10

// ------------ Speed Settings --------------------------
const int SPEED_FORWARD = 90;
const int SPEED_BACK    = 90;
const int SPEED_TURN    = 100;
const int SPEED_PUSH    = 100;    // faster speed for pushing obstacle

// Time constants (ms)
const int BACK_TIME     = 300;   // how long to go back when black detected
const int TURN_TIME     = 350;   // how long to turn after backing

// Ultrasonic behaviour
const int OBSTACLE_DISTANCE = 25;   // cm: obstacle considered "in front"

// ------------ Robot Modes -----------------------------
enum RobotMode {
  MODE_ROAM = 0,   // free roaming inside box
  MODE_PUSH = 1    // pushing obstacle towards border
};

RobotMode mode = MODE_ROAM;

// ------------ Helper: Motor Control -------------------
void setLeftRight(int leftSpeed, int rightSpeed) {
  // left side: motor1, motor2
  motor1.setSpeed(abs(leftSpeed));
  motor2.setSpeed(abs(leftSpeed));

  if (leftSpeed > 0) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  } else if (leftSpeed < 0) {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  } else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }

  // right side: motor3, motor4
  motor3.setSpeed(abs(rightSpeed));
  motor4.setSpeed(abs(rightSpeed));

  if (rightSpeed > 0) {
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  } else if (rightSpeed < 0) {
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  } else {
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  setLeftRight(SPEED_FORWARD, SPEED_FORWARD);
}

void moveBackward() {
  setLeftRight(-SPEED_BACK, -SPEED_BACK);
}

void turnLeft() {
  // left side backward, right side forward (spin on spot)
  setLeftRight(-SPEED_TURN, SPEED_TURN);
}

void turnRight() {
  // left side forward, right side backward (spin on spot)
  setLeftRight(SPEED_TURN, -SPEED_TURN);
}

// ------------ Border Handling (same as before) --------
void handleBorder(bool fl, bool fr) {
  // ----- CASE 2: Left sensor on black -----
  if (fl == HIGH && fr == LOW) {
    moveBackward();
    delay(BACK_TIME);
    turnRight();
    delay(TURN_TIME);
    stopMotors();
  }
  // ----- CASE 3: Right sensor on black -----
  else if (fl == LOW && fr == HIGH) {
    moveBackward();
    delay(BACK_TIME);
    turnLeft();
    delay(TURN_TIME);
    stopMotors();
  }
  // ----- CASE 4: both sensors on black (corner / direct hit) -----
  else { // fl == HIGH && fr == HIGH
    moveBackward();
    delay(BACK_TIME + 150);

    if (random(2) == 0) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(TURN_TIME + 150);
    stopMotors();
  }

  // after touching border, go back to roaming mode
  mode = MODE_ROAM;
}

// ------------ SETUP -----------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(IR_FL, INPUT);
  pinMode(IR_FR, INPUT);

  scanner.attach(SERVO_PIN);
  scanner.write(90);   // center

  randomSeed(analogRead(0)); // for random corner turns
}

// ------------ LOOP ------------------------------------
void loop() {
  // Read IR sensors (DIGITAL)
  bool fl = digitalRead(IR_FL);
  bool fr = digitalRead(IR_FR);

  // *** IMPORTANT ***
  // If your sensor gives:
  //   WHITE = HIGH, BLACK = LOW
  // then invert here:
  // fl = !fl;
  // fr = !fr;

  // Here we assume:
  //   WHITE area  -> LOW (0)
  //   BLACK line  -> HIGH (1)

  // ---- 1) BORDER HAS HIGHEST PRIORITY ----
  if (fl == HIGH || fr == HIGH) {
    handleBorder(fl, fr);
    delay(20);
    return;  // don't do ultrasonic logic in this loop when border touched
  }

  // ---- 2) ULTRASONIC READING ----
  unsigned int dist = sonar.ping_cm();   // 0 = no echo (no obstacle or too far)

  // ---- 3) STATE MACHINE ----
  if (mode == MODE_ROAM) {
    // In roaming mode: move freely unless obstacle appears
    if (dist > 0 && dist < OBSTACLE_DISTANCE) {
      // Obstacle detected in front -> start pushing mode
      mode = MODE_PUSH;
      Serial.println("Obstacle detected -> PUSH MODE");
    } else {
      // No obstacle, just roam forward
      moveForward();
    }
  }
  else if (mode == MODE_PUSH) {
    // In push mode: go faster and keep pushing the obstacle
    if (dist > 0 && dist < (OBSTACLE_DISTANCE + 10)) {
      // Still see obstacle -> push at high speed
      setLeftRight(SPEED_PUSH, SPEED_PUSH);
    } else {
      // Obstacle lost (moved out or pushed outside border)
      // Go back to roaming
      Serial.println("Lost obstacle -> back to ROAM");
      mode = MODE_ROAM;
    }
  }

  delay(40); // small delay for stability
}