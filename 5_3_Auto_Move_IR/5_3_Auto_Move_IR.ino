#include <AFMotor.h>
#include <SharpIR.h>

// ======== MOTOR SETUP ======== //
AF_DCMotor M1(1);   // Front Left
AF_DCMotor M2(2);   // Front Right
AF_DCMotor M3(3);   // Rear Right
AF_DCMotor M4(4);   // Rear Left

// ======== SHARP IR SETUP ======== //
#define IR1 A1
#define IR2 A2
#define model 1080   // GP2Y0A21YK0F model code

SharpIR SharpIR1(IR1, model);
SharpIR SharpIR2(IR2, model);

int safeDistance = 20;  // cm threshold

// ======== MOTOR SPEED ======== //
void setup() {
  Serial.begin(9600);

  M1.setSpeed(200);
  M2.setSpeed(200);
  M3.setSpeed(200);
  M4.setSpeed(200);
}

// ======== MOVEMENT FUNCTIONS ======== //
void stopMove() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void turnLeft() {
  M1.run(BACKWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(BACKWARD);
}

void turnRight() {
  M1.run(FORWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(FORWARD);
}

// ======== OBSTACLE DETECTION FUNCTION ======== //
bool obstacleDetected() {
  int d1 = SharpIR1.distance();
  int d2 = SharpIR2.distance();

  Serial.print("IR1: "); Serial.print(d1); Serial.print(" cm   ");
  Serial.print("IR2: "); Serial.println(d2);

  if (d1 < safeDistance || d2 < safeDistance) {
    return true;
  }
  return false;
}

// ======== MAIN LOOP ======== //
void loop() {
  if (obstacleDetected()) {
    Serial.println("OBSTACLE DETECTED!");
    stopMove();
    delay(300);

    backward();
    delay(400);

    stopMove();
    delay(200);

    // turnRight();
    // delay(400);

    stopMove();
  }
  else {
    forward(); 
  }

  delay(50);
}
