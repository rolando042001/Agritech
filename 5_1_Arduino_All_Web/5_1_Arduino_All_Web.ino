#include <AFMotor.h>
#include <Servo.h>
#include <SharpIR.h>
#include <math.h>

// ======= MOTOR DEFINITIONS USING AFMotor =======
AF_DCMotor motorFL(1, MOTOR34_8KHZ);   // Front Left
AF_DCMotor motorFR(2, MOTOR34_8KHZ);   // Front Right
AF_DCMotor motorBL(3, MOTOR34_8KHZ);   // Back Left
AF_DCMotor motorBR(4, MOTOR34_8KHZ);   // Back Right

// ===== IR PINS =====
#define IR_FL A0
#define IR_FR A1
#define IR_BL A2
#define IR_BR A3

// ======= ULTRASONIC =======
// #define Trig_PIN 12
// #define Echo_PIN 13

// // ======= LINE TRACKING =======
// #define LEFT_LINE_TRACJING   A2
// #define CENTER_LINE_TRACJING A1
// #define right_LINE_TRACJING  A0

// ======= COMMAND CONSTANTS =======
const int Forward       = 163;
const int Backward      = 92;
const int Turn_Left     = 106;
const int Turn_Right    = 149;
const int Top_Left      = 34;
const int Bottom_Left   = 72;
const int Top_Right     = 129;
const int Bottom_Right  = 20;
const int Stop          = 0;
const int Contrarotate  = 83;
const int Clockwise     = 172;
const int Model1        = 25;
const int Model2        = 26;
const int Model3        = 27;
const int Model4        = 28;
const int Speed         = 29;

const int Servo_tilt      = 30;   // existing servo tilt
const int Servo_pan       = 40;   // pan servo
const int Servo_dolly     = 41;   // dolly servo
const int Actuator_up     = 225;  // actuator extend
const int Actuator_down   = 226;  // actuator retract
const int Blower_fan      = 227 ;  // blower fan


// ======= SERVO PINS =======
#define SERVO_TILT_PIN 13    // existing tilt servo
#define SERVO_PAN_PIN  10    // NEW pan servo
#define SERVO_DOLLY_PIN 9   // NEW dolly servo
Servo MOTORservo;            // tilt servo
Servo PANservo;              // pan servo
Servo DOLLYservo;            // dolly servo

// ======= ACTUATOR PINS =======
#define ACTUATOR_UP_PIN   A4   // extend
#define ACTUATOR_DOWN_PIN A5   // retract

// ======== BLOWER FAN PIN =======
#define BLOWER_FAN_PIN 2 // Blower pin


int speeds = 250;
int Left_Tra_Value, Center_Tra_Value, Right_Tra_Value;
int Black_Line = 400;
uint16_t angle = 90;

byte RX_package[4] = {0};
byte model_var = 0; 
byte val = 0;

int UT_distance;


// ===================================================
//  ACCURATE SHARP IR FORMULA FUNCTION
// ===================================================
float getIRdistanceCM(int pin) {
  float voltage = analogRead(pin) * (5.0 / 1023.0);

  float distanceCM = 27.86 * pow(voltage, -1.15); // Real sensor curve

  // keep realistic usable range for GP2Y0A21
  if (distanceCM < 10) distanceCM = 10;
  if (distanceCM > 80) distanceCM = 80;

  return distanceCM;
}


// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  MOTORservo.attach(SERVO_TILT_PIN);   // tilt servo
  PANservo.attach(SERVO_PAN_PIN);      // pan servo
  DOLLYservo.attach(SERVO_DOLLY_PIN); // dolly servo

  MOTORservo.write(angle);             // initialize tilt servo
  PANservo.write(90);                  // center pan servo
  DOLLYservo.write(90);                // center dolly servo 

  pinMode(ACTUATOR_UP_PIN, OUTPUT);
  pinMode(ACTUATOR_DOWN_PIN, OUTPUT);
  pinMode(BLOWER_FAN_PIN, OUTPUT);


  // make sure actuator is OFF
  digitalWrite(ACTUATOR_UP_PIN, HIGH);
  digitalWrite(ACTUATOR_DOWN_PIN, HIGH);

  // make sure blower fan is OFF
  digitalWrite(BLOWER_FAN_PIN, HIGH);

  // pinMode(Trig_PIN, OUTPUT);
  // pinMode(Echo_PIN, INPUT);

  // pinMode(LEFT_LINE_TRACJING, INPUT);
  // pinMode(CENTER_LINE_TRACJING, INPUT);
  // pinMode(right_LINE_TRACJING, INPUT);

  Motor(Stop, 0);
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  RXpack_func();

  switch (model_var) {
    case 0: model1_func(val); break;
    case 1: model2_func(); break;
    // case 2: model3_func(); break;
    // case 3: model4_func(); break;
    case 4: Servo_Move(); break;
    case 5: break;  // pan servo direct
    case 6: break;  // dolly servo direct

    
  }
}

// ===================================================
// MODEL 1 — MANUAL CONTROL
// ===================================================
void model1_func(byte values) {
  Motor(values, speeds);
}

// ===================================================
// MODEL 2 — 4 IR OBSTACLE AVOID (FL, FR, BL, BR)
// ===================================================
void model2_func() {
  while (model_var == 1) {

    RXpack_func();
    if (model_var != 1) break;

    // Read FRONT IR sensors
    float FL = getIRdistanceCM(IR_FL);
    float FR = getIRdistanceCM(IR_FR);

    // Read BACK IR sensors
    float BL = getIRdistanceCM(IR_BL);
    float BR = getIRdistanceCM(IR_BR);

    float frontMin = min(FL, FR);
    float backMin  = min(BL, BR);

    bool frontBlocked = (frontMin < 25);
    bool backBlocked  = (backMin  < 25);

    Serial.print("FL: "); Serial.print(FL);
    Serial.print("  FR: "); Serial.print(FR);
    Serial.print("  BL: "); Serial.print(BL);
    Serial.print("  BR: "); Serial.println(BR);

    // ------------------------------------------
    // FRONT BLOCKED → Avoid
    // ------------------------------------------
    if (frontBlocked) {

      Motor(Stop, 0);
      delay(120);

      // If back is CLEAR → reverse slightly
      if (!backBlocked) {
        Motor(Backward, speeds - 60);
        delay(300);
      }

      // Choose turn direction
      float diffFront = FL - FR;

      // dead-zone: avoid bias (fix continuous CCW rotation)
      if (abs(diffFront) < 5) {
        // go straight backward briefly to re-center
        Motor(Backward, speeds - 60);
        delay(200);
        continue;
      }

      if (diffFront > 0) {
        Motor(Contrarotate, speeds);  // turn LEFT
      } else {
        Motor(Clockwise, speeds);     // turn RIGHT
      }

      delay(300);
      continue;
    }

    // ------------------------------------------
    // BACK BLOCKED → Move Forward
    // ------------------------------------------
    if (backBlocked) {
      Motor(Forward, speeds - 60);
      delay(300);
      continue;
    }

    // ------------------------------------------
    // NO OBSTACLE → Forward
    // ------------------------------------------
    Motor(Forward, speeds);
  }
}



// ===================================================
// MODEL 3 — FOLLOW MODE
// ===================================================
// void model3_func() {
//   MOTORservo.write(90);
//   UT_distance = SR04();
//   Serial.println(UT_distance);

//   if (UT_distance < 15)
//     Motor(Backward, speeds - 50);
//   else if (UT_distance <= 20)
//     Motor(Stop, 0);
//   else if (UT_distance <= 25)
//     Motor(Forward, speeds - 70);
//   else if (UT_distance <= 50)
//     Motor(Forward, speeds - 30);
//   else
//     Motor(Stop, 0);
// }

// ===================================================
// MODEL 4 — LINE TRACKING
// ===================================================
// void model4_func() {
//   MOTORservo.write(90);

//   Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
//   Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
//   Right_Tra_Value = analogRead(right_LINE_TRACJING);

//   if (Center_Tra_Value <= Black_Line)
//     Motor(Forward, 180);
//   else if (Left_Tra_Value < Black_Line)
//     Motor(Contrarotate, 160);
//   else if (Right_Tra_Value < Black_Line)
//     Motor(Clockwise, 160);
//   else
//     Motor(Forward, 130);
// }

// ===================================================
// SERVO CONTROL
// ===================================================
void Servo_Move() {
  MOTORservo.write(angle);
  delay(10);
}

// ===================================================
// MOTOR CONTROL (AFMotor)
// ===================================================
void Motor(int Dir, int Speed) {
  int spd = map(Speed, 0, 255, 0, 255);

  if (Dir == Stop) {
    motorFL.run(RELEASE);
    motorFR.run(RELEASE);
    motorBL.run(RELEASE);
    motorBR.run(RELEASE);
    return;
  }

  if (Dir == Forward) {
    motorFL.setSpeed(spd);
    motorFR.setSpeed(spd);
    motorBL.setSpeed(spd);
    motorBR.setSpeed(spd);
    motorFL.run(FORWARD);
    motorFR.run(FORWARD);
    motorBL.run(FORWARD);
    motorBR.run(FORWARD);
    return;
  }

  if (Dir == Backward) {
    motorFL.setSpeed(spd);
    motorFR.setSpeed(spd);
    motorBL.setSpeed(spd);
    motorBR.setSpeed(spd);
    motorFL.run(BACKWARD);
    motorFR.run(BACKWARD);
    motorBL.run(BACKWARD);
    motorBR.run(BACKWARD);
    return;
  }

  if (Dir == Turn_Left) {
    motorFL.setSpeed(spd);
    motorFR.setSpeed(spd);
    motorBL.setSpeed(spd);
    motorBR.setSpeed(spd);
    motorFL.run(BACKWARD);
    motorBL.run(BACKWARD);
    motorFR.run(FORWARD);
    motorBR.run(FORWARD);
    return;
  }

  if (Dir == Turn_Right) {
    motorFL.setSpeed(spd);
    motorFR.setSpeed(spd);
    motorBL.setSpeed(spd);
    motorBR.setSpeed(spd);
    motorFL.run(FORWARD);
    motorBL.run(FORWARD);
    motorFR.run(BACKWARD);
    motorBR.run(BACKWARD);
    return;
  }

  if (Dir == Clockwise) {
    motorFL.setSpeed(spd);
    motorFR.setSpeed(spd);
    motorBL.setSpeed(spd);
    motorBR.setSpeed(spd);
    motorFL.run(BACKWARD);
    motorBL.run(FORWARD);
    motorFR.run(BACKWARD);
    motorBR.run(FORWARD);
    return;
  }

  if (Dir == Contrarotate) {
    motorFL.setSpeed(spd);
    motorFR.setSpeed(spd);
    motorBL.setSpeed(spd);
    motorBR.setSpeed(spd);
    motorFL.run(FORWARD);
    motorBL.run(BACKWARD);
    motorFR.run(FORWARD);
    motorBR.run(BACKWARD);
    return;
  }

  if (Dir == Top_Left) {
    motorFL.run(BACKWARD);
    motorBL.run(BACKWARD);
    motorFR.run(FORWARD);
    motorBR.run(FORWARD);
    return;
  }

  if (Dir == Top_Right) {
    motorFL.run(FORWARD);
    motorBL.run(FORWARD);
    motorFR.run(BACKWARD);
    motorBR.run(BACKWARD);
    return;
  }

  if (Dir == Bottom_Left) {
    motorFL.run(BACKWARD);
    motorBL.run(FORWARD);
    motorFR.run(FORWARD);
    motorBR.run(BACKWARD);
    return;
  }

  if (Dir == Bottom_Right) {
    motorFL.run(FORWARD);
    motorBL.run(BACKWARD);
    motorFR.run(BACKWARD);
    motorBR.run(FORWARD);
    return;
  }
}

// ===================================================
// ULTRASONIC
// ===================================================
// float SR04() {
//   digitalWrite(Trig_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(Trig_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(Trig_PIN, LOW);
//   return pulseIn(Echo_PIN, HIGH) / 58.0;
// }

// ===================================================
// RECEIVE COMMAND PACKET
// ===================================================
void RXpack_func() {
  if (Serial.available() > 0) {
    if (Serial.readBytes(RX_package, 4)) {
      if (RX_package[0] == 0xAB && RX_package[3] == 0xFF) {

        byte Model = RX_package[1];
        val = RX_package[2];

        // SPEED
        if (Model == Speed) {
          speeds = val;
        }

        // MODE SELECT
        else if (Model == Model1) model_var = 0;
        else if (Model == Model2) model_var = 1;
        else if (Model == Model3) model_var = 2;
        else if (Model == Model4) model_var = 3;

        // ===== TILT SERVO =====
        else if (Model == Servo_tilt) {
          angle = val;
          model_var = 4;
        }

        // ===== PAN SERVO =====
        else if (Model == Servo_pan) {
          PANservo.write(val);
          model_var = 5;
        }

        // ====== DOLYY SERVO ====
        else if (Model == Servo_dolly) {
           DOLLYservo.write(val);        
           model_var = 6;
        }

        else if (Model == Actuator_up) {
  // ACTUATOR DEPLOY
  digitalWrite(ACTUATOR_UP_PIN, LOW);
  digitalWrite(ACTUATOR_DOWN_PIN, HIGH);

  // ensure blower OFF
  digitalWrite(BLOWER_FAN_PIN, HIGH);
}

else if (Model == Actuator_down) {
  // ACTUATOR RETRACT
  digitalWrite(ACTUATOR_UP_PIN, HIGH);
  digitalWrite(ACTUATOR_DOWN_PIN, LOW);

  // ensure blower OFF
  digitalWrite(BLOWER_FAN_PIN, HIGH);
}

else if (Model == 0) {
  // ACTUATOR STOP COMMAND
  digitalWrite(ACTUATOR_UP_PIN, HIGH);
  digitalWrite(ACTUATOR_DOWN_PIN, HIGH);
}

else if (Model == Blower_fan) {
  // BLOWER ON
  digitalWrite(BLOWER_FAN_PIN, LOW);

  // FORCE ACTUATOR OFF
  digitalWrite(ACTUATOR_UP_PIN, HIGH);
  digitalWrite(ACTUATOR_DOWN_PIN, HIGH);
}

else if (Model == 228) {
  // BLOWER OFF
  digitalWrite(BLOWER_FAN_PIN, HIGH);
}
      }
    }
  }
}

