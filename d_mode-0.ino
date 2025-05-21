
void rotateMotor0(int16_t motorNumber, int16_t motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
    ledcWrite(motorPins[motorNumber].pinEN, abs(motorSpeed));
  } else if (motorSpeed < 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
    ledcWrite(motorPins[motorNumber].pinEN, abs(motorSpeed));
  } else {
    ledcWrite(motorPins[motorNumber].pinEN, 0);
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
}

void handleMotorSpeed(int16_t &joySpeed, int16_t mtrPwmValue) {
  if (joySpeed != 0) {
    if ((joySpeed > 0 && mtrPwmValue < 0) || (joySpeed < 0 && mtrPwmValue > 0) || (abs(joySpeed) < 700)) {
      joySpeed = 0;
    }
  }
}

void motorControlMode0(int16_t joyrf_speed, int16_t joylf_speed, int16_t joyrr_speed, int16_t joylr_speed) {
  rotateMotor0(RIGHT_FRONT_MOTOR, joyrf_speed);
  rotateMotor0(LEFT_FRONT_MOTOR, joylf_speed);
  rotateMotor0(RIGHT_REAR_MOTOR, joyrr_speed);
  rotateMotor0(LEFT_REAR_MOTOR, joylr_speed);
}
void moveForward() {
  camrandom = true;
  if (rf_speed < MAXSPEED)
    rf_speed = constrain(rf_speed + 100, 2400, 4000);
  if (lf_speed < MAXSPEED)
    lf_speed = constrain(lf_speed + 100, 2400, 4000);
  if (rr_speed < MAXSPEED)
    rr_speed = constrain(rr_speed + 100, 2400, 4000);
  if (lr_speed < MAXSPEED)
    lr_speed = constrain(lr_speed + 100, 2400, 4000);
}
static short rd = 0;
void moveDiagonal() {
  rd = 0;
  if (camrandom) {
    rd = random(2);
    camrandom = false;
  }
  if (rd == 1) {
    rf_speed = MAXSPEED + 100;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = MAXSPEED + 100;
  } else {
    rf_speed = 0;
    lf_speed = MAXSPEED + 100;
    rr_speed = MAXSPEED + 100;
    lr_speed = 0;
  }
}

void rotateSlowly() {  // หมุนซ้าย
  camrandom = true;
  int16_t SPEEDSLOW = 2600;
  if (rd == 0) {
    rf_speed = -SPEEDSLOW;
    lf_speed = SPEEDSLOW;
    rr_speed = -SPEEDSLOW;
    lr_speed = SPEEDSLOW;
  } else {
    rf_speed = SPEEDSLOW;
    lf_speed = -SPEEDSLOW;
    rr_speed = SPEEDSLOW;
    lr_speed = -SPEEDSLOW;
  }
}
void stopMotors() {
  camrandom = true;
  rf_speed = 0;
  lf_speed = 0;
  rr_speed = 0;
  lr_speed = 0;
  rotateMotor0(RIGHT_FRONT_MOTOR, 0);
  rotateMotor0(LEFT_FRONT_MOTOR, 0);
  rotateMotor0(RIGHT_REAR_MOTOR, 0);
  rotateMotor0(LEFT_REAR_MOTOR, 0);
}
