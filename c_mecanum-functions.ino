
void toggleMecanumMode() {
  // Toggle Mecanum Mode value from 0 to 5
  int currentMecMode = carMode;
  if (currentMecMode == 2) {
    carMode = 0;
  } else {
    carMode = currentMecMode + 1;
  }
}
void dancecar() {
  static bool valuecontrol = false;
  int16_t rfspeed = 0;
  int16_t lfspeed = 0;
  int16_t rrspeed = 0;
  int16_t lrspeed = 0;
  int16_t speedmax = MAXSPEED;    // เริ่มต้น 3000
  int16_t speedmin = 1500;      // ค่าต่ำสุดที่รถจะเคลื่อนที่
  int delaywait = 1000 + random(1000);
  if (millis() - lastPlayTime >= delaywait) {
    lastPlayTime = millis();
    if (valuecontrol) {
      rfspeed = lfspeed = rrspeed = lrspeed = 0;
      valuecontrol = false;
    } else {
      valuecontrol = true;
     float distance =  readDistanceSensor();  // เช็ควัดระยะหากไปเข้าใกล้สิ่งกีดขวาง แล้วเก็บไว้ที่ตัวแปร distance
      if (distance != 0 && distance < 30) {
        rfspeed = lfspeed = rrspeed = lrspeed = -2500;
      } else {
        int16_t joyxaxis = random(speedmin, speedmax) * (random(2) * 2 - 1);
        int16_t joyyaxis = random(speedmin, speedmax) * (random(2) * 2 - 1);
        int16_t joyXaxis2 = random(speedmin, speedmax) * (random(2) * 2 - 1);
        rfspeed = constrain(joyyaxis + joyxaxis - joyXaxis2, -speedmax, speedmax);
        lfspeed = constrain(joyyaxis - joyxaxis + joyXaxis2, -speedmax, speedmax);
        rrspeed = constrain(joyyaxis - joyxaxis - joyXaxis2, -speedmax, speedmax);
        lrspeed = constrain(joyyaxis + joyxaxis + joyXaxis2, -speedmax, speedmax);
      }
    }
    motorControlMode0(rfspeed, lfspeed, rrspeed, lrspeed);
  }
}
