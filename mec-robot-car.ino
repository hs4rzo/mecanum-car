
//KITTITHUS KAISORN
#include <vector>
#include <ArduinoOTA.h>
#include <WiFiMulti.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <esp_wifi.h>
WiFiMulti wifiMulti;

//S3
#define CE_PIN 9
#define CSN_PIN 10
#define SCK_PIN 12
#define MOSI_PIN 11
#define MISO_PIN 13
#define SIGNAL_TIMEOUT 500
// สร้างออบเจ็กต์ RF24
//RF24 radio(15, 2);//38
RF24 radio(CE_PIN, CSN_PIN);  //s3
const byte address[6] = "00001";
// Watchdog timer period in seconds
#define FORWARD 1
#define BACKWARD 2
#define STOP 0
#define TRIGGER_PIN 7
#define ECHO_PIN 15
#define MENU 0
#define MECANUM 1
#define MP3 2
#define STANDBY 3
#define WIFI 4
#define RO 5

bool connectstatus = false;

const int PWM_MOVE = 0;
const int PWM_SCAN = 0;
const int SAFE_DISTANCE = 100;     // cm, ปลอดภัยพอจะเดินหน้า
const int OBSTACLE_DISTANCE = 70;  // cm, เจอสิ่งกีดขวางต้องหยุด
const int SCAN_TIME = 2000;        // ms, เวลาสแกนหมุนหาทางใหม่
bool camrandom = true;
#define RIGHT_FRONT_MOTOR 0
#define LEFT_FRONT_MOTOR 1
#define RIGHT_REAR_MOTOR 2
#define LEFT_REAR_MOTOR 3
const int mtrPWMFreq = 5000;
// PWM Resolution
const int mtrPWMResolution = 12;
enum State {
  IDLE,
  SCANNING,
  MOVING_FORWARD,
  ROTATING,
  WAIT
};
enum DiagonalDirection {
  NONE,
  LEFT,
  RIGHT
};

unsigned long startackDatatTime = 0;
State currentState = IDLE;
unsigned long lastActionTime = 0;
/*
std::vector<MOTOR_PINS> motorPins = {
  // 38 pin
  { 22, 21, 5 },
  { 25, 27, 26 },
  { 4, 17, 16 },
  { 13, 12, 14 }
};
*/
struct MOTOR_PINS {
  int pinEN;
  int pinIN1;
  int pinIN2;
};
std::vector<MOTOR_PINS> motorPins = {
  // s3
  { 38, 37, 36 },  // Right Front Motor    L298N-Right
  { 4, 6, 5 },     //  Left  Front Motor    L298N-Left
  { 20, 35, 21 },  // Right Rear Motor     L298N-Right
  { 8, 18, 17 }    // Left Rear Motor   L298N-Left
};
typedef struct __attribute__((packed)) struct_message_rcv {
  int16_t rf_speed;  // -4095 ถึง 4095
  int16_t lf_speed;
  int16_t rr_speed;
  int16_t lr_speed;
  bool pbSwitch;      // true / false
  uint16_t MAXSPEED;  // 0 ถึง 4095
  uint8_t carMode;
} struct_message_rcv;
struct_message_rcv rcvData;


typedef struct __attribute__((packed)) struct_message_ack {
  int16_t mtrRF_PWM;
  int16_t mtrLF_PWM;
  int16_t mtrRR_PWM;
  int16_t mtrLR_PWM;
} struct_message_ack;
struct_message_ack ackData;
unsigned long lastRecvTime = 0;

volatile uint8_t carMode = 0;
// Variables for Motor PWM speeds
int16_t mtrRFpwmValue = 0;
int16_t mtrLFpwmValue = 0;
int16_t mtrRRpwmValue = 0;
int16_t mtrLRpwmValue = 0;
// Variables for Joystick values
int16_t rf_speed = 0;  // -4095 ถึง 4095
int16_t lf_speed = 0;
int16_t rr_speed = 0;
int16_t lr_speed = 0;
int16_t MAXSPEED = 0;  // 0 ถึง 4095
// Variable for Joystick pushbutton state
volatile bool pbSwitch = true;
unsigned long lastPlayTime = 0;
unsigned long interval = 10000;  // 20 วินาที
void setUpPinModes() {
  for (int i = 0; i < motorPins.size(); i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    pinMode(motorPins[i].pinEN, OUTPUT);
    ledcAttach(motorPins[i].pinEN, mtrPWMFreq, mtrPWMResolution);
  }
  stopMotors();
  motorControlMode0(rf_speed, lf_speed, rr_speed, lr_speed);
  pinMode(TRIGGER_PIN, OUTPUT);  // ตั้ง TRIGGER_PIN เป็น OUTPUT
  pinMode(ECHO_PIN, INPUT);      // ตั้ง ECHO_PIN เป็น INPUT
}

void disconnectWiFi() {
  WiFi.disconnect(true);  // ล้าง config ด้วย
  WiFi.mode(WIFI_OFF);    // ปิดวิทยุทั้งหมด
  esp_wifi_stop();        // ปิด Wi-Fi stack
  ArduinoOTA.end();
}
void setup() {
  Serial.begin(115200);
  // SPI.begin(CLK_PIN, -1, DATA_PIN, CS_PIN);
  setUpPinModes();
  //pinMode(NRF_IRQ_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), nrf_isr, FALLING);
  if (!radio.begin()) {
    Serial.println("ไม่พบโมดูล nRF24L01 แล้ว!");
    while (1)
      ;  // แฮงก์ไว้เลยหรือแจ้งเตือนเรื่อยๆ
  } else
    Serial.println(" พบโมดูล nRF24L01 แล้ว!");
  //radio.openWritingPipe(address); //ถ้าใช้ ACK Payload ตรงนี้ไม่ใช้
  // radio.openWritingPipe(address);
  //radio.enableDynamicPayloads();
  analogReadResolution(12);

  radio.setChannel(108);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.enableAckPayload();
  // radio.setPayloadSize(sizeof(rcvData));
  radio.enableDynamicPayloads();  //ปรับขนาดของ Payload ได้อัตโนมัติตามขนาดของข้อมูลจริงที่ถูกส่งมา
  radio.setAutoAck(true);
  radio.openReadingPipe(1, address);

  ackData.mtrRF_PWM = mtrRFpwmValue;
  ackData.mtrLF_PWM = mtrLFpwmValue;
  ackData.mtrRR_PWM = mtrRRpwmValue;
  ackData.mtrLR_PWM = mtrLRpwmValue;
  radio.writeAckPayload(1, &ackData, sizeof(ackData));
  radio.startListening();
}
long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
int readDistanceSensor() {
  long duration, inches, cm;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  //inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  return cm;
}



void loop() {
  ArduinoOTA.handle();
  int timedelay = 20;
  String message = "0";
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT) {
    stopMotors();
    if (WiFi.getMode() == WIFI_OFF)
      setupwifi();
  } else {
    if (WiFi.getMode() != WIFI_OFF)
      disconnectWiFi();
  }

  if (radio.available()) {
    byte len = radio.getDynamicPayloadSize();
    radio.read(&rcvData, len);
    ackData.mtrRF_PWM = mtrRFpwmValue;
    ackData.mtrLF_PWM = mtrLFpwmValue;
    ackData.mtrRR_PWM = mtrRRpwmValue;
    ackData.mtrLR_PWM = mtrLRpwmValue;
    radio.writeAckPayload(1, &ackData, sizeof(ackData));
    pbSwitch = rcvData.pbSwitch;
    MAXSPEED = rcvData.MAXSPEED;
    carMode = rcvData.carMode;
    if (pbSwitch)
      automove();
    else {
      currentState = IDLE;
      rf_speed = rcvData.rf_speed;
      lf_speed = rcvData.lf_speed;
      rr_speed = rcvData.rr_speed;
      lr_speed = rcvData.lr_speed;
    }
    lastRecvTime = millis();
  }

  handleMotorSpeed(rf_speed, mtrRFpwmValue);
  handleMotorSpeed(lf_speed, mtrLFpwmValue);
  handleMotorSpeed(rr_speed, mtrRRpwmValue);
  handleMotorSpeed(lr_speed, mtrLRpwmValue);
  motorControlMode0(rf_speed, lf_speed, rr_speed, lr_speed);
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
  //delay(timedelay);
}
