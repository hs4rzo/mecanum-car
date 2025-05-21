void setupwifi() {
  esp_wifi_start();
  WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP
    WiFi.enableIPv6();
  wifiMulti.addAP("Car", "999999999");
  wifiMulti.addAP("Milin_2.4G", "999999999");
  Serial.println("Connecting Wifi...");

  unsigned long startAttemptTime = millis();
  while (wifiMulti.run() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(100);
  }
  WiFi.linkLocalIPv6();
  ArduinoOTA.onStart([]() {
              String type;
              if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
              } else {  // U_SPIFFS
                type = "filesystem";
              }
              // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
              Serial.println("Start updating " + type);
            })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();
  // แสดง IPv6 Address
  Serial.print("IPv6 Address: ");
  Serial.println(WiFi.linkLocalIPv6());
}

int getMappedPWM(int raw) {
  if (abs(raw) < 1000)
    raw = 0;
  return raw;
}
void rotateMotor(int motorNumber, int motorSpeed, byte dircontrol, int bit1, int bit2) {
  digitalWrite(motorPins[motorNumber].pinIN1, bitRead(dircontrol, bit1));
  digitalWrite(motorPins[motorNumber].pinIN2, bitRead(dircontrol, bit2));
  ledcWrite(motorPins[motorNumber].pinEN, abs(motorSpeed));
}
void stopMotorsss() {
  // Stops all motors and motor controllers
  rotateMotor0(RIGHT_FRONT_MOTOR, 0);
  rotateMotor0(LEFT_FRONT_MOTOR, 0);
  rotateMotor0(RIGHT_REAR_MOTOR, 0);
  rotateMotor0(LEFT_REAR_MOTOR, 0);
  rf_speed = 0;
  lf_speed = 0;
  rr_speed = 0;
  lr_speed = 0;  
}