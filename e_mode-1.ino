

unsigned long WaitTime = 0;
void automove() {
  static State targetState = IDLE;
  int distance = 0;
  switch (currentState) {
    case WAIT:
      if (millis() - WaitTime > SCAN_TIME) {
        if (targetState == ROTATING)
          lastActionTime = millis();
        currentState = targetState;
      }
      break;
    case IDLE:
      currentState = SCANNING;
      break;
    case SCANNING:
      distance = readDistanceSensor();
      if (distance > SAFE_DISTANCE) {
        stopMotors();
        currentState = WAIT;
        WaitTime = millis();
        targetState = MOVING_FORWARD;
      } else {
        currentState = ROTATING;
        lastActionTime = millis();
      }
      break;
    case MOVING_FORWARD:
      distance = readDistanceSensor();
      if (distance > 0 && distance < OBSTACLE_DISTANCE) {
        stopMotors();
        currentState = WAIT;
        WaitTime = millis();
        targetState = ROTATING;
      } else {
        if (distance > SAFE_DISTANCE)
          moveForward();
        else if (distance != 0)
          moveDiagonal();
      }
      break;
    case ROTATING:
      rotateSlowly();
        if (millis() - lastActionTime > (SCAN_TIME)) {
          stopMotors();
          currentState = WAIT;
          WaitTime = millis();
          targetState = SCANNING;
        }
      
      break;
  }
}
