const float leftBoost = 1.2; 
const float leftBoostTurn = 1.2;

constexpr float PID_KP = 150.0; 
constexpr float PID_KI = 0.0;
constexpr float PID_KD = 30.0;  
int lineDetectCounter = 0;
unsigned long alignStartTime = 0;

constexpr int SPEED_BASE    = 65;  
constexpr int SPEED_MAX     = 255;  
constexpr int SPEED_TURN    = 80;  
constexpr int SPEED_SEARCH  = 10;

constexpr int OBSTACLE_DISTANCE = 25;  
constexpr int OBSTACLE_CONFIRM  = 3;   

constexpr unsigned long STOP_DELAY            = 500;   
constexpr unsigned long DEBUG_TURN_LEFT_TIME  = 200;   
constexpr unsigned long DEBUG_FWD_TIME        = 800;  
constexpr unsigned long DEBUG_PAUSE_TIME      = 300;   
constexpr unsigned long DEBUG_TURN_RIGHT_TIME = 250;   

constexpr unsigned long DEBUG_BLIND_FWD_TIME  = 600;  
constexpr unsigned long DEBUG_SEARCH_TIMEOUT  = 1000;  

constexpr unsigned long PID_INTERVAL_MS = 20;                     
constexpr float         PID_DT          = PID_INTERVAL_MS / 1000.0; 
constexpr float         PID_CORRECTION_CLAMP = 255.0; 

unsigned long obstacleCheckInterval = 50;   
unsigned long lastObstacleCheck     = 0;
int  obstacleCheckCount = 0; 
bool obstacleFlag = false;

constexpr uint8_t PIN_IR_LEFT  = A0;
constexpr uint8_t PIN_IR_RIGHT = A1;

constexpr uint8_t PIN_ENA = 6;
constexpr uint8_t PIN_IN1 = 8;
constexpr uint8_t PIN_IN2 = 9;

constexpr uint8_t PIN_ENB = 5;
constexpr uint8_t PIN_IN3 = 7; 
constexpr uint8_t PIN_IN4 = 4; 

constexpr uint8_t PIN_TRIG_LEFT  = 2; 
constexpr uint8_t PIN_ECHO_LEFT  = 3;
constexpr uint8_t PIN_TRIG_RIGHT = 11; 
constexpr uint8_t PIN_ECHO_RIGHT = 12;

enum RobotState {
  LINE_FOLLOW,
  OBSTACLE_STOP,
  DEBUG_TURN_LEFT,
  DEBUG_FWD,
  DEBUG_PAUSE,
  DEBUG_TURN_RIGHT,
  DEBUG_FIND_LINE,
  DEBUG_ALIGN_LINE,
  DEBUG_STOP
};

RobotState currentState = LINE_FOLLOW;
unsigned long stateStartTime = 0;
bool rightSignalGiven = false;  

float pidIntegral  = 0;
float pidLastError = 0;
unsigned long pidLastTime = 0;
int lastLineError = 0; 

int getDistance(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;  

  int d = duration * 0.034 / 2;
  return (d < 2 || d > 400) ? -1 : d; 
} 

void updateObstacleCheck() {
  unsigned long now = millis();
  if (now - lastObstacleCheck < obstacleCheckInterval) return;
  lastObstacleCheck = now;

  int dL = getDistance(PIN_TRIG_LEFT,  PIN_ECHO_LEFT);
  int dR = getDistance(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
  
  int currentLimit = (currentState == LINE_FOLLOW) ? OBSTACLE_DISTANCE : 15;
  bool hit = (dL > 0 && dL < currentLimit) || (dR > 0 && dR < currentLimit);

  if (hit) {
    obstacleCheckCount++;
    if (obstacleCheckCount >= OBSTACLE_CONFIRM) {
      obstacleFlag = true;
    }
  } else {
    obstacleCheckCount = 0; 
    obstacleFlag = false;
  }
}

int lineError() {
  int L = digitalRead(PIN_IR_LEFT);
  int R = digitalRead(PIN_IR_RIGHT);

  if (L == LOW && R == HIGH) { lastLineError = 1;  return 1; }
  if (L == HIGH && R == LOW) { lastLineError = -1; return -1; }
  if (L == LOW && R == LOW)  { lastLineError = 0;  return 0; }

  if (L == HIGH && R == HIGH) {
    if (lastLineError < 0) return -2; 
    if (lastLineError > 0) return 2;  
    return 0; 
  }
  return 0;
}

void setMotors(int leftSpeed, int rightSpeed) {
  int boostedLeft = constrain((int)(leftSpeed * leftBoost), -SPEED_MAX, SPEED_MAX);
  int finalRight  = constrain(rightSpeed, -SPEED_MAX, SPEED_MAX);

  analogWrite(PIN_ENA, abs(boostedLeft));
  digitalWrite(PIN_IN1, boostedLeft  < 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, boostedLeft  > 0 ? HIGH : LOW);

  analogWrite(PIN_ENB, abs(finalRight));
  digitalWrite(PIN_IN3, finalRight < 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, finalRight > 0 ? HIGH : LOW);
}

void setMotorsTurn(int leftSpeed, int rightSpeed) {
  int finalLeft  = constrain((int)(leftSpeed * leftBoostTurn), -SPEED_MAX, SPEED_MAX);
  int finalRight = constrain(rightSpeed, -SPEED_MAX, SPEED_MAX);

  analogWrite(PIN_ENA, abs(finalLeft));
  digitalWrite(PIN_IN1, finalLeft  < 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, finalLeft  > 0 ? HIGH : LOW);

  analogWrite(PIN_ENB, abs(finalRight));
  digitalWrite(PIN_IN3, finalRight < 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, finalRight > 0 ? HIGH : LOW);
}

void stopMotors() {
  setMotors(0, 0);
}

void resetPID() {
  pidIntegral = 0;
  pidLastError = 0;
  pidLastTime = millis();
}

void handleLineFollow() {
  if (obstacleFlag) {
    stopMotors();
    currentState   = OBSTACLE_STOP;
    stateStartTime = millis();
    obstacleCheckInterval = 150; 
    obstacleFlag = false; 
    obstacleCheckCount = 0;
    Serial.println("Obstacle Confirmed! -> OBSTACLE_STOP");
    return;
  }

  unsigned long now = millis();
  if (now - pidLastTime < PID_INTERVAL_MS) return;
  pidLastTime = now;

  float error      = lineError();
  pidIntegral     += error * PID_DT;
  pidIntegral      = constrain(pidIntegral, -10.0, 10.0);
  float derivative = (error - pidLastError) / PID_DT;
  pidLastError     = error;

  float correction = PID_KP * error + PID_KI * pidIntegral + PID_KD * derivative;
  correction = constrain(correction, -PID_CORRECTION_CLAMP, PID_CORRECTION_CLAMP);

  setMotors(SPEED_BASE + (int)correction, SPEED_BASE - (int)correction);
}

void handleObstacleStop() {
  if (millis() - stateStartTime >= STOP_DELAY) {
    currentState   = DEBUG_TURN_LEFT;
    stateStartTime = millis();
    Serial.println("-> DEBUG_TURN_LEFT");
  }
}

void handleDebugTurnLeft() {
  setMotorsTurn(-SPEED_TURN, SPEED_TURN); 
  if (millis() - stateStartTime >= DEBUG_TURN_LEFT_TIME) {
    currentState   = DEBUG_FWD;
    stateStartTime = millis();
    Serial.println("-> DEBUG_FWD");
  }
}

void handleDebugFwd() {
  setMotors(SPEED_BASE, SPEED_BASE);
  if(millis() - stateStartTime >= DEBUG_FWD_TIME) {
    currentState = DEBUG_PAUSE;
    stateStartTime = millis();
    Serial.println("-> DEBUG_PAUSE");
  }
}

void handleDebugPause() {
  stopMotors();
  if (millis() - stateStartTime >= DEBUG_PAUSE_TIME) {
    currentState   = DEBUG_TURN_RIGHT;
    stateStartTime = millis();
    Serial.println("-> DEBUG_TURN_RIGHT");
  }
}

void handleDebugTurnRight() {
  setMotorsTurn(SPEED_TURN, -SPEED_TURN); 
  if (millis() - stateStartTime >= DEBUG_TURN_RIGHT_TIME) {
    unsigned long actualTurnDuration = millis() - stateStartTime;  
    currentState      = DEBUG_FIND_LINE; 
    stateStartTime    = millis();
    rightSignalGiven  = false;
    Serial.print("-> DEBUG_FIND_LINE (turn ran for ");
    Serial.print(actualTurnDuration);
    Serial.println("ms)");
  }
}

void handleDebugFindLine() {
  bool left  = digitalRead(PIN_IR_LEFT) == LOW;
  bool right = digitalRead(PIN_IR_RIGHT) == LOW;

  if(millis() - stateStartTime < 100) {
    setMotors(12,12);
    return;
  }

  if(!left && right) {
    setMotorsTurn(25,-25);
    return;
  }

  if(left && right) {
    stopMotors();
    delay(50);
    setMotorsTurn(-20,20);
    delay(80);
    stopMotors();
    resetPID();
    currentState = LINE_FOLLOW;
    Serial.println("BOTH SENSOR -> LINE FOLLOW");
    return;
  }

  if(left && !right) {
    stopMotors();
    delay(80);
    resetPID();
    currentState = LINE_FOLLOW;
    Serial.println("LEFT SENSOR -> LINE FOLLOW");
    return;
  }

  setMotors(10,10);

  if(millis()-stateStartTime > DEBUG_SEARCH_TIMEOUT) {
    stopMotors();
    currentState = DEBUG_STOP;
    Serial.println("LINE SEARCH FAILED");
  }
}

void handleDebugAlignLine() {
  bool left  = digitalRead(PIN_IR_LEFT) == LOW;
  bool right = digitalRead(PIN_IR_RIGHT) == LOW;

  if(left && right) {
    stopMotors();
    delay(30);
    resetPID();
    obstacleCheckInterval = 50;
    currentState = LINE_FOLLOW;
    Serial.println("ALIGNED -> LINE FOLLOW");
    return;
  }

  if(left && !right) {
    setMotorsTurn(25,-25);
    return;
  }

  if(right && !left) {
    setMotorsTurn(-25,25);
    return;
  }

  if(millis()-alignStartTime > 800) {
    resetPID();
    currentState = LINE_FOLLOW;
    Serial.println("ALIGN TIMEOUT -> LINE FOLLOW");
  }
}

void handleDebugStop() {
  stopMotors();
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_IR_LEFT,  INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  pinMode(PIN_TRIG_LEFT,  OUTPUT);
  pinMode(PIN_ECHO_LEFT,  INPUT);
  pinMode(PIN_TRIG_RIGHT, OUTPUT);
  pinMode(PIN_ECHO_RIGHT, INPUT);

  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

  pidLastTime = millis();
  Serial.println("Debug Sequence Ready!");
}

void loop() {
  if (currentState == LINE_FOLLOW) {
    updateObstacleCheck();
  }

  switch (currentState) {
    case LINE_FOLLOW:      handleLineFollow();      break;
    case OBSTACLE_STOP:    handleObstacleStop();    break;
    case DEBUG_TURN_LEFT:  handleDebugTurnLeft();   break;
    case DEBUG_FWD:        handleDebugFwd();        break;
    case DEBUG_PAUSE:      handleDebugPause();      break;
    case DEBUG_TURN_RIGHT: handleDebugTurnRight();  break;
    case DEBUG_FIND_LINE:  handleDebugFindLine();   break; 
    case DEBUG_ALIGN_LINE: handleDebugAlignLine(); break;
    case DEBUG_STOP:       handleDebugStop();       break;
  }
}
