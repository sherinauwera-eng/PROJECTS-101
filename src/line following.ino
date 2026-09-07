
// --- PIN ASSIGNMENTS ---

// IR Sensors 
int leftSensor = A0;
int rightSensor = A1;

// L298N Motor Driver Pins
int ENA = 8; // Left motor speed
int IN1 = 7; // Left motor direction
int IN2 = 6; // Left motor direction

int ENB = 5; // Right motor speed
int IN3 = 3; // Right motor direction
int IN4 = 2; // Right motor direction

// --- TUNING VARIABLES ---
int baseSpeed = 100;      
int softTurnSpeed = 60;   
int sharpTurnSpeed = 0;   

// Memory: 0 = straight, 1 = left, 2 = right
int lastTurn = 0; 

void setup()
{
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  followLine();
}

void followLine()
{
  int leftValue = digitalRead(leftSensor);
  int rightValue = digitalRead(rightSensor);

  // 1. CENTERED OR LOST (White / White)
  if(leftValue == 0 && rightValue == 0)
  {
    if (lastTurn == 1) {
      sharpLeft(); // Emergency recovery: lost to the left
    } 
    else if (lastTurn == 2) {
      sharpRight(); // Emergency recovery: lost to the right
    } 
    else {
      moveForward(); // Perfectly centered
    }
  }

  // 2. DRIFTING RIGHT (Black / White)
  else if(leftValue == 1 && rightValue == 0)
  {
    lastTurn = 1; 
    softLeft(); // Smooth correction, keep moving forward
  }

  // 3. DRIFTING LEFT (White / Black)
  else if(leftValue == 0 && rightValue == 1)
  {
    lastTurn = 2; 
    softRight(); // Smooth correction, keep moving forward
  }

  // 4. THE 4cm TRAP (Black / Black)
  else if(leftValue == 1 && rightValue == 1)
  {
    lastTurn = 0;  // Reset memory so it doesn't spin wildly after
    moveForward(); // Blast straight through it
  }
}

// --- MOTOR CONTROL FUNCTIONS ---

void moveForward() {
  analogWrite(ENA, baseSpeed);
  analogWrite(ENB, baseSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void softLeft() {
  // Both wheels go forward, but left wheel slows down
  analogWrite(ENA, softTurnSpeed); 
  analogWrite(ENB, baseSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void softRight() {
  // Both wheels go forward, but right wheel slows down
  analogWrite(ENA, baseSpeed); 
  analogWrite(ENB, softTurnSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void sharpLeft() {
  // Left wheel stops completely, right wheel pushes hard
  analogWrite(ENA, sharpTurnSpeed); 
  analogWrite(ENB, baseSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void sharpRight() {
  // Right wheel stops completely, left wheel pushes hard
  analogWrite(ENA, baseSpeed); 
  analogWrite(ENB, sharpTurnSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
