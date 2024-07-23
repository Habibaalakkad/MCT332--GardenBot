// Define the motor right control pins
#define encoderPin1 49
#define encoderPin2 48
#define PWMPin 6
#define in1Pin 53
#define in2Pin 52

// Define the motor left control pins
#define encoderPin1_left 47
#define encoderPin2_left 46
#define PWMPin_left 7
#define in1Pin_left 50
#define in2Pin_left 51

int proportional = 1;
float kpv = 30;
float kiv = 0.1;

int proportional_left = 1;
float kpv_left = 10;
float kiv_left = 0.005;

volatile float motorPosition = 0;
volatile float motorPosition_left = 0;
float previousTime = 0;
float previousTime_left = 0;
float errorIntegral = 0;
float errorIntegral_left = 0;
float velocity = 0;
float velocity_left = 0;
float pos = 0;
float pos_left = 0;
float errorvelocity = 0;
float errorvelocity_left = 0;
float controlSignal = 0;
float controlSignal_left = 0;
float controlsignalvelocity = 0;
float controlsignalvelocity_left = 0;
float target_pos_cm = 140;
float target_pos_cm_left = 140;
float prev_target_pos_cm = 0;
float prev_target_pos_cm_left = 0;
float previousError = 0;
float derivative = 0;
float previousError_left = 0;

// Define the ultrasonic sensor pins
#define trigPin_front1 8
#define echoPin_front1 9

#define trigPin_front2 44
#define echoPin_front2 45

#define trigPin_right 42
#define echoPin_right 43

#define trigPin_left 40
#define echoPin_left 41

long duration_front1, distance_front1;
long duration_front2, distance_front2;
long duration_right, distance_right;
long duration_left, distance_left;

unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 50; // Sensor read interval in milliseconds

void setup() {
  Serial.begin(115200);

  // Motor right setup
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(PWMPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), checkEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), checkEncoder2, RISING);

  // Motor left setup
  pinMode(encoderPin1_left, INPUT_PULLUP);
  pinMode(encoderPin2_left, INPUT_PULLUP);
  pinMode(PWMPin_left, OUTPUT);
  pinMode(in1Pin_left, OUTPUT);
  pinMode(in2Pin_left, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1_left), checkEncoder1_left, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2_left), checkEncoder2_left, RISING);

  pinMode(trigPin_front1, OUTPUT);
  pinMode(echoPin_front1, INPUT);

  pinMode(trigPin_front2, OUTPUT);
  pinMode(echoPin_front2, INPUT);

  pinMode(trigPin_right, OUTPUT);
  pinMode(echoPin_right, INPUT);

  pinMode(trigPin_left, OUTPUT);
  pinMode(echoPin_left, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorReadTime >= sensorReadInterval) {
    readSensors();
    lastSensorReadTime = currentMillis;
  }

  if (distance_front1 <= 30 || distance_front2 <= 30) {
    stopMotors();
    delay(2000);
    if (distance_front1 <= 30 && distance_front2 <= 30 && distance_right > 20 && distance_left > 20) {
      moveMotors(-40, -20);
      delay(2000);
      moveMotors(15, 0); // Turn left
      delay(1000);
    }
    else if(distance_front1 <= 30 && distance_front2 <= 30 && distance_right <= 20 && distance_left > 20){
      moveMotors(15, 0); // Turn left 
      delay(1150);
    }
    else if(distance_front1 <= 30 && distance_front2 <= 30 && distance_right > 20 && distance_left <= 20){
     moveMotors(0, 15); // Turn left
     delay(1150);
    }

    else{
       moveMotors(50,20);
    }
  }
  else if (distance_left <= 20) {
      moveMotors(-50, 10); // Turn right
  } else if (distance_right <= 20 ) {
      moveMotors(50,-20); 
    }
   else {
    moveMotors(50,20); // Move forward
  }
}

void readSensors() {
  distance_front1 = getDistance(trigPin_front1, echoPin_front1);
  distance_front2 = getDistance(trigPin_front2, echoPin_front2);
  distance_right = getDistance(trigPin_right, echoPin_right);
  distance_left = getDistance(trigPin_left, echoPin_left);
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (pulseIn(echoPin, HIGH) * 0.0343) / 2;
}

void moveMotors(int desiredvelocity, int desiredvelocity_left) {
  PID(desiredvelocity, desiredvelocity_left);
}

void stopMotors() {
  PID(0, 0);
}

void PID(int desiredvelocity, int desiredvelocity_left) {
  float currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1.0e6; // Convert to seconds
  previousTime = currentTime;

  float currentTime_left = micros();
  float deltaTime_left = (currentTime_left - previousTime_left) / 1.0e6; // Convert to seconds
  previousTime_left = currentTime_left;

  noInterrupts(); // disable interrupts temporarily while reading
  velocity = (motorPosition - pos) / deltaTime; // Corrected calculation
  velocity_left = (motorPosition_left - pos_left) / deltaTime_left; // Corrected calculation
  pos = motorPosition;
  pos_left = motorPosition_left;
  interrupts(); // turn interrupts back on

  float desiredpos = target_pos_cm;
  controlSignal = (desiredpos - pos) * proportional;

  float desiredpos_left = target_pos_cm_left;
  controlSignal_left = (desiredpos_left - pos_left) * proportional_left;

  // Limit control signals to desired velocity
  controlSignal = constrain(controlSignal, -desiredvelocity, desiredvelocity);
  controlSignal_left = constrain(controlSignal_left, -desiredvelocity_left, desiredvelocity_left);

  // Calculate errors and PID terms
  errorvelocity = desiredvelocity - velocity;
  errorIntegral += errorvelocity * deltaTime;
  derivative = (errorvelocity - previousError) / deltaTime;
  previousError = errorvelocity;

  errorvelocity_left = desiredvelocity_left - velocity_left;
  errorIntegral_left += errorvelocity_left * deltaTime_left;
  float derivative_left = (errorvelocity_left - previousError_left) / deltaTime_left; // Declare derivative_left locally
  previousError_left = errorvelocity_left;

  // Calculate control signals with PID terms
  controlsignalvelocity = kpv * errorvelocity + kiv * errorIntegral; // Added kdv term
  controlsignalvelocity_left = kpv_left * errorvelocity_left + kiv_left * errorIntegral_left; // Used local derivative_left

  // Convert control signals to PWM values
  int motorDirection = (controlsignalvelocity >= 0) ? 1 : -1;
  int motorDirection_left = (controlsignalvelocity_left >= 0) ? 1 : -1;
  float PWMValue = constrain(fabs(controlsignalvelocity), 0, 255);
  float PWMValue_left = constrain(fabs(controlsignalvelocity_left), 0, 255);

  // Set motor directions and PWM values
  analogWrite(PWMPin, PWMValue);
  analogWrite(PWMPin_left, PWMValue_left);
  digitalWrite(in1Pin, motorDirection == 1 ? HIGH : LOW);
  digitalWrite(in2Pin, motorDirection == -1 ? HIGH : LOW);
  digitalWrite(in1Pin_left, motorDirection_left == 1 ? HIGH : LOW);
  digitalWrite(in2Pin_left, motorDirection_left == -1 ? HIGH : LOW);
}

void checkEncoder1() {
  motorPosition++;
}

void checkEncoder2() {
  motorPosition--;
}

void checkEncoder1_left() {
  motorPosition_left++;
}

void checkEncoder2_left() {
  motorPosition_left--;
}