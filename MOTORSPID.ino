#include <Arduino.h>
#include <Encoder.h>
//PID PARAMETERS FOR MOTOR A
float kp = 10;
float ki = 1;
float kd = 5;
//PID PARAMETERS FOR MOTOR B
float kp2 = 10;
float ki2 = 1;
float kd2 = 5;
unsigned long t;
unsigned long t_prev = 0;
const byte interruptPinA = 8;
const byte interruptPinB = 9;
const byte interruptPinA1 = 10;
const byte interruptPinB1 = 11;
volatile long EncoderCount1 = 0; // Encoder count for motor 1
volatile long EncoderCount2 = 0; // Encoder count for motor 2
const byte PWMPin1 = 2; // PWM pin for motor 1
const byte PWMPin2 = 5; // PWM pin for motor 2
const byte DirPin1_1 = 3; // Direction pin 1 for motor 1
const byte DirPin2_1 = 4; // Direction pin 2 for motor 1
const byte DirPin1_2 = 7; // Direction pin 1 for motor 2
const byte DirPin2_2 = 6; // Direction pin 2 for motor 2
volatile unsigned long count = 0;
unsigned long count_prev = 0;
float Theta1, RPM1, RPM_d1; // Variables for motor 1
float Theta2, RPM2, RPM_d2; // Variables for motor 2
float Theta_prev1 = 0;
float Theta_prev2 = 0;
int dt;
float RPM_max = 130;

#define pi 3.1416
float Vmax = 12;
float Vmin = -12;
float V1 = 0.1; // Voltage for motor 1
float V2 = 0.1; // Voltage for motor 2
float e1, e_prev1 = 0, inte1, inte_prev1 = 0; // Variables for PID controller of
motor 1
float e2, e_prev2 = 0, inte2, inte_prev2 = 0; // Variables for PID controller of
motor 2
Encoder MotorAencoder(interruptPinA, interruptPinB);
Encoder MotorBencoder(interruptPinA1, interruptPinB1);
///////////////////////INTRUPPT/////////////////////
void ISR_EncoderA() {
bool PinB = digitalRead(interruptPinB);
bool PinA = digitalRead(interruptPinA);
if (PinB == LOW) {
if (PinA == HIGH) {
EncoderCount1++;
}
else {
EncoderCount1--;
}
}
else {
if (PinA == HIGH) {
EncoderCount1--;
}
else {
EncoderCount1++;
}
}
}
void ISR_EncoderB() {
bool PinB = digitalRead(interruptPinA);
bool PinA = digitalRead(interruptPinB);
if (PinA == LOW) {
if (PinB == HIGH) {

EncoderCount2--;
}
else {
EncoderCount2++;
}
}
else {
if (PinB == HIGH) {
EncoderCount2++;
}
else {
EncoderCount2--;
}
}
}
////////////////////////////////////////////////////////////////////////////////
float sign(float x) {
if (x >= 0) {
return 1;
} else if (x < 0) {
return -1;
} else {
return 0;
}
}
//Motor Driver Functions****
void WriteDriverVoltage(float V, float Vmax, int motor) {
int PWMval = int(255 * abs(V) / Vmax);
if (PWMval > 255) {
PWMval = 255;
}
if (V >= 0) {
if (motor == 1) {
digitalWrite(DirPin1_1, HIGH);
digitalWrite(DirPin2_1, LOW);
} else if (motor == 2) {
digitalWrite(DirPin1_2, HIGH);
digitalWrite(DirPin2_2, LOW);
}

}
else {
if (motor == 1) {
digitalWrite(DirPin1_1, HIGH);
digitalWrite(DirPin2_1, LOW);
} else if (motor == 2) {
digitalWrite(DirPin1_2, HIGH);
digitalWrite(DirPin2_2, LOW);
}
}
if (motor == 1) {
analogWrite(PWMPin1, PWMval);
} else if (motor == 2) {
analogWrite(PWMPin2, PWMval);
}
}
void setup() {
Serial.begin(115200);
/////////////////Target//////////////////////////////
RPM_d1 = 100; // Set the target speed in RPM for motor 1
RPM_d2 = 100; // Set the target speed in RPM for motor 2
/////////////////Target//////////////////////////////
pinMode(interruptPinA, INPUT_PULLUP);
pinMode(interruptPinB, INPUT_PULLUP);
pinMode(interruptPinA1, INPUT_PULLUP);
pinMode(interruptPinB1, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
pinMode(DirPin1_1, OUTPUT);
pinMode(DirPin2_1, OUTPUT);
pinMode(DirPin1_2, OUTPUT);
pinMode(DirPin2_2, OUTPUT);
pinMode (PWMPin1 , OUTPUT);
pinMode (PWMPin2 , OUTPUT);
cli();
TCCR1A = 0;

TCCR1B = 0;
TCNT1 = 0;
OCR1A = 12499; //Prescaler = 64
TCCR1B |= (1 << WGM12);
TCCR1B |= (1 << CS11 | 1 << CS10);
TIMSK1 |= (1 << OCIE1A);
sei();
}
void loop() {
if (count > count_prev) {
t = millis();
Theta1 =MotorAencoder.read() ;
Theta2 =MotorBencoder.read() ;
dt = (t - t_prev);
RPM_d1 = 100; // Set the target speed in RPM for motor 1
RPM_d2 = 100; // Set the target speed in RPM for motor 2
if (t / 1000.0 > 100) {
RPM_d1 = 0;
RPM_d2 = 0;
}
RPM1 = ((Theta1-Theta_prev1) / 224) * (1000 / dt)* 60 ;
RPM2 = ((Theta2-Theta_prev2) / 224) * (1000 / dt) * 60;
e1 = RPM_d1 - RPM1;
e2 = RPM_d2 - RPM2;
inte1 = inte_prev1 + (dt * (e1 + e_prev1) / 2);
inte2 = inte_prev2 + (dt * (e2 + e_prev2) / 2);
V1 = kp * e1 + ki * inte1 + (kd * (e1 - e_prev1) / dt);
V2 = kp2 * e2 + ki2 * inte2 + (kd2 * (e2 - e_prev2) / dt);
if (V1 > RPM_d1) {
V1 = RPM_d1;
inte1 = inte_prev1;
}
if (V1 < RPM_d1) {
V1 = RPM_d1;
inte1 = inte_prev1;
}
if (V2 > RPM_d2) {
V2 = RPM_d2;
inte2 = inte_prev2;
}
if (V2 < Vmin) {

V2 = Vmin;
inte2 = inte_prev2;
}
WriteDriverVoltage(V1, RPM_d1, 1);
WriteDriverVoltage(V2, RPM_d2, 2);
Serial.print("Motor 1 - Target RPM: "); Serial.print(RPM_d1);
Serial.print("\t");
Serial.print("Current RPM: "); Serial.print(RPM1); Serial.print("\t");
Serial.print("Voltage: "); Serial.print(V1); Serial.print("\t");
Serial.print("Error: "); Serial.println(e1);
Serial.print("Motor 2 - Target RPM: "); Serial.print(RPM_d2);
Serial.print("\t");
Serial.print("Current RPM: "); Serial.print(RPM2); Serial.print("\t");
Serial.print("Voltage: "); Serial.print(V2); Serial.print("\t");
Serial.print("Error: "); Serial.println(e2);
////////////////////////////////////////////PLOT
Serial.println(RPM1);
Serial.print(",");
Serial.println(RPM2);
///////////////////////////////////////////
Theta_prev1 = Theta1;
Theta_prev2 = Theta2;
count_prev = count;
t_prev = t;
inte_prev1 = inte1;
inte_prev2 = inte2;
e_prev1 = e1;
e_prev2 = e2;
}
}
ISR(TIMER1_COMPA_vect) {
count++;
}