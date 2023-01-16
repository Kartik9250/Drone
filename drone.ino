#include <Servo.h>

#define potentiometerPin A0
int servoPin1 = 6; // signal pin for the ESC.
int servoPin2 = 12;
int servoPin3 = 10;
int servoPin4 = 11;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
Serial.begin(9600);
servo1.attach(servoPin1);
servo2.attach(servoPin2);
servo3.attach(servoPin3);
servo4.attach(servoPin4);
servo1.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo2.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo3.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo4.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.

delay(5000); // delay to allow the ESC to recognize the stopped signal.
}

void loop() {

int potVal = analogRead(potentiometerPin); // read input from potentiometer.

int pwmVal = map(potVal,0, 1023, 1100, 2000); // maps potentiometer values to PWM value.
Serial.println(pwmVal);
servo1.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo2.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo3.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo4.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
}
