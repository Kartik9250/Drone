#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>


MPU6050 mpu(Wire);

long timer = 0;

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
Wire.begin();

//mpu setup
byte status = mpu.begin();
Serial.print(F("MPU6050 status: "));
Serial.println(status);
while(status!=0){ } // stop everything if could not connect to MPU6050

Serial.println(F("Calculating offsets, do not move MPU6050"));
delay(1000);
mpu.calcOffsets(true,true); // gyro and accelero
Serial.println("Done!\n");

}


void loop() {

int potVal = analogRead(potentiometerPin); // read input from potentiometer.

int pwmVal = map(potVal,0, 1023, 1100, 2000); // maps potentiometer values to PWM value.
Serial.println(pwmVal);
servo1.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo2.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo3.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
servo4.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.

//mpu output
mpu.update();

if(millis() - timer > 1000){ // print data every second
  Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
  Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
  Serial.print("\tY: ");Serial.print(mpu.getAccY());
  Serial.print("\tZ: ");Serial.println(mpu.getAccZ());

  Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
  Serial.print("\tY: ");Serial.print(mpu.getGyroY());
  Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());

  Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
  Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
  
  Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
  Serial.print("\tY: ");Serial.print(mpu.getAngleY());
  Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
  Serial.println(F("=====================================================\n"));
  timer = millis();
}

}
