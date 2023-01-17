#include <Servo.h>
#include "Wire.h"
#include <MPU6050.h>

MPU6050 mpu;

#define potentiometerPin A0
int servoPin1 = 6; // signal pin for the ESC.
int servoPin2 = 12;
int servoPin3 = 10;
int servoPin4 = 11;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

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

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_8G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();

}

void read_mpu() {

    // Read normalized values
    Vector norm = mpu.readNormalizeGyro();
  
    // Calculate Pitch, Roll and Yaw
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;
  
    // Output raw
    Serial.print(" Pitch = ");Serial.print(pitch);
    Serial.print(" Roll = ");Serial.print(roll);
    Serial.print(" Yaw = ");Serial.println(yaw);

    // Wait to full timeStep period
    delay((timeStep*1000) - (millis() - timer));
   }
}

void pid_controller() {
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = pulse_length[mode_mapping[THROTTLE]];
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
