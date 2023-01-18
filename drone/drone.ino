#include <Servo.h>
#include "Wire.h"
#include <MPU6050.h>

MPU6050 mpu;

#define potentiometerPin A0
int servoPin1 = 6; // signal pin for the ESC.
int servoPin2 = 12;
int servoPin3 = 10;
int servoPin4 = 11;

int potVal = 0;
char Incoming_value = 0;
// PID constants
int kp = 1.3;
int ki = 0.05;
int kd = 0.3;

// defining servo motor names
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Defining throttle
int throttle = 0;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

float pitch_offset = -7.1;
float roll_offet = -23.4;
float yaw_offset = 61.5;

void setup() {
  Serial.begin(9600);
  
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);

  potVal = 1100;
  
  servo1.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo2.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo3.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo4.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  
  delay(5000); // delay to allow the ESC to recognize the stopped signal.
  Wire.begin();

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_8G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();

}

void print_val()
{
    Serial.print("In val = "); 
    Serial.print(Incoming_value);
    Serial.print("|");
    //Serial.print("Motor speed = "); 
    Serial.print(throttle);
     Serial.print("%"); 
    Serial.print("|");
   // Serial.print("left val = ");
     Serial.print(" cm");
    Serial.print("|");
    //Serial.print("right val = ");
    Serial.print(" cm");
     Serial.print("|");
    //Serial.print("distance = ");
    Serial.print(" cm");
    Serial.print("|");
    Serial.print(" cm");
     Serial.print("|");
    Serial.print(" cm/h");
    Serial.println(" ");
}

void ble(){
  if(Serial.available() > 0) {
    Incoming_value = Serial.read();
    if(Incoming_value == '4')             
      increase_throttle();
        
        
    else if(Incoming_value == '5')       
      decrease_throttle(); 
  }
  print_val();
  drone_power();
  }
/*void read_mpu() {

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
   
   if(pitch != 0 or roll != 0)
   //pid_controller();
}

void pid_controller() {
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    //int throttle = pulse_length[mode_mapping[THROTTLE]];

    if (pitch >= 0) {
      pid_temp_error = pitch;
      pitch_prop = pitch_temp_error;
      pitch_inte = pitch_inte + pitch_temp_error*ki;
      pitch_deri = (pitch_temp_error - pitch_previous_error);
      throttle = kp*pitch_prop + pitch_inte + kd*pitch_deri;
      pitch_previous_error = pitch_temp_error;
      servo1.writeMicroseconds(throttle);
    }
    else if (pitch <= 0) {
      pid_temp_error = -pitch;
      pitch_prop = pitch_temp_error;
      pitch_inte = pitch_inte + pitch_temp_error;
      pitch_deri = (pitch_temp_error - pitch_previous_error);
      throttle = kp*pitch_prop + ki*pitch_inte + kd*pitch_deri;
      pitch_previous_error = pitch_temp_error;
      servo2.writeMicroseconds(throttle);
    }
    else if (roll >= 0) {
      pid_temp_error = -roll;
      roll_prop = roll_temp_error;
      roll_inte = roll_inte + roll_temp_error;
      roll_deri = (roll_temp_error - roll_previous_error);
      throttle = kp*roll_prop + ki*roll_inte + kd*roll_deri;
      roll_previous_error = roll_temp_error;
      servo3.writeMicroseconds(throttle);
    }
    else if (roll <= 0) {
      pid_temp_error = -roll;
      roll_prop = roll_temp_error;
      roll_inte = roll_inte + roll_temp_error;
      roll_deri = (roll_temp_error - roll_previous_error);
      throttle = kp*roll_prop + ki*roll_inte + kd*roll_deri;
      roll_previous_error = roll_temp_error;
      servo4.writeMicroseconds(throttle);
    }
}*/

void drone_power() {
  Serial.println(throttle);
  servo1.writeMicroseconds(throttle); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo2.writeMicroseconds(throttle); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo3.writeMicroseconds(throttle); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo4.writeMicroseconds(throttle); // send "stop" signal to ESC. Also necessary to arm the ESC.
}

void increase_throttle() {
  if (throttle <= 1500) {
    throttle += 100;
  }
  drone_power();
}

void decrease_throttle() {
  if(throttle >= 0) {
    throttle -= 100;
  }
  drone_power();
}

void loop() {

//int potVal = analogRead(potentiometerPin); // read input from potentiometer.

// int pwmVal = map(potVal,0, 1023, 1100, 2000); // maps potentiometer values to PWM value.
// Serial.println(pwmVal);
// servo1.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
// servo2.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
// servo3.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.
// servo4.writeMicroseconds(pwmVal); // send "stop" signal to ESC. Also necessary to arm the ESC.

// taking the values of pitch, yaw and roll form the help of MPU6050 gyro sensor
//read_mpu()

ble(); 
// potVal = 0
}
