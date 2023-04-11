#include <Wire.h>
#include <Servo.h>

// taking ch values from receiver
int rc_pulse1;
int rc_pulse2;
int rc_pulse3;
int rc_pulse4;

// defining servo motor for drone 
Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;

// initializing pid values
int roll_PID = 0;
int pitch_PID = 0;

// setting buffer values for ch3
int rc_pulse3_min = 1185;

void setup() {

  Serial.println("script started");
  
  pinMode(5,INPUT); // ch1
  pinMode(2,INPUT); // ch2
  pinMode(3,INPUT); // ch3
  pinMode(4,INPUT); // ch4

  L_F_prop.attach(11); //left front motor
  L_B_prop.attach(6); //left back motor
  R_F_prop.attach(12); //right front motor 
  R_B_prop.attach(10); //right back motor 
  /*in order to make sure that the ESCs won't enter into config mode
  *I send a 1000us pulse to each ESC.*/
  L_F_prop.writeMicroseconds(1000); 
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000); 
  R_B_prop.writeMicroseconds(1000);
  
  Serial.begin(9600);

  Wire.begin(); // begin the wire comunication

  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission

  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Serial.println("gyro 1");
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (100dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro

  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);  

  delay(1000);
  time = millis(); //Start counting time in milliseconds
}

void loop() {
  delay(10);
  rc_pulse1 = pulseIn(5,HIGH,25000);
  delay(10);
  rc_pulse2 = pulseIn(2,HIGH,25000);
  delay(10);
  rc_pulse3 = pulseIn(3,HIGH,25000) - rc_pulse3_min;
  delay(10);
  rc_pulse4 = pulseIn(4,HIGH,25000);
  delay(10);

  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;    

  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers

  pwm_R_F  = input_THROTTLE - roll_PID - pitch_PID;
  pwm_R_B  = input_THROTTLE - roll_PID + pitch_PID;
  pwm_L_B  = input_THROTTLE + roll_PID + pitch_PID;
  pwm_L_F  = input_THROTTLE + roll_PID - pitch_PID;

  if(pwm_R_F < 1100)
  {
    pwm_R_F= 1100;
  }
  if(pwm_R_F > 2000)
  {
    pwm_R_F=2000;
  }
  
  //Left front
  if(pwm_L_F < 1100)
  {
    pwm_L_F= 1100;
  }
  if(pwm_L_F > 2000)
  {
    pwm_L_F=2000;
  }
  
  //Right back
  if(pwm_R_B < 1100)
  {
    pwm_R_B= 1100;
  }
  if(pwm_R_B > 2000)
  {
    pwm_R_B=2000;
  }
  
  //Left back
  if(pwm_L_B < 1100)
  {
    pwm_L_B= 1100;
  }
  if(pwm_L_B > 2000)
  {
    pwm_L_B=2000;
  }

  Serial.print("RF: ");
  Serial.print(pwm_R_F);
  Serial.print("  ");
  Serial.print("RB: ");
  Serial.print(pwm_R_B);
  Serial.print("  ");
  Serial.print("LB: ");
  Serial.print(pwm_L_B);
  Serial.print(" ");
  Serial.print("L_F: ");
  Serial.print(pwm_L_F);
  Serial.println(" ");

  L_F_prop.writeMicroseconds(pwm_L_F); 
  L_B_prop.writeMicroseconds(pwm_L_B);
  R_F_prop.writeMicroseconds(pwm_R_F); 
  R_B_prop.writeMicroseconds(pwm_R_B);
}
