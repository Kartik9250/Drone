int rc_pulse1;
int rc_pulse2;
int rc_pulse3;
int rc_pulse4;

void setup() {
  //taking inputs from reciever channels: 4,2,3,5
  pinMode(5,INPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  Serial.begin(9600);
}

void loop() {
  delay(10);
  //pulseIn waits for signal to go from low to high, with a timeout of 25000
  rc_pulse1 = pulseIn(5,HIGH,25000);
  delay(10);
  rc_pulse2 = pulseIn(2,HIGH,25000);
  delay(10);
  rc_pulse3 = pulseIn(3,HIGH,25000);
  delay(10);
  rc_pulse4 = pulseIn(4,HIGH,25000);
  delay(10);
  //printing the values to plot it on serial plotter
  Serial.print("rc_ch1:");
  Serial.print(rc_pulse1);
  Serial.print(",");
  Serial.print("rc_ch2:");
  Serial.print(rc_pulse2);
  Serial.print(",");
  Serial.print("rc_ch3:");
  Serial.print(rc_pulse3);
  Serial.print(",");
  Serial.print("rc_ch4:");
  Serial.println(rc_pulse4);
  delay(50);
}
