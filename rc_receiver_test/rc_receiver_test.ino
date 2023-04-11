int rc_pulse1;
int rc_pulse2;
int rc_pulse3;
int rc_pulse4;

void setup() {
  pinMode(5,INPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  Serial.begin(9600);
}

void loop() {
  delay(10);
  rc_pulse1 = pulseIn(5,HIGH,25000);
  delay(10);
  rc_pulse2 = pulseIn(2,HIGH,25000);
  delay(10);
  rc_pulse3 = pulseIn(3,HIGH,25000);
  delay(10);
  rc_pulse4 = pulseIn(4,HIGH,25000);
  delay(10);
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
