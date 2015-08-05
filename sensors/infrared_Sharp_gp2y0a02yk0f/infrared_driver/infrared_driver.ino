/*
Sample code to interface Sharp GP2Y0A02YK0F with Arduino UNO
Tested on Arduino IDE 1.0.5

*/

#define sensorIR 0         //analog pin
float v, cm;               //voltage and distance variable

void setup() {
  Serial.begin(9600);
}

void loop() {
  v = analogRead(sensorIR);
  // approximate solution calculated using http://www.wolframalpha.com computational knowledge engine
  cm = (2.7655*pow(10,8))/((50*pow((50*v+4323),0.593625))*v+4323*pow((50*v+4323),0.593625));
  
  delay(200);
  Serial.print("cm: ");
  Serial.println(cm);
}

