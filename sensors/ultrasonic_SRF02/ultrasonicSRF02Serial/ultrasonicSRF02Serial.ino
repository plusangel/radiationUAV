/*
Sample code to interface Devantech’s SRF02 with Arduino UNO
Serial communication
Tested on Arduino IDE 1.0.5


Parts of the code taken from the following webpage:
http://www.dfrobot.com/wiki/index.php/SRF02_Ultrasonic_sensor_(SKU:SEN0005)
*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);  // RX, TX 

void SendCmd(unsigned char address, unsigned char cmd)
{
  mySerial.write(address);      //set the address of SRF02(factory default is 0)
  delayMicroseconds(100);       //serial data is fixed at 9600 so we need some time to create the second stop bit
  
  mySerial.write(cmd);            //send the command to SRF02
  delayMicroseconds(100);       //serial data is fixed at 9600 so we need some time to create the second stop bit
}

void setup(void)
{
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("SRF02 measurements log");
}

void loop(void)
{
  unsigned int reading;
  SendCmd(0x00,0x51);            //Real Ranging Mode - Result in centimetres
  
  delay(70);                     //time for SRF02 to measure the range
  SendCmd(0x00,0x5E);        //Get Range, returns two bytes (high byte first) from the most recent ranging.
  
  delay(10);                 //wait for some time and let the Arduino receive 2 bytes data from the TX pin of SRF02
  
  if(mySerial.available()>=2)      //if two bytes were received
  {
    reading = mySerial.read()<<8;  //receive high byte (overwrites previous reading) and shift high byte to be high 8 bits
    reading |= mySerial.read();        // receive low byte as lower 8 bits
    Serial.print(reading); 
    Serial.println("cm");
  }
  
  delay(250); // wait to avoid terminal flood
}

