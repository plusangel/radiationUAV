// Laser obstacle detection application based on ROS (rosserial)
// Angelos Plastropoulos
//
// The functions which handle the LIDAR-Lite measurements taken from 
// PulsedLight3D LIDARLite_Basics online git repository
//
// and from renuncln.com blog guides for the servo callibration

#include <ros.h>
#include <std_msgs/Float32.h>

#include <Servo.h>
#include <I2C.h>            // DSS I2S library for Laser sensor
#include <SoftwareSerial.h> // Sonar

// sonar related info
SoftwareSerial mySerial(10, 11);  // RX, TX

// servo related info
Servo servo1;
int low = 555;
int high = 2390;
int mid = 1500;

// Lidar-lite addresses
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

std_msgs::Float32 dist0_msg, dist90_msg, distm90_msg, sonar_msg;
ros::Publisher pub_dist1("dist0", &dist0_msg);
ros::Publisher pub_dist2("dist90", &dist90_msg);
ros::Publisher pub_dist3("distm90", &distm90_msg);
ros::Publisher pub_groundDistance("sonar", &sonar_msg);
ros::NodeHandle nh;

void setup() {

 Serial.begin(9600);
 mySerial.begin(9600);
 
 I2c.begin(); // Opens & joins the irc bus as master
 delay(100); // Waits to make sure everything is powered up before sending or receiving data  
 I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
 
 servo1.attach(7);

 // set the custom baud rate other than the ROS default which is equal to 57600bps
 nh.getHardware()->setBaud(9600);
 nh.initNode();
 nh.advertise(pub_dist1);
 nh.advertise(pub_dist2);
 nh.advertise(pub_dist3);
 nh.advertise(pub_groundDistance);
 
}


void loop() {

 int dist, dist1, dist2;
 int groundDistance;
 
 // move to 0
 moveTheServo(degrees2ms(0), 700);
 
 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;

 groundDistance = sonarMeasurement();

 sonar_msg.data = groundDistance;
 dist90_msg.data = dist;
 pub_groundDistance.publish(&sonar_msg);
 pub_dist2.publish(&dist90_msg);
 nh.spinOnce();
 
 Serial.print("-90: "); Serial.println(dist); 
 
 // move to 90
 moveTheServo(degrees2ms(90), 400);

 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;

 groundDistance = sonarMeasurement();

 sonar_msg.data = groundDistance;
 dist0_msg.data = dist;
 pub_groundDistance.publish(&sonar_msg);
 pub_dist1.publish(&dist0_msg);
 nh.spinOnce();
 
 Serial.print("0: "); Serial.println(dist);
 
 // move to 180
 moveTheServo(degrees2ms(180), 400);
 
 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;

 groundDistance = sonarMeasurement();

 sonar_msg.data = groundDistance;
 distm90_msg.data = dist;
 pub_groundDistance.publish(&sonar_msg);
 pub_dist3.publish(&distm90_msg);
 nh.spinOnce();
 
 Serial.print("90: "); Serial.println(dist);
}

// convert angle in degrees to pulse width in microseconds 
int degrees2ms(int deg) {
  if(deg <= 0) {
    return low;
  } // end if
  
  if(deg >= 180) {
    return high;
  } // end if
  
  if(deg==90) {
    return mid;
  }
  
  if(deg<90) {
    double val = (double(mid) - double(low)) / 90.000;
    return int((val * double(deg)) + double(low));
  }
  
  if(deg>90) {
    double val = (double(high) - double(mid)) / 90.000;
    deg = deg-90;
    return int((val * double(deg)) + double(mid));
  }
} 

// move servo using pulse width in microseconds
void moveTheServo(int ms, int duration) {
  servo1.attach(7);

  if(ms >= 555 && ms <= 2390) {
    servo1.writeMicroseconds(ms);
    delay(duration);
  } // end if
  
  servo1.detach();
} 

// SRF02 ultrasonic sensor - helper method
void SendCmd(unsigned char address, unsigned char cmd)
{
  mySerial.write(address);      //set the address of SRF02(factory default is 0)
  delayMicroseconds(100);       //serial data is fixed at 9600 so we need some time to create the second stop bit
  
  mySerial.write(cmd);          //send the command to SRF02
  delayMicroseconds(100);       //serial data is fixed at 9600 so we need some time to create the second stop bit
}

// Get the measured distance from the SRF02 ultrasonic sensor
int sonarMeasurement() {
  int reading;
  SendCmd(0x00,0x51);        //Real Ranging Mode - Result in centimetres
  
  delay(70);                 //time for SRF02 to measure the range
  SendCmd(0x00,0x5E);        //Get Range, returns two bytes (high byte first) from the most recent ranging.
  
  delay(10);                 //wait for some time and let the Arduino receive 2 bytes data from the TX pin of SRF02
  
  if(mySerial.available()>=2)      //if two bytes were received
  {
    reading = mySerial.read()<<8;  //receive high byte (overwrites previous reading) and shift high byte to be high 8 bits
    reading |= mySerial.read();        // receive low byte as lower 8 bits
    // Serial.print(reading); 
    // Serial.println("cm");
    return reading;
  }
   
}

// Get the measured distance using the DSS I2C library 
int getDistance(){
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }

  return (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
}
