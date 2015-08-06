// Laser proximity application based on ROS (rosserial)
// Angelos Plastropoulos
//
// System set-up
// ROS Indigo on Ubuntu 32bit 14.04LTS
// Arduino Uno (RedBoard by SparkFun)
//
// It utilizes the 'Arduino I2C Master Library' from DSS Circuits:
// http://www.dsscircuits.com/index.php/articles/66-arduino-i2c-master-library 
//
// Parts of the code based on the PulsedLight3d online repositories
// https://github.com/PulsedLight3D/LIDARLite_Basics


#include <ros.h>
#include <std_msgs/Float32.h>

#include <Servo.h>
#include <Time.h>  // usefull to measure time durations
#include <I2C.h>   // DSS I2S library

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.
#define    CalibrationRegister     0x13          // The register to set for calibration
#define    CalibrationOffsetVlue   0xFE          // The calibration offset... see note below. 

Servo servo1;
int low_us = 555;
int high_us = 2390;
int mid_us = 1500;

std_msgs::Float32 dist0_msg, dist90_msg, distm90_msg;
ros::Publisher pub_dist1("dist0", &dist0_msg);
ros::Publisher pub_dist2("dist90", &dist90_msg);
ros::Publisher pub_dist3("distm90", &distm90_msg);
ros::NodeHandle nh;

void setup() {

 int index;
 Serial.begin(57600);      
 I2c.begin(); // Opens & joins the irc bus as master
 delay(100); // Waits to make sure everything is powered up before sending or receiving data  
 I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
 LidarLiteCalibrate(); // Laser calibration (zeroing)
   
 servo1.attach(9);

 nh.initNode(); //instantiate the node handle
 nh.advertise(pub_dist1); // Declare the topics publishers
 nh.advertise(pub_dist2);
 nh.advertise(pub_dist3);
 
}

void loop() {
 
 int dist, dist1, dist2; 
 time_t startT, endT;
 float period;
 
 //dist = getDistance();
 //Serial.println(dist);
 //delay(200);

 startT = now();
 
 // move to 0
 moveTheServo(degrees2ms(0));

 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;
  
 dist90_msg.data = dist;
 pub_dist2.publish(&dist90_msg);
 nh.spinOnce();

 // move to 90
 moveTheServo(degrees2ms(90));

 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;
 
 dist = getDistance();
 dist0_msg.data = dist;
 pub_dist1.publish(&dist0_msg);
 nh.spinOnce();

 // move to 180
 moveTheServo(degrees2ms(180));

 // get two measurements and report the average 
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;
 
 dist = getDistance();
 distm90_msg.data = dist;
 pub_dist3.publish(&distm90_msg);
 nh.spinOnce();

 // calculate the period and print to serial terminal
 endT = now();
 period = second(endT) - second(startT);
 Serial.println(period);

 // move to 90
 moveTheServo(degrees2ms(90));

 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist = (dist1+dist2)/2;
 
 dist = getDistance();
 dist0_msg.data = dist;
 pub_dist1.publish(&dist0_msg);
 nh.spinOnce();
 
}

// convert angle in degrees to pulse width in microseconds 
int degrees2ms(int deg) {
  if(deg <= 0) {
    return low_us;
  } // end if
  
  if(deg >= 180) {
    return high_us;
  } // end if
  
  if(deg==90) {
    return mid_us;
  }
  
  if(deg<90) {
    double uS_val = (double(mid_us) - double(low_us)) / 90.000;
    return int((uS_val * double(deg)) + double(low_us));
  }
  
  if(deg>90) {
    double uS_val = (double(high_us) - double(mid_us)) / 90.000;
    deg = deg-90;
    return int((uS_val * double(deg)) + double(mid_us));
  }
} 

// move servo using pulse width in microseconds
void moveTheServo(int uS) {
  servo1.attach(9);

  if(uS >= 555 && uS <= 2400) {
    servo1.writeMicroseconds(uS);
    delay(600);
  } // end if
  
  servo1.detach();
} 

// zeroing the LIDARLite
void LidarLiteCalibrate(){
 // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,CalibrationRegister, CalibrationOffsetVlue); // Write Calibration Offset Value to 0x13
    delay(1); // Wait 1 ms to prevent overpolling
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
