// The functions which handle the LIDAR-Lite measurements taken from 
// PulsedLight3D LIDARLite_Basics online git repository

#include <Servo.h>
#include <SPI.h>
#include <EEPROM.h>
#include <I2C.h>          // DSS I2S library
#include <boards.h>       // for BLE
#include <RBL_nRF8001.h>  // for BLE
#include "Boards.h"

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

// Unions for the iOS-BLE connection
typedef union {
 int integer;
 byte binary[2];
} binaryInt16;

typedef union {
 int integer;
 byte binary[4];
} binaryInt32;

void setup() {

 Serial.begin(57600);
 
 // initialize BLE with a name
 ble_begin();
 ble_set_name("My BLE");
 delay(100);
 // --
 
 I2c.begin(); // Opens & joins the irc bus as master
 delay(100); // Waits to make sure everything is powered up before sending or receiving data  
 I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
 
 servo1.attach(7);
}


void loop() {

 binaryInt16 dir0, dir90, dir180;
 binaryInt32 dist0, dist90, dist180;
 int dist1, dist2;
 
 // move to 0
 moveTheServo(degrees2ms(0), 700);
 
 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist0.integer = (dist1+dist2)/2;
 dir0.integer = 0;

 ble_write(dist0.binary[3]); ble_write(dist0.binary[2]); ble_write(dist0.binary[1]); ble_write(dist0.binary[0]);
 ble_write(dir0.binary[1]); ble_write(dir0.binary[0]);
 ble_do_events();
 
 Serial.print("0: "); Serial.println(dist0.integer); 
 
 // move to 90
 moveTheServo(degrees2ms(90), 400);

 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist90.integer = (dist1+dist2)/2;
 dir90.integer = 90;

 ble_write(dist90.binary[3]); ble_write(dist90.binary[2]); ble_write(dist90.binary[1]); ble_write(dist90.binary[0]);
 ble_write(dir90.binary[1]); ble_write(dir90.binary[0]);
 ble_do_events();
 Serial.print("90: "); Serial.println(dist90.integer);
 
 // move to 180
 moveTheServo(degrees2ms(180), 400);
 
 // get two measurements and report the average
 dist1 = getDistance();
 dist2 = getDistance();
 dist180.integer = (dist1+dist2)/2;
 dir180.integer = 180;
 
 ble_write(dist180.binary[3]); ble_write(dist180.binary[2]); ble_write(dist180.binary[1]); ble_write(dist180.binary[0]);
 ble_write(dir180.binary[1]); ble_write(dir180.binary[0]);
 ble_do_events();
 Serial.print("180: "); Serial.println(dist180.integer);
 
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
