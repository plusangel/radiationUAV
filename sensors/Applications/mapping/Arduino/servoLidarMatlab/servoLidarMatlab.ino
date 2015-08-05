// Laser mapping application
// Tested on Arduino 1.6.5
//
// Parts of the code taken from the PulsedLight3D online repository
// https://github.com/PulsedLight3D/LIDARLite_Basics


#include <Servo.h>  // servo library
#include <I2C.h> //DSS I2C library

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.
#define    CalibrationRegister     0x13          // The register to set for calibration
#define    CalibrationOffsetVlue   0xFE          // The calibration offset... see note below. 

Servo servo1;  // servo control object

void setup() {
 
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  LidarLiteCalibrate();
  
  servo1.attach(9);
}

void LidarLiteCalibrate(){
 // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,CalibrationRegister, CalibrationOffsetVlue); // Write Calibration Offset Value to 0x13
    delay(1); // Wait 1 ms to prevent overpolling
  }
} 

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

void loop() {
  int position;
  int dist1, dist2, avgdist;
  int angle;

 // Tell servo to go to 180 degrees, stepping by two degrees
 for(position = 0; position < 180; position += 2)
  {
    servo1.write(position);  // Move to next position
    delay(20);// Short pause to allow it to move

    angle = servo1.read();
    Serial.println(angle);

    // measure distance twice and calculate the average
    dist1 = getDistance();
    dist2 = getDistance();
    
    avgdist = (dist1+dist2)/2;
    
    Serial.println(avgdist);
  }

  
  // Tell servo to go to 0 degrees, stepping by two degrees
  for(position = 180; position >= 0; position -= 2)
  {                 
    servo1.write(position);  // Move to next position
    delay(20);               // Short pause to allow it to move
    
    angle = servo1.read();
    Serial.println(angle);

    // measure distance twice and calculate the average
    dist1 = getDistance();
    dist2 = getDistance();
    
    avgdist = (dist1+dist2)/2;
    
    Serial.println(avgdist);
  } 
}
