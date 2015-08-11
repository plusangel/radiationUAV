///////////////////////////////////////////////////////////
// The code was developed at Interface Analysis Centre
// (IAC) at Bristol University
//
// BLE Integration was implemented by Angelos Plastropoulos
// as part of the MSc in Robotics Dissertation
//
///////////////////////////////////////////////////////////

#include <hiduniversal.h>
#include <hidescriptorparser.h>
//for SD card
#include <SPI.h>
#include <SD.h>
//for gps
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "pgmstrings.h"  

//for BLE
#include <boards.h>
#include <RBL_nRF8001.h>

// Union for the iOS-BLE connection
typedef union {
 float floatingPoint;
 byte binary[4];
} binaryFloat;

//--

//Globals

String time=0;
String distance=0;
int exposure =500;
long timing =0;
int counts=0;
long tic;
long toc;
//int data[100];//max counts per exposure
int minChannel =50;
int totalcount =0;
long run =0;

//for SD card
File myFile;
String name;

//for gps
Adafruit_GPS GPS(&Serial2);// TX2, RX2  
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//for GR1
class HIDUniversal2 : 
public HIDUniversal
{
public:
  HIDUniversal2(USB *usb) : 
  HIDUniversal(usb) {
  };

protected:
  virtual uint8_t OnInitSuccessful();
};

uint8_t HIDUniversal2::OnInitSuccessful()
{
  uint8_t    rcode;

  HexDumper<USBReadParser, uint16_t, uint16_t>    Hex;
  ReportDescParser                                Rpt;

  if (rcode = GetReportDescr(0, &Hex))
    goto FailGetReportDescr1;

  if (rcode = GetReportDescr(0, &Rpt))
    goto FailGetReportDescr2;

  return 0;

FailGetReportDescr1:
  USBTRACE("GetReportDescr1:");
  goto Fail;

FailGetReportDescr2:
  USBTRACE("GetReportDescr2:");
  goto Fail;

Fail:
  Serial.println(rcode, HEX);
  Release();
  return rcode;
}

USB                                             Usb;
USB       Usb2;
//USBHub                                          Hub(&Usb);
HIDUniversal2                                   Hid(&Usb);
UniversalReportParser                           Uni;
uint8_t lens;
uint8_t *bufs;
ReportDescParser2 obj(lens, bufs);
ReportDescParserBase test;
HIDReportParser* rep;
HID* o;
uint8_t* newdata = new uint8_t[100];

USBReadParser*p;
void parse(uint8_t lens, uint8_t *bufs) {
  //   printf("You've passed me %d worth of data\n", len);
  uint8_t data[63];
  newdata = bufs;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup()
{
  // initialize BLE with a name
  ble_begin();
  ble_set_name("My BLE");
  // --
  
  Uni.set(parse);
  Serial.begin( 57600 );
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay( 200 );

  if (!Hid.SetReportParser(0, &Uni))
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  ); 
    
  //gps setup  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  useInterrupt(true);

  Serial.println("Initializing SD card...");
  pinMode(53, OUTPUT);//53

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  int newfile =0;
  int filecount = 0;
  char filename;
  //String name;

  while (newfile ==0){

    name="Data";
    name.concat("_");
    name.concat(filecount);
    name.concat(".txt");

    char filename[name.length()+1];
    name.toCharArray(filename, sizeof(filename));
    //Serial.println("checking filename:");
    //Serial.println(filename); 

    if (SD.exists(filename)) {
      newfile = 0;
      filecount+=1;
    }
    else{
      Serial.println("making new file called: ");
      Serial.println(filename);

      myFile = SD.open(filename, FILE_WRITE);
      myFile.println("GR1 Data File");//write header in here
      //myFile.close();  //this is the only file that will be open
      newfile =1;
    }
  }
  delay(200);
  tic=millis();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//GPS data collect
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
 
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop()
{  

  if (run==0){
    char filename[name.length()+1];
    name.toCharArray(filename, sizeof(filename));
  }
  
  // iOS - BLE connection
  binaryFloat myLat, myLon;
  byte satellitesByte;
  byte cpsByte;
  byte statusByte = 0x00;
  // --

  int i=1;
  int dum=0;
  int test =0;
  int gooddata=1;
  //int data[31];
  int dataStore[61];

  i=1;

  Usb.Task();

  for(uint8_t a; a<61;a++) {
    dataStore[a] = newdata[a];
  }

  while (i <= 63 & gooddata ==1 ) {
    // the next word is formed from dataStore[i] and dataStore[i+1]
    // check the least significant bit of the word to test
    // for a valid channel number
      if (0x01 & dataStore[i + 1]) {
        // derive channel number from 12 most significant bits of word
        // formed by the ith and ith+1 bytes
        int channel = (((unsigned int) dataStore[i]) << 4) |
        (((unsigned int) dataStore[i + 1] & 0xf0) >> 4);
        //Serial.println(channel);
        if (channel>minChannel){
          //Serial.println(channel);
          //delay(20);
          //next two lines causing issues
          myFile.println(String(channel,DEC));

          //data[counts]=channel;
          counts+=1;
        }
      //Serial.println(data[counts]);
      gooddata=1;
      i += 2;

      //Serial.print("counts detected");
      } 
      else {
      gooddata=0;
      i += 2;
      //if not a valid channel, no further entries are valid
      }
 }


  if (timing>exposure){
    
    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
    }
  
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  
    // iOS connection
    myLat.floatingPoint = GPS.latitudeDegrees;
    myLon.floatingPoint = GPS.longitudeDegrees;
    satellitesByte = (byte)GPS.satellites;
    // --
    
    if (GPS.fix) {
      
      //prepare the status byte
      statusByte = 0x01;
      satellitesByte = satellitesByte << 1;
      statusByte = statusByte | satellitesByte;
      cpsByte = (byte)counts;
      
      //Serial.print("Status: ");
      //Serial.println(statusByte);
      
      //send the stuff to iOS
      ble_write(statusByte);
      ble_write(myLat.binary[3]);
      ble_write(myLat.binary[2]);
      ble_write(myLat.binary[1]);
      ble_write(myLat.binary[0]);
      ble_write(myLon.binary[3]);
      ble_write(myLon.binary[2]);
      ble_write(myLon.binary[1]);
      ble_write(myLon.binary[0]);
      ble_write(cpsByte);
      // end of iOS transmission
      
      Serial.print("loc ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      myFile.print("LOC ");
      myFile.print(GPS.latitude, 4); myFile.print(GPS.lat);
      myFile.print(", "); 
      myFile.print(GPS.longitude, 4); myFile.println(GPS.lon);
      
      Serial.print("SPD "); Serial.println(GPS.speed);
      Serial.print("BRG "); Serial.println(GPS.angle);
      Serial.print("ALT "); Serial.println(GPS.altitude);
      
      myFile.print("SPD "); myFile.println(GPS.speed);
      myFile.print("BRG "); myFile.println(GPS.angle);
      myFile.print("ALT "); myFile.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    } else {
    Serial.println("NO FIX");
    myFile.println("NO FIX");
    
    ble_write(statusByte);
    ble_write(0x00); ble_write(0x00); ble_write(0x00); ble_write(0x00);
    ble_write(0x00); ble_write(0x00); ble_write(0x00); ble_write(0x00);
    ble_write(0x00);
    }
    
   
   int sensorValue = analogRead(A0);
   float voltage = sensorValue * (5.0 / 1023.0);// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
   Serial.print("SNR ");
   Serial.println(voltage);// print out the value you read:
   myFile.print("SNR ");
   myFile.println(voltage);
   
    myFile.println("NWP");//print to sd
    //myFile.println(totaltime);
    myFile.flush();
    //Serial.println();
    Serial.println("NWP");
    Serial.println(counts);
    counts=0;
    timing=0;
    //int data[100];
  }

  toc=millis();
  timing=timing +toc-tic;
  tic=toc;
  test+=1;
  
  
  ble_do_events();
  delay(50);
}




















