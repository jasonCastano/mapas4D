/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

String msg = "";
//(RX,TX)
SoftwareSerial gps_ss(RXPin,TXPin);

void setup() {
  // serial to display data

  CLKPR = 0x80;
  CLKPR = 0x00;
  
  Serial.begin(115200);
  gps_ss.begin(GPSBaud); //gps serial communication bauds

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  IMU.readSensor();
  msg = "";*/
  // read the sensor
  // display the data
  /*Serial.print(IMU.getAccelX_mss(),6);
  Serial.print(",");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print(",");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print(",");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print(",");  
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print(",");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.println(",");*/
/*
  msg += String(IMU.getAccelX_mss(),6) + ",";
  msg += String(IMU.getAccelY_mss(),6) + ",";
  msg += String(IMU.getAccelZ_mss(),6) + ",";
  msg += String(IMU.getGyroX_rads(),6) + ",";
  msg += String(IMU.getGyroY_rads(),6) + ",";
  msg += String(IMU.getGyroZ_rads(),6);
*/
/*  unsigned long start = millis();
  do{
     while(gps_ss.available() > 0){
      gps.encode(gps_ss.read());
      }
    }while(millis() - start < 0.0000000001);
  */
 /* if(gps_ss.available()>0){
    if(gps.encode(gps_ss.read())){
      if(gps.location.isValid()){
    //Serial.println("***SE PASA PRIMER IF**");
          msg += "," + String(gps.location.lat(),6) + ",";
          msg += String(gps.location.lng(),6) + ",";
          //msg += String(gps.altitude.meters(),6);
      }
    }
  }*/
          
  
  //Serial.print(msg+'\n');
  
  /*Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);*/
  //delay(100);
//}
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

String msg = "";

SoftwareSerial gps_ss(RXPin,TXPin);

void setup() {
  // serial to display data

  CLKPR = 0x80;
  CLKPR = 0x00;
  
  Serial.begin(230400);
  while(!Serial) {}

  gps_ss.begin(9600);

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
   msg = "";
  // read the sensor
  IMU.readSensor();

  msg += String(IMU.getAccelX_mss(),6) + ",";
  msg += String(IMU.getAccelY_mss(),6) + ",";
  msg += String(IMU.getAccelZ_mss(),6) + ",";
  msg += String(IMU.getGyroX_rads(),6) + ",";
  msg += String(IMU.getGyroY_rads(),6) + ",";
  msg += String(IMU.getGyroZ_rads(),6);
  Serial.println(msg);

}
