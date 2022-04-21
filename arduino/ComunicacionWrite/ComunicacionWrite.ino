  #include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

String msg = "";

void setup() {
  // serial to display data

  CLKPR = 0x80;
  CLKPR = 0x00;
  
  Serial.begin(115200);
  while(!Serial) {}

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
  // put your main code here, to run repeatedly:
  IMU.readSensor();
  msg = "";
  msg += String(IMU.getAccelX_mss(), 5);
  msg += "*";
  msg += String(IMU.getAccelY_mss(), 5) + "w";
  msg += String(IMU.getAccelZ_mss(), 5) + "w";
  msg += String(IMU.getGyroX_rads(), 5) + "w";
  msg += String(IMU.getGyroY_rads(), 5) + "w";
  msg += String(IMU.getGyroZ_rads(), 5) + "x";

  int sizeM = sizeof(msg);
  
  char buff[sizeM];

  msg.toCharArray(buff, sizeM);

  Serial.write(buff, sizeM);
  
}
