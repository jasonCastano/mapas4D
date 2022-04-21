#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

String msg = "";
String gps_msg = "";

char c = ' ';
void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);

  // Configuracion prescaler clock
  CLKPR = 0x80;
  CLKPR = 0x00;
  // Configuracion USART 1
  UCSR1B = 0x90; //0b10010000
  UCSR1C = 0x06; //0b00000110
  UBRR1 = 103; //0b01100111

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  msg = "";
  // read the sensor
  //sei();
  IMU.readSensor();
  //sei();
  msg += String(IMU.getAccelX_mss(),6) + ",";
  msg += String(IMU.getAccelY_mss(),6) + ",";
  msg += String(IMU.getAccelZ_mss(),6) + ",";
  msg += String(IMU.getGyroX_rads(),6) + ",";
  msg += String(IMU.getGyroY_rads(),6) + ",";
  msg += String(IMU.getGyroZ_rads(),6);
  cli();
  Serial.println(msg);
  sei();
}

ISR(USART1_RX_vect){
  c =   UDR1;
  //Serial.print(c);
  if(c == '\n'){
    Serial.println(gps_msg);
    gps_msg = "";
  }
  else{
    gps_msg += c;  
  }
}
