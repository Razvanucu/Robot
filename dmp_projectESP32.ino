#include "MPU_X.h"

const int IMU_ADDR_ON_BUS = 0x68;

MPUx imu(IMU_ADDR_ON_BUS);

void setup() {
  Serial.begin(9600);
  while(!Serial){
    delay(10); //wait for Serial Monitor to open
  }

  Wire.begin();
  delay(200);

  Serial1.begin(9600);
  while(!Serial1){
    delay(10);
  }

  // Initialize the IMU
  imu.init();
  delay(200);
}

void loop() {
  imu.printMPUData();
  delay(1000);
}
