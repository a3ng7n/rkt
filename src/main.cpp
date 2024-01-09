#include <Arduino.h>
// #include <Wire.h>
#include "mpu6500.h"

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

/* Mpu6500 object */
bfs::Mpu6500 imu;

void setup() {
  while (!Serial) {
    // wait for Arduino Serial Monitor to be ready
    delay(1000);
  }

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);
  //This will pipe all NMEA sentences to the serial port so we can see them
  myGNSS.setNMEAOutputPort(Serial);


  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(0)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

}

void loop() {
  /* Check if data read */
  // if (imu.Read()) {
  //   Serial.print(imu.new_imu_data());
  //   Serial.print("\t Accel x: ");
  //   Serial.print(imu.accel_x_mps2());
  //   Serial.print("\t Accel y: ");
  //   Serial.print(imu.accel_y_mps2());
  //   Serial.print("\t Accel z: ");
  //   Serial.print(imu.accel_z_mps2());
  //   Serial.print("\t Rate x: ");
  //   Serial.print(imu.gyro_x_radps());
  //   Serial.print("\t Rate y: ");
  //   Serial.print(imu.gyro_y_radps());
  //   Serial.print("\t Rate z: ");
  //   Serial.print(imu.gyro_z_radps());
  //   Serial.print("\t");
  //   Serial.print(imu.die_temp_c());
  //   Serial.print("\n");
  // }

  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);

    byte fixType = myGNSS.getFixType();
    Serial.print(F(" Fix: "));
    if(fixType == 0) Serial.print(F("No fix"));
    else if(fixType == 1) Serial.print(F("Dead reckoning"));
    else if(fixType == 2) Serial.print(F("2D"));
    else if(fixType == 3) Serial.print(F("3D"));
    else if(fixType == 4) Serial.print(F("GNSS + Dead reckoning"));
    else if(fixType == 5) Serial.print(F("Time only"));

    byte RTK = myGNSS.getCarrierSolutionType();
    Serial.print(" RTK: ");
    Serial.print(RTK);
    if (RTK == 0) Serial.print(F(" (No solution)"));
    else if (RTK == 1) Serial.print(F(" (High precision floating fix)"));
    else if (RTK == 2) Serial.print(F(" (High precision fix)"));

    Serial.println();
  }
}