#include <Arduino.h>
// #include <Wire.h>

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

#include "mpu6500.h"
/* Mpu6500 object */
bfs::Mpu6500 imu;

#include <RadioLib.h>
// SX1276 has the following connections:
// CS pin:   10
// DIO0 pin:  3
// RESET pin: 2
// DIO1 pin:  4 <-- not doing this
SX1276 radio = new Module(10, 3, 2);

#include <Adafruit_BMP280.h>

// #define BMP_SCK  (13)
// #define BMP_MISO (12)
// #define BMP_MOSI (11)
// #define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void setup() {
  // while (!Serial) {
  //   // wait for Arduino Serial Monitor to be ready
  //   delay(1000);
  // }

  // initialize SX1276 with default settings
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);
  //This will pipe all NMEA sentences to the serial port so we can see them
  // myGNSS.setNMEAOutputPort(Serial);


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

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
  /* Check if data read */

  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    if (imu.Read()) {
      Serial.print(imu.new_imu_data());
      Serial.print("\t Accel x: ");
      Serial.print(imu.accel_x_mps2());
      Serial.print("\t Accel y: ");
      Serial.print(imu.accel_y_mps2());
      Serial.print("\t Accel z: ");
      Serial.print(imu.accel_z_mps2());
      Serial.print("\t Rate x: ");
      Serial.print(imu.gyro_x_radps());
      Serial.print("\t Rate y: ");
      Serial.print(imu.gyro_y_radps());
      Serial.print("\t Rate z: ");
      Serial.print(imu.gyro_z_radps());
      Serial.print("\t");
      Serial.print(imu.die_temp_c());
      Serial.print("\n");
    }
    
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
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
    
    Serial.print(F("[SX1276] Transmitting packet ... "));

    // you can transmit C-string or Arduino string up to
    // 255 characters long
    count++;
    if (count > 255) count = 0;
    String str = String(latitude) + "," + String(longitude) + "," + String(count++);
    int state = radio.transmit(str);

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
      int state = radio.transmit(byteArr, 8);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // the packet was successfully transmitted
      Serial.println(F(" success!"));

      // print measured data rate
      Serial.print(F("[SX1276] Datarate:\t"));
      Serial.print(radio.getDataRate());
      Serial.println(F(" bps"));

    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
      // the supplied packet was longer than 256 bytes
      Serial.println(F("too long!"));

    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
      // timeout occurred while transmitting packet
      Serial.println(F("timeout!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);

    }

  }
}