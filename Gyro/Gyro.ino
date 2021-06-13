/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   See the bottom of this file for the license terms.
*/

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read gyroscope data
*/

#include <BMI160Gen.h>

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  //BMI160.begin(BMI160GenClass::I2C_MODE);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(250);
  BMI160.autoCalibrateGyroOffset();

  // Set accelerometer range to 2g
  BMI160.setAccelerometerRange(2);
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  //BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, -1);

  Serial.println("Initializing IMU device...done.");
}

void loop() {
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  int accelX, accelY, accelZ;
  float gx, gy, gz;
  float ax, ay, az;

  // read raw gyro measurements from device
  BMI160.readGyro(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();


  accelX = BMI160.readAccelerometer(X_AXIS);
  accelY = BMI160.readAccelerometer(Y_AXIS);
  accelZ = BMI160.readAccelerometer(Z_AXIS);
  
  // convert the raw gyro data to degrees/second
  ax = convertRawAccel(accelX);
  ay = convertRawAccel(accelY);
  az = convertRawAccel(accelZ);
  
  Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();
  Serial.println();

  delay(500);
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

float convertRawAccel(int aRaw) {
  // since we are using 2g range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  float a = (aRaw / 32768.0) * 2 * -9.8;
  return a;
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
