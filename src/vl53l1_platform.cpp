/**
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "vl53l1_platform.h"
#include "Wire.h"
#include <math.h>
#include <string.h>
#include <time.h>

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata,
                         uint32_t count) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */

  Wire.beginTransmission(static_cast<uint8_t>(dev));

  // Send index to write at (16-bit address)
  Wire.write(index >> 8);   // High byte
  Wire.write(index & 0xFF); // Low byte

  // Write data
  for (uint32_t i = 0; i < count; i++) {
    Wire.write(pdata[i]);
  }

  // Successful if transmission completes
  if (Wire.endTransmission() == 0) {
    status = 0;
  }

  return status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata,
                        uint32_t count) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));

  // Send index to write at (16-bit address)
  Wire.write(index >> 8);   // High byte
  Wire.write(index & 0xFF); // Low byte

  if (Wire.endTransmission(false) != 0) // repeated start
    return status;

  // Request from device
  Wire.requestFrom(dev, count);
  // Read data
  if (Wire.available() != count)
    return status;

  for (uint32_t i = 0; i < count; i++) {
    pdata[i] = Wire.read();
  }

  return status;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));

  Wire.write(index >> 8);
  Wire.write(index & 0xFF);

  Wire.write(data);
  // Successful if transmission completes
  if (Wire.endTransmission() == 0) {
    status = 0;
  }
  return status;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));

  Wire.write(index >> 8);
  Wire.write(index & 0xFF);

  Wire.write((data >> 8) & 0xFF);
  Wire.write(data & 0xFF);

  // Successful if transmission completes
  if (Wire.endTransmission() == 0) {
    status = 0;
  }
  return status;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));

  Wire.write(index >> 8);
  Wire.write(index & 0xFF);

  Wire.write((data >> 24) & 0xFF);
  Wire.write((data >> 16) & 0xFF);
  Wire.write((data >> 8) & 0xFF);
  Wire.write(data & 0xFF);

  // Successful if transmission completes
  if (Wire.endTransmission() == 0) {
    status = 0;
  }
  return status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));
  Wire.write(index >> 8);
  Wire.write(index & 0xFF);
  uint8_t result = Wire.endTransmission(false);
  if (result != 0) {
    status = result;
    return status;
  }

  if (Wire.requestFrom(static_cast<uint8_t>(dev), static_cast<uint8_t>(1)) !=
      1) {
    status = 20;
    return status;
  }

  if (Wire.available() >= 1) {
    *pdata = Wire.read();
    status = 0;
  }

  return status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));
  Wire.write(index >> 8);
  Wire.write(index & 0xFF);

  if (Wire.endTransmission(false) != 0) {
    return status;
  }

  Wire.requestFrom(static_cast<uint8_t>(dev), static_cast<uint8_t>(2));

  if (Wire.available() == 2) {
    *pdata = ((uint32_t)Wire.read() << 8) | ((uint32_t)Wire.read());
    status = 0;
  }

  return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata) {
  uint8_t status = 255;

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  Wire.beginTransmission(static_cast<uint8_t>(dev));
  Wire.write(index >> 8);
  Wire.write(index & 0xFF);

  if (Wire.endTransmission(false) != 0) {
    return status;
  }

  Wire.requestFrom(static_cast<uint8_t>(dev), static_cast<uint8_t>(1));

  if (Wire.available() == 4) {
    *pdata = ((uint32_t)Wire.read() << 24) | ((uint32_t)Wire.read() << 16) |
             ((uint32_t)Wire.read() << 8) | ((uint32_t)Wire.read());
    status = 0;
  }

  return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {

  /* To be filled by customer. Return 0 if OK */
  /* Warning : For big endian platforms, fields 'RegisterAdress' and 'value'
   * need to be swapped. */
  delay(wait_ms);
  return 0;
}
