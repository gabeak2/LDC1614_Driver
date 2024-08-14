/*
 * ldc1614.h
 *
 *  Created on: Aug 10, 2024
 *      Author: gabea
 */

#ifndef INC_LDC1614_H_
#define INC_LDC1614_H_

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define NSAMPS 100
#define SAMP_DELAY 10

#define leftshift(value,nbits) (((uint32_t)(value)) << (nbits))

// Register Addresses
#define LDC1614_ADDR (0x2B << 1) //if ADDR pin pulled low, 0x2A, but I pulled it High so 0x2B
// Register Addresses
#define LDC1614_DATA0_MSB        0x00  // Channel 0 MSB Conversion Result and Error Status
#define LDC1614_DATA0_LSB        0x01  // Channel 0 LSB Conversion Result. Must be read after Register address 0x00.
#define LDC1614_DATA1_MSB        0x02  // Channel 1 MSB Conversion Result and Error Status
#define LDC1614_DATA1_LSB        0x03  // Channel 1 LSB Conversion Result. Must be read after Register address 0x02.
#define LDC1614_DATA2_MSB        0x04  // Channel 2 MSB Conversion Result and Error Status. (LDC1614 only)
#define LDC1614_DATA2_LSB        0x05  // Channel 2 LSB Conversion Result. Must be read after Register address 0x04. (LDC1614 only)
#define LDC1614_DATA3_MSB        0x06  // Channel 3 MSB Conversion Result and Error Status. (LDC1614 only)
#define LDC1614_DATA3_LSB        0x07  // Channel 3 LSB Conversion Result. Must be read after Register address 0x06. (LDC1614 only)
#define LDC1614_RCOUNT0          0x08  // Reference Count setting for Channel 0
#define LDC1614_RCOUNT1          0x09  // Reference Count setting for Channel 1
#define LDC1614_RCOUNT2          0x0A  // Reference Count setting for Channel 2. (LDC1614 only)
#define LDC1614_RCOUNT3          0x0B  // Reference Count setting for Channel 3. (LDC1614 only)
#define LDC1614_OFFSET0          0x0C  // Offset value for Channel 0
#define LDC1614_OFFSET1          0x0D  // Offset value for Channel 1
#define LDC1614_OFFSET2          0x0E  // Offset value for Channel 2. (LDC1614 only)
#define LDC1614_OFFSET3          0x0F  // Offset value for Channel 3. (LDC1614 only)
#define LDC1614_SETTLECOUNT0     0x10  // Channel 0 Settling Reference Count
#define LDC1614_SETTLECOUNT1     0x11  // Channel 1 Settling Reference Count
#define LDC1614_SETTLECOUNT2     0x12  // Channel 2 Settling Reference Count. (LDC1614 only)
#define LDC1614_SETTLECOUNT3     0x13  // Channel 3 Settling Reference Count. (LDC1614 only)
#define LDC1614_CLOCK_DIVIDERS0  0x14  // Reference and Sensor Divider settings for Channel 0
#define LDC1614_CLOCK_DIVIDERS1  0x15  // Reference and Sensor Divider settings for Channel 1
#define LDC1614_CLOCK_DIVIDERS2  0x16  // Reference and Sensor Divider settings for Channel 2. (LDC1614 only)
#define LDC1614_CLOCK_DIVIDERS3  0x17  // Reference and Sensor Divider settings for Channel 3. (LDC1614 only)
#define LDC1614_STATUS           0x18  // Device Status Report
#define LDC1614_ERROR_CONFIG     0x19  // Error Reporting Configuration
#define LDC1614_CONFIG           0x1A  // Conversion Configuration
#define LDC1614_MUX_CONFIG       0x1B  // Channel Multiplexing Configuration
#define LDC1614_RESET_DEV        0x1C  // Reset Device
#define LDC1614_DRIVE_CURRENT0   0x1E  // Channel 0 sensor current drive configuration
#define LDC1614_DRIVE_CURRENT1   0x1F  // Channel 1 sensor current drive configuration
#define LDC1614_DRIVE_CURRENT2   0x20  // Channel 2 sensor current drive configuration. (LDC1614 only)
#define LDC1614_DRIVE_CURRENT3   0x21  // Channel 3 sensor current drive configuration. (LDC1614 only)
#define LDC1614_MANUFACTURER_ID  0x7E  // Manufacturer ID
#define LDC1614_DEVICE_ID        0x7F  // Device ID


typedef struct
{
	I2C_HandleTypeDef *i2c_handle;
	uint32_t sensor_readings[4]; //1 per channel
} LDC1614;

int ldc1614_init(LDC1614 *dev, I2C_HandleTypeDef *i2c_handle, int channel, int L_uH, int C_pF);
HAL_StatusTypeDef ldc1614_write_reg(LDC1614 *dev, uint8_t reg_addr, uint16_t data);
HAL_StatusTypeDef ldc1614_read_reg(LDC1614 *dev, uint8_t reg_addr, uint8_t data[2]);
uint32_t ldc1614_get_ch0_reading(LDC1614 *dev);

#endif /* INC_LDC1614_H_ */
