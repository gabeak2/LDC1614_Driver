/*
 * ldc1614.c
 *
 *  Created on: Aug 10, 2024
 *      Author: gabea
 */
#include "ldc1614.h"

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef ldc1614_write_reg(LDC1614 *dev, uint8_t reg_addr, uint16_t data)
{

	//does the I2C Transmit function send the 0th entry first or
	//the 2nd entry first??
	uint8_t packet[2];
	packet[0] = data>>8;
	packet[1] = data & 0xFF;

	return HAL_I2C_Mem_Write(dev->i2c_handle, LDC1614_ADDR, reg_addr, 1, packet, 2, 100);

}

HAL_StatusTypeDef ldc1614_read_reg(LDC1614 *dev, uint8_t reg_addr, uint8_t data[2])
{

	//set Read bit to 1 for 2nd transaction
	uint8_t read_addr = (LDC1614_ADDR | 0x01);

	return HAL_I2C_Mem_Read(dev->i2c_handle, read_addr, reg_addr, 1, data, 2, 100);

}

int ldc1614_init(LDC1614 *dev, I2C_HandleTypeDef *i2c_handle, int channel, int L_uH, int C_pF)
{

	dev->i2c_handle = i2c_handle; //tie the LDC1614 struct and the I2C handle together

	//	See "Recommended Initial Register Configuration Values" p51 in datasheet

	//	When the LDC powers up, it enters into Sleep Mode and will wait for configuration.
	//	Once the device is configured, exit Sleep Mode and begin conversions
	//	by setting CONFIG.SLEEP_MODE_EN to b0.

	//maximum conversion interval --> highest resolution?
	ldc1614_write_reg(dev, LDC1614_RCOUNT0, 0xFFFF);

	//	0x0000: Settle Time (tS0)= 32 ÷ ƒREF0
	//	0x0001: Settle Time (tS0)= 32 ÷ ƒREF0
	//	0x0002 - 0xFFFF: Settle Time (tS0)= (SETTLECOUNT0ˣ16) ÷ ƒREF0

	ldc1614_write_reg(dev, LDC1614_SETTLECOUNT0, 0x000A);

	//	Don't divide either clk - reference or sensor
	ldc1614_write_reg(dev, LDC1614_CLOCK_DIVIDERS0, 0x1002);

	//	don't report any errors
	ldc1614_write_reg(dev, LDC1614_ERROR_CONFIG, 0x0000);

	//Enable Channel 0 in continuous mode, set Input deglitch bandwidth to 3.3MHz
	ldc1614_write_reg(dev, LDC1614_MUX_CONFIG, 0x020C);

	//Manually set sensor drive current on channel 0
	//Why don't they recommend to use automatic amplitude control??
	ldc1614_write_reg(dev, LDC1614_DRIVE_CURRENT0, 0x9000);

	//Select active channel = ch 0, disable auto-amplitude correction and autocalibration, enable full current drive during sensor activation, select
	//external clock source, wake up device to start conversion. This register
	//write must occur last because device configuration is not permitted while
	//the LDC is in active mode.
	ldc1614_write_reg(dev, LDC1614_CONFIG, 0x1601);

	return 0;
}

uint32_t ldc1614_get_ch0_reading(LDC1614 *dev)

{
	uint32_t sensor_value = 0;
	uint8_t errors = 0;
	uint8_t data_msb[2] = {0};
	uint8_t data_lsb[2] = {0};
	int read_ok;

	//read MSB register
	read_ok = ldc1614_read_reg(dev, 0x00, data_msb);

	//read LSB register
	read_ok = ldc1614_read_reg(dev, 0x01, data_lsb);

	sensor_value = (leftshift((data_msb[0]&0x0F),24) | leftshift(data_msb[1],16) | leftshift(data_lsb[0],8) | data_lsb[1]);

	errors = (data_msb[0] >> 4) & 0x0F;

	dev->sensor_readings[0] = sensor_value;

	return sensor_value;
}



