#include "bno055.h"

#include <iostream>
#include <chrono>
#include <thread>

#include <wiringPi.h>
#include <wiringPiI2C.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <unistd.h>

#include "utils.h"

void BNO055::read_len(uint8_t* buffer, uint8_t len) {
	if(read(fd, buffer, len) != len) {
		std::cout << "Read error" << std::endl;
	}
}

void BNO055::write_len(uint8_t* buffer, uint8_t len) {
	if(write(fd, buffer, len) != len) {
		std::cout << "Write error" << std::endl;
	}
}

// void BNO055::read_block(uint8_t reg, uint8_t* buffer, uint8_t len) {
// 	if(i2c_smbus_read_block_data(fd, reg, buffer) < 0) {
// 		std::cout << "Block read error" << std::endl;
// 	}
// }

void BNO055::write8(int reg, uint8_t data) {
	wiringPiI2CWriteReg8(fd, reg, data);
}

void BNO055::write16(int reg, uint16_t data) {
	wiringPiI2CWriteReg16(fd, reg, data);
}

uint8_t BNO055::read8(int reg) {
	return wiringPiI2CReadReg8(fd, reg);
}

uint16_t BNO055::read16(int reg) {
	return wiringPiI2CReadReg16(fd, reg);
}

bool BNO055::begin() {
	fd = wiringPiI2CSetup(BNO055_ADDR);
	if(fd == -1) {
		std::cout << "Error setting up BNO055" << std::endl;
		return false;
	}

	// Switch to config mode (just in case since this is the default)
	write8(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_CONFIG);
	delay(30);

	// Reset
	write8(BNO055_SYS_TRIGGER_ADDR, 0x20);

	while(read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
		delay(10);
	}
	delay(50);

	// Set to normal power mode
	write8(BNO055_PWR_MODE_ADDR, BNO055_PWR_MODE_NORMAL);
	delay(10);

	write8(BNO055_PAGE_ID_ADDR, 0);

	write8(BNO055_SYS_TRIGGER_ADDR, 0);
	delay(20);

	write8(BNO055_OPR_MODE_ADDR, 0x0C);
	delay(50);

	return true;
}

float BNO055::get_z_orientation() {
	uint8_t buffer[6];
	memset(buffer, 0, 6);

	int16_t x, y, z;

	//write_len(write_buffer, 1);

	//read_len(buffer, 6);
	//read_block(reg, buffer, len);
	i2c_smbus_read_block_data(fd, BNO055_EULER_H_LSB_ADDR, buffer);

	x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  	y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  	z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  	std::cout << "X: " << std::to_string((float)x / 16.0f) << std::endl;
  	std::cout << "Y: " << std::to_string((float)y / 16.0f) << std::endl;
  	std::cout << "Z: " << std::to_string((float)z / 16.0f) << std::endl;

  	return (float)z / 16.0f;
}