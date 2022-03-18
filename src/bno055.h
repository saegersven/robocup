#pragma once

#include <cstdint>

#define BNO055_ID (0xA0)

#define BNO055_ADDR 0x28
#define BNO055_CHIP_ID_ADDR 0x00

#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_EULER_H_MSB_ADDR 0x1B

#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_OPR_MODE_CONFIG 0x00

#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_PWR_MODE_NORMAL 0x00

#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_PAGE_ID_ADDR 0x07

class BNO055 {
public:
	bool begin();
	float get_z_orientation();

private:
	int fd;

	void read_len(uint8_t* buffer, uint8_t len);
	void write_len(uint8_t* buffer, uint8_t len);

	void write8(int reg, uint8_t data);
	void write16(int reg, uint16_t data);

	uint8_t read8(int reg);
	uint16_t read16(int reg);

	bool read_len(int reg, uint8_t* buffer, uint8_t len);
};