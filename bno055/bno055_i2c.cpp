#include <px4_platform_common/px4_config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

#include "board_config.h"
#include "bno055.hpp"

class BNO055_I2C : public device::I2C
{
public:
	BNO055_I2C(int bus, int bus_frequency);
	~BNO055_I2C() override = default;

	int read(unsigned address, void *data, unsigned count);
	int write(unsigned address, void *data, unsigned count);

protected:
	int probe() override;
};

device::Device *BNO055_I2C_interface(int bus, int bus_frequency);

device::Device *BNO055_I2C_interface(int bus, int bus_frequency)
{
	return new BNO055_I2C(bus, bus_frequency);
}

BNO055_I2C::BNO055_I2C(int bus, int bus_frequency) :
	I2C(DRV_MAG_DEVTYPE_BNO055, MODULE_NAME, bus, BNO055_I2C_ADDR, bus_frequency)
{
}

int BNO055_I2C::probe()
{
	uint8_t data = 0;

	_retries = 10;

	if (read(BNO055_I2C_ADDR, &data, 1)) {
		DEVICE_DEBUG("BNO055 read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if (data != BNO055_ID) {
		DEVICE_DEBUG("BNO055 bad ID: %02x", data);
		return -EIO;
	}

	return OK;
}

int BNO055_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	int ret;

	/* We need a first transfer where we write the register to read */
	ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != OK) {
		return ret;
	}

	/* Now we read the previously selected register */
	ret = transfer(nullptr, 0, (uint8_t *)data, count);

	return ret;
}

int BNO055_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}
