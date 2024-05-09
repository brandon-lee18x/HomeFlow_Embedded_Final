#ifndef HR
#define HR
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#define MFIO 11 //PIN_0_11
#define RST 12 //PIN_0_12

extern uint8_t HR_ADDR;

extern const struct device* gpio0_dev;
extern const struct device* i2c0_dev;

void init_hr();
float read_hr();
float read_blood_ox();
#endif