#include <stdint.h>
#include <zephyr/drivers/i2c.h>
extern const struct device* i2c0_dev;

float get_body_temp();