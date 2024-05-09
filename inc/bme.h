#ifndef BME
#define BME
#include <stdint.h>
#include <zephyr/drivers/i2c.h>

extern const struct device* i2c0_dev;

void BME688_Init();
void BME688_Take_Measurement();
double BME688_Get_Temp_C();
double BME688_Get_Humidity();
double BME688_Get_Pressure();
double BME688_Get_Gas();
#endif