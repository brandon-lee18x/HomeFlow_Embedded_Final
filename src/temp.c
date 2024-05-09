#include "../inc/temp.h"

float get_body_temp() {
    uint8_t BT_ADDR = 0x48;
    // Data to write to the sensor (this is just an example, you'll want to replace with actual values)
    uint8_t write_data1[1] = {0x00};

    // Writing 2 bytes to the sensor
    int ret = i2c_write(i2c0_dev, write_data1, 1, BT_ADDR);
    if (ret < 0) {
        return 0; // Error writing to sensor
    }

    // Reading 2 bytes from the sensor
    uint8_t data_buffer[2];
    ret = i2c_read(i2c0_dev, data_buffer, sizeof(data_buffer), BT_ADDR);
    if (ret < 0) {
        return 0; // Error reading from sensor
    }

    // Raw temp output
    uint16_t temperature = (data_buffer[0]<<8)+(data_buffer[1]);
    float final_temp_F = (temperature / 128.0)*1.8 + 32;

    return final_temp_F;
}