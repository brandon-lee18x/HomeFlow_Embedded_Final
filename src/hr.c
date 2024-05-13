#include "../inc/hr.h"

uint8_t HR_ADDR = 0x55;

void init_hr() {
    int ret;
    uint8_t data_buffer[2];
	
    // Configure pins 11 and 12 as outputs for use with heart rate monitor
	// P0.11 = MFIO and P0.12 = RST
    gpio_pin_configure(gpio0_dev, MFIO, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
	gpio_pin_set(gpio0_dev, MFIO, 0);
    gpio_pin_configure(gpio0_dev, RST, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH); 
	gpio_pin_set(gpio0_dev, RST, 1);

	k_msleep(1000);

	gpio_pin_set(gpio0_dev, RST, 0);
	k_msleep(5);
	gpio_pin_set(gpio0_dev, MFIO, 1);
	k_msleep(5);
	gpio_pin_set(gpio0_dev, RST, 1);
	k_msleep(750);

	// uint8_t write_mode[3] = {0x01, 0x00, 0x00};
	// ret = i2c_write(i2c0_dev, write_mode, 3, HR_ADDR);
	// if (ret < 0) {
	// 	printk("error writing to hr sensor");
	// 	return 0; // Error writing to sensor
	// }
	
	uint8_t read_mode[2] = {0x02, 0x00};
	ret = i2c_write(i2c0_dev, read_mode, 2, HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t mode_data[2];
	ret = i2c_read(i2c0_dev, mode_data, sizeof(mode_data), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	printf("Mode:  %u %u\n\n", mode_data[0], mode_data[1]);

	uint8_t read_register_attributes[2] = {0x42, 0x03};
	ret = i2c_write(i2c0_dev, read_register_attributes, 2, HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t reg_atts[3];
	ret = i2c_read(i2c0_dev, reg_atts, sizeof(reg_atts), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	printf("MAX30101 Register Attributes:  %u %u %u\n\n", reg_atts[0], reg_atts[1], reg_atts[2]);


	uint8_t read_reg_7[3] = {0x41, 0x03, 0x07};
	ret = i2c_write(i2c0_dev, read_reg_7, sizeof(read_reg_7), HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t reg_7[2];
	ret = i2c_read(i2c0_dev, reg_7, sizeof(reg_7), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	printf("MAX30101 Register 7:  %u %u\n\n", reg_7[0], reg_7[1]);

	uint8_t raw_alg_mode[3] = {0x10, 0x00, 0x03};
	ret = i2c_write(i2c0_dev, raw_alg_mode, sizeof(raw_alg_mode), HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t raw_alg_response[1];
	ret = i2c_read(i2c0_dev, raw_alg_response, sizeof(raw_alg_response), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	printf("Raw+Alg Data:  %u\n\n", raw_alg_response[0]);
	
	uint8_t set_fifo_limit[3] = {0x10, 0x01, 0x0F};
	ret = i2c_write(i2c0_dev, set_fifo_limit, sizeof(set_fifo_limit), HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t fifo_limit_response[2];
	ret = i2c_read(i2c0_dev, fifo_limit_response, sizeof(fifo_limit_response), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	k_msleep(1000);
	
	printf("FIFO Threshold:  %u\n\n", fifo_limit_response[0]);
	
	uint8_t en_30101[3] = {0x44, 0x03, 0x01};
	ret = i2c_write(i2c0_dev, en_30101, sizeof(en_30101), HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t en_30101_response[2];
	ret = i2c_read(i2c0_dev, en_30101_response, sizeof(en_30101_response), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	printf("MAX30101 Enable:  %u\n\n", en_30101_response[0]);

	k_msleep(200);
	
	uint8_t enable_mf[3] = {0x52, 0x02, 0x01};
	ret = i2c_write(i2c0_dev, enable_mf, sizeof(enable_mf), HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
	
	uint8_t mf_en_response[2];
	ret = i2c_read(i2c0_dev, mf_en_response, sizeof(mf_en_response), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	
	printf("MaximFast Enable:  %u\n\n", mf_en_response[0]);
	k_msleep(500);
	uint8_t read_sensor_status[2] = {0x00, 0x00};
	ret = i2c_write(i2c0_dev, read_sensor_status, sizeof(read_sensor_status), HR_ADDR);
		if (ret < 0) {
			printk("error writing to hr sensor");
			return 0; // Error writing to sensor
		}
		
	uint8_t sensor_status[2];
	ret = i2c_read(i2c0_dev, sensor_status, sizeof(sensor_status), HR_ADDR);
		if (ret < 0) {
			printk("error reading from hr sensor");
			return 0; // Error reading from sensor
		}
	k_msleep(1000);
	printf("Sensor Status:  %u\n\n", sensor_status[0], sensor_status[1]);
}

float* read_hr() {
    uint8_t get_fifo[2] = {0x12, 0x01};
    int ret = i2c_write(i2c0_dev, get_fifo, sizeof(get_fifo), HR_ADDR);
    if (ret < 0) {
		printk("error writing to hr sensor");
        return 0; // Error writing to sensor
    }
		
    uint8_t fifo[19];
    ret = i2c_read(i2c0_dev, fifo, sizeof(fifo), HR_ADDR);
    if (ret < 0) {
		printk("error reading from hr sensor");
        return 0; // Error reading from sensor
    }

	static float arr[3];
    arr[0] = (fifo[13]<<8|fifo[14])/10.0; // Heart rate (BPM)
	arr[1] = fifo[15]; // Heart rate confidence  (%)
	arr[2] = (fifo[16]<<8|fifo[17])/10.0; // Blood oxygen saturation (%)
    return arr;
}