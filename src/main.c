#include <zephyr/types.h>
#include <zephyr/zephyr.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include "bluetooth.c"
#include "../inc/spi.h"
#include "../inc/r_enc.h"
#include "../inc/bme.h"
#include "../inc/temp.h"
#include "../inc/hr.h"
#include "../inc/imu.h"
#include "../inc/display.h"
#include "../inc/mutexes.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_saadc.h>

const struct device* i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device* gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
const struct device* gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device* spi1_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

#define IMU_STACK_SIZE 2048
#define IMU_PRIORITY 8
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_STACK_SIZE);
struct k_thread imu_thread_data;

#define DISPLAY_STACK_SIZE 1024
#define DISPLAY_PRIORITY 1
K_THREAD_STACK_DEFINE(display_thread_stack, DISPLAY_STACK_SIZE);
struct k_thread display_thread_data;


int main(void)
{
	//ble init
	initialize_ble();

	BME688_Init();

	// IMU init
	init_IMU_cs();
	init_IMU();
	init_RCFilters();

	//r_enc init
	init_enc();

	// Heart rate sensor initialization
	init_hr();

	//mutex init for shared resources
	init_mutexes();
	
	sensor_data data;
	int steps = 0;

	k_thread_create(&imu_thread_data, imu_thread_stack, K_THREAD_STACK_SIZEOF(imu_thread_stack),
					imu_thread_entry, &steps, NULL, NULL, IMU_PRIORITY, 0, K_NO_WAIT);
	k_thread_create(&display_thread_data, display_thread_stack, K_THREAD_STACK_SIZEOF(display_thread_stack),
					display_thread_entry, &data, &steps, NULL, DISPLAY_PRIORITY, 0, K_NO_WAIT);
	k_thread_start(&imu_thread_data);
	k_thread_start(&display_thread_data);

	while (1) {
		k_msleep(1000);

		char temp_buf[10];
		char humidity_buf[10];
		char temp_c_buf[10];
		char gas_buf[10];
		char pressure_buf[10];
		char hr_buf[10];
		char spo2_buf[10];
		char hr_confidence_buf[10];
		char step_buf[10];
		char xl_x_buf[10];
		char xl_y_buf[10];
		char xl_z_buf[10];

		//poll I2C sensors
		k_mutex_lock(&i2c_lock, K_FOREVER);

		//body temp sensor
		data.body_temp = get_body_temp();
		snprintf(temp_buf, sizeof(temp_buf), "BTS:%f", data.body_temp);

		//weather sensor
		BME688_Take_Measurement();
		data.humidity = (float)BME688_Get_Humidity();
		snprintf(humidity_buf, sizeof(humidity_buf), "BMH:%f", data.humidity);
		data.weather_temp = (float)BME688_Get_Temp_C();
		snprintf(temp_c_buf, sizeof(temp_c_buf), "BMT:%f", (data.weather_temp*1.8)+32);
		data.aqi = (float)BME688_Get_Gas();
		snprintf(gas_buf, sizeof(gas_buf), "BMG:%f", data.aqi);
		data.pressure = (float)BME688_Get_Pressure();
		snprintf(pressure_buf, sizeof(pressure_buf), "BMP:%f", data.pressure);

		//hr sensor
		float* hr_data = read_hr();
		data.hr = hr_data[0];
		data.hr_confidence = (int)hr_data[1];
		data.bos = hr_data[2];
		snprintf(hr_buf, sizeof(hr_buf), "BPM:%f", data.hr);
		snprintf(spo2_buf, sizeof(spo2_buf), "BOS:%f", data.bos);
		snprintf(hr_confidence_buf, sizeof(hr_confidence_buf), "HRC:%d", data.hr_confidence);

		k_mutex_unlock(&i2c_lock);

		//read step count
		k_mutex_lock(&steps_lock, K_FOREVER);
		snprintf(step_buf, sizeof(step_buf), "STP:%d", steps);
		k_mutex_unlock(&steps_lock);

		//read IMU readings
		k_mutex_lock(&imu_lock, K_FOREVER);
		snprintf(xl_x_buf, sizeof(xl_x_buf), "XLX:%f", IMU_readings[0]);
		snprintf(xl_y_buf, sizeof(xl_y_buf), "XLY:%f", IMU_readings[1]);
		snprintf(xl_z_buf, sizeof(xl_z_buf), "XLZ:%f", IMU_readings[2]);
		k_mutex_unlock(&imu_lock);

		if (current_conn) {
			bt_nus_send(current_conn, temp_buf, strlen(temp_buf));
			bt_nus_send(current_conn, humidity_buf, strlen(humidity_buf));
			bt_nus_send(current_conn, temp_c_buf, strlen(temp_c_buf));
			bt_nus_send(current_conn, gas_buf, strlen(gas_buf));
			bt_nus_send(current_conn, pressure_buf, strlen(pressure_buf));
			bt_nus_send(current_conn, step_buf, strlen(step_buf));
			bt_nus_send(current_conn, hr_buf, strlen(hr_buf));
			bt_nus_send(current_conn, spo2_buf, strlen(spo2_buf));
			bt_nus_send(current_conn, hr_confidence_buf, strlen(hr_confidence_buf));
			bt_nus_send(current_conn, xl_x_buf, strlen(xl_x_buf));
			bt_nus_send(current_conn, xl_y_buf, strlen(xl_y_buf));
			bt_nus_send(current_conn, xl_z_buf, strlen(xl_z_buf));
		}
	}
	
}