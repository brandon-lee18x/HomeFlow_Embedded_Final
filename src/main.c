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

#define IMU_STACK_SIZE 1024
#define IMU_PRIORITY 8
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_STACK_SIZE);
struct k_thread imu_thread_data;

#define DISPLAY_STACK_SIZE 1024
#define DISPLAY_PRIORITY 8
K_THREAD_STACK_DEFINE(display_thread_stack, DISPLAY_STACK_SIZE);
struct k_thread display_thread_data;

/*threads:
display/rotary encoder
IMU
I2C (HR Sensor, Weather Sensor, Temp Sensor)
BT

*/

/*
Remaining TODO's
- step tracking
- rotary encoder/display integration (in a thread)
- display UI
- fall detection (priority 2)
- 3D print rotary encoder handle
- enclosure (bands)
- project and paper
- firebase
*/

int main(void)
{
	//ble init
	initialize_ble();

	//weather sensor init
	BME688_Init();

	// IMU init
	init_IMU_cs();
	init_IMU();
	init_IMU_interrupts();
	init_RCFilters();

	// Heart rate sensor initialization
	init_hr();

	k_thread_create(&imu_thread_data, imu_thread_stack, K_THREAD_STACK_SIZEOF(imu_thread_stack),
					imu_thread_entry, NULL, NULL, NULL, IMU_PRIORITY, 0, K_NO_WAIT);
	k_thread_create(&display_thread_data, display_thread_stack, K_THREAD_STACK_SIZEOF(display_thread_stack),
					display_thread_entry, NULL, NULL, NULL, IMU_PRIORITY, 0, K_NO_WAIT);
	

	k_thread_start(&imu_thread_data);
	k_thread_start(&display_thread_data);

	while (1) {
		k_msleep(1000);
		// Temp sensor
		float temp = get_body_temp();
		char temp_buf[10];
		snprintf(temp_buf, sizeof(temp_buf), "BTS:%f", temp);

		// Take BME688 measurement
		BME688_Take_Measurement();
		char humidity_buf[10];
		float humidity = (float)BME688_Get_Humidity();
		snprintf(humidity_buf, sizeof(humidity_buf), "BMH:%f", humidity);

		char temp_c_buf[10];
		float temp_c = (float)BME688_Get_Temp_C();
		snprintf(temp_c_buf, sizeof(temp_c_buf), "BMT:%f", (temp_c*1.8)+32);

		char gas_buf[10];
		float gas = (float)BME688_Get_Gas();
		snprintf(gas_buf, sizeof(gas_buf), "BMG:%f", gas);

		char pressure_buf[10];
		float pressure = (float)BME688_Get_Pressure();
		snprintf(pressure_buf, sizeof(pressure_buf), "BMP:%f", pressure);

		char step_buf[10];
		snprintf(step_buf, sizeof(step_buf), "STP:%d", steps);

		float* hr_data = read_hr();
		float hr = hr_data[0];
		int hr_confidence = (int)hr_data[1];
		float bos = hr_data[2];

		char hr_buf[10];
		snprintf(hr_buf, sizeof(hr_buf), "BPM:%f", hr);
		
		char spo2_buf[10];
		snprintf(spo2_buf, sizeof(spo2_buf), "BOS:%f", bos);

		char hr_confidence_buf[10];
		snprintf(hr_confidence_buf, sizeof(hr_confidence_buf), "HRC:%d", hr_confidence);

		char xl_x_buf[10];
		char xl_y_buf[10];
		char xl_z_buf[10];
		snprintf(xl_x_buf, sizeof(xl_x_buf), "XLX:%f", IMU_readings[0]);
		snprintf(xl_y_buf, sizeof(xl_y_buf), "XLY:%f", IMU_readings[1]);
		snprintf(xl_z_buf, sizeof(xl_z_buf), "XLZ:%f", IMU_readings[2]);

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