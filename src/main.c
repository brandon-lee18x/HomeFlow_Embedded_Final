#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include "bluetooth.c"
#include "../inc/display.h"
#include "../inc/spi.h"
#include "../inc/r_enc.h"
#include "../inc/bme.h"
#include "../inc/temp.h"
#include "../inc/hr.h"
#include "../inc/imu.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>

const struct device* i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device* gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = uart_init();
	if (err) {
		// error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		// error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	//weather sensor init
	BME688_Init();

	//IMU init
	init_IMU_cs();
	init_IMU();
	init_IMU_interrupts();
	init_RCFilters();

	while (1) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));

		//temp sensor
		float temp = get_body_temp();
		char temp_buf[10];
		snprintf(temp_buf, sizeof(temp_buf), "BTS:%f", temp);

		// weather sensor
		BME688_Take_Measurement();
		char weather_buf[10];
		int humidity = (int)BME688_Get_Temp_C();
		snprintf(weather_buf, sizeof(weather_buf), "BMH:%d", humidity);

		float x_xl = poll_IMU();
		char x_xl_buf[10];
		snprintf(x_xl_buf, sizeof(x_xl_buf), "BBV:%f", x_xl);

		if (current_conn) {
			bt_nus_send(current_conn, temp_buf, strlen(temp_buf));
			bt_nus_send(current_conn, weather_buf, strlen(weather_buf));
			bt_nus_send(current_conn, x_xl_buf, strlen(x_xl_buf));
		}
	}
	
}