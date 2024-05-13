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

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

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

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/*threads:
display
IMU
I2C (HR Sensor, Weather Sensor, Temp Sensor)
BT

*/

int main(void)
{
	// ADC Initialization
	int blink_status = 0;
	int err;
	uint32_t count = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 0;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
			return 0;
		}
	}

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

	// IMU init
	init_IMU_cs();
	init_IMU();
	init_IMU_interrupts();
	init_RCFilters();

	// Heart rate sensor initialization
	init_hr();

	char adc_buf[30];
	int adc_mv = 0;

	while (1) {
		// Read ADC Values
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int32_t val_mv;

			printk("- %s, channel %d: ",
			       adc_channels[i].dev->name,
			       adc_channels[i].channel_id);

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			err = adc_read(adc_channels[i].dev, &sequence);
			if (err < 0) {
				printk("Could not read (%d)\n", err);
				continue;
			}

			/*
			 * If using differential mode, the 16 bit value
			 * in the ADC sample buffer should be a signed 2's
			 * complement value.
			 */
			if (adc_channels[i].channel_cfg.differential) {
				val_mv = (int32_t)((int16_t)buf);
			} else {
				val_mv = (int32_t)buf;
			}
			if (val_mv != 0) {
				adc_mv = val_mv;
			}
			printk("%"PRId32, val_mv);
			err = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
			/* conversion to mV may not be supported, skip if not */
			if (err < 0) {
				printk(" (value in mV not available)\n");
			} else {
				printk(" = %"PRId32" mV\n", val_mv);
			}
		}

		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
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

		float x_xl = poll_IMU();
		char x_xl_buf[10];
		snprintf(x_xl_buf, sizeof(x_xl_buf), "BBV:%f", x_xl);

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
		
		snprintf(adc_buf, sizeof(adc_buf), "MIC:%d", adc_mv);

		if (current_conn) {
			bt_nus_send(current_conn, temp_buf, strlen(temp_buf));
			bt_nus_send(current_conn, humidity_buf, strlen(humidity_buf));
			bt_nus_send(current_conn, temp_c_buf, strlen(temp_c_buf));
			bt_nus_send(current_conn, gas_buf, strlen(gas_buf));
			bt_nus_send(current_conn, pressure_buf, strlen(pressure_buf));
			bt_nus_send(current_conn, x_xl_buf, strlen(x_xl_buf));
			bt_nus_send(current_conn, adc_buf, strlen(adc_buf));
			bt_nus_send(current_conn, hr_buf, strlen(hr_buf));
			bt_nus_send(current_conn, spo2_buf, strlen(spo2_buf));
			bt_nus_send(current_conn, hr_confidence_buf, strlen(hr_confidence_buf));
		}
	}
	
}