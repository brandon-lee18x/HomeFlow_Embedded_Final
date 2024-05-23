#include "../inc/imu.h"

const struct device *spi3_dev = DEVICE_DT_GET(SPI3_NODE);
struct spi_config spi3_cfg = {
	.operation = SPI_WORD_SET(8),
	.frequency = 10000000, // 10 MHz, but this depends on your device and board capabilities
	.slave = 0, // Typically, your SPI slave's chip select pin
};

//order: xl_x, xl_y, xl_z, g_x, g_y, g_z
RCFilter lp_filters[NUM_AXES]; 
float cutoff_freqs[NUM_AXES] = {6.0f, 6.0f, 6.0f, 6.0f, 6.0f, 6.0f};

volatile float IMU_readings[NUM_AXES]; //sensor readings in g's and dps
volatile int interrupt_called = 0;
volatile int max_x = 0;
volatile float filtered_imu_data[3];
float prev_magnitude = 1;
int steps = 0;

LOG_MODULE_REGISTER(imu);

ButterworthFilter filter;

void spi_read_reg(uint8_t reg, uint8_t values[], uint8_t size) {
    int err;

	uint8_t tx_buffer[1];
	tx_buffer[0] = reg;

	struct spi_buf tx_spi_bufs[] = {
		{ .buf = tx_buffer, .len = sizeof(tx_buffer)},	
	};

	struct spi_buf_set spi_tx_buffer_set = {
		.buffers = tx_spi_bufs,
		.count = 1
	};

	struct spi_buf rx_spi_bufs[] = {
		{ .buf = values, .len = size }
	};

	struct spi_buf_set spi_rx_buffer_set = {
		.buffers = rx_spi_bufs,
		.count = 1
	};

	gpio_pin_set(gpio1_dev, GPIO_1_CS, 0);
	do {
		err = spi_write(spi3_dev, &spi3_cfg, &spi_tx_buffer_set);
		if (err < 0) break;

		err = spi_read(spi3_dev, &spi3_cfg, &spi_rx_buffer_set);
	} while (false);

	gpio_pin_set(gpio1_dev, GPIO_1_CS, 1);

	if (err < 0) printk("Read Registers failed: &d\n", err);
}

void spi_write_reg(uint8_t reg, uint8_t data, uint8_t size) {
    int err;

	uint8_t tx_address_buffer[1];
	tx_address_buffer[0] = reg;

	struct spi_buf tx_spi_address_bufs[] = {
		{ .buf = tx_address_buffer, .len = sizeof(tx_address_buffer)},	
	};

	struct spi_buf_set spi_tx_address_buffer_set = {
		.buffers = tx_spi_address_bufs,
		.count = 1
	};

	uint8_t tx_data_buffer[1];
	tx_data_buffer[0] = data;

	struct spi_buf tx_spi_data_bufs[] = {
		{ .buf = tx_data_buffer, .len = sizeof(tx_data_buffer)},	
	};

	struct spi_buf_set spi_tx_data_buffer_set = {
		.buffers = tx_spi_data_bufs,
		.count = 1
	};

	gpio_pin_set(gpio1_dev, GPIO_1_CS, 0);
	do {
		err = spi_write(spi3_dev, &spi3_cfg, &spi_tx_address_buffer_set);
		if (err < 0) break;

		err = spi_write(spi3_dev, &spi3_cfg, &spi_tx_data_buffer_set);
	} while (false);

	gpio_pin_set(gpio1_dev, GPIO_1_CS, 1);

	if (err < 0) printk("Read Registers failed: &d\n", err);
}

void init_IMU() {
	uint8_t reg[1] = {0};
    //output data rate config for gyroscope (952 HZ)
	spi_write_reg(CTRL_REG1_G_WRITE, 0, 1);
	spi_write_reg(CTRL_REG1_G_WRITE, CTRL_REG1_G_952_ODR, 1);
	spi_read_reg(CTRL_REG1_G_READ, reg, 1);
	printk("CTRL_REG1_G reg: %x\n", reg[0]);

    //linear accelerometer sensitivity config (set to +/- 2g), ODR = 952HZ
	spi_write_reg(CTRL_REG6_XL_WRITE, 0, 1);
	spi_write_reg(CTRL_REG6_XL_WRITE, CTRL_REG6_XL_ODR_XL_952, 1);
	spi_read_reg(CTRL_REG6_XL_READ, reg, 1);
	printk("CTRL_REG6_XL reg: %x\n", reg[0]);
}

//enables IMU interrupt generation for certain axes, sets thresholds for desired axes (rn x axis low)
void init_IMU_interrupts() {
	/*
	possibly relevant registers:
	INT_GEN_CFG_XL (06h) - need (enables interrupt generation for certain axes)
	INT_GEN_THS_X/Y/Z_XL (07h) - need (threshold)
	INT_GEN_DUR_XL (0Ah) - don't need (adds wait time after interrupt finishes execution)
	INT1/2_CTRL (0Ch) - might need (interrupt generator for accelerometer, gyroscope, etc.)
	STATUS_REG (17h) - might need (interrupt output signal)
	INT_GEN_SRC_XL (26h) - might need (interrupt x/y/z output); source of interrupts 
                        (e.g. register bits corresponding to axis get set when interrupt fires)
	STATUS_REG (27h): source of interrupts but only for any accelerometer interrupt or gyroscope interrupt

	steps:
	1) enable interrupts for acceleration and/or gyroscope in INT1_CTRL (0Ch)
	2) enable interrupt generation in INT_GEN_CFG_XL (06h) (only enabling x axis for now)
	3) set interrupt threshold in INT_GEN_THS_X/Y/Z_XL
	*/

    //INTERRUPT CONFIG FOR IMU USING IMU REGS

	//set INT_IG_XL in INT1_CTRL (enables interrupt generator on INT1 pin)
	uint8_t reg[1] = {0};
	spi_write_reg(INT1_CTRL_WRITE, 0, 1);
	spi_write_reg(INT1_CTRL_WRITE, INT_IG_XL, 1);
	spi_read_reg(INT1_CTRL_READ, reg, 1);
	printk("INT1_CTRL: %x\n", reg[0]);

	//set XLIE_XL IN INT_GEN_CFG_XL (enables interrupt generation on accelerometer x axis low)
	spi_write_reg(INT_GEN_CFG_XL_WRITE, 0, 1);
	spi_write_reg(INT_GEN_CFG_XL_WRITE, XLIE_XL, 1);
	spi_read_reg(INT_GEN_CFG_XL_READ, reg, 1);
	printk("INT_GEN_CFG_XL: %x\n", reg[0]);

	//set THS_XL_X in INT_GEN_THS_X_XL (sets threshold for x axis of accelerometer)
	spi_write_reg(INT_GEN_THS_X_XL_WRITE, 0, 1);
	spi_write_reg(INT_GEN_THS_X_XL_WRITE, 0b00000001, 1);
	spi_read_reg(INT_GEN_THS_X_XL_READ, reg, 1);
	printk("INT_GEN_THS_X_XL: %x\n", reg[0]);


    //INTERRUPT CONFIG FOR BOARD

    //configure gpio pin connected to interrupt pin (INT1)
	static const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	if (!gpio1) {
        printk("Cannot find!\n");
        return 0;
    }
	int ret = gpio_pin_configure(gpio1, PIN_1_02, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret != 0) {
        printk("Error %d: Could not configure GPIO pin\n", ret);
        return;
	}

	// gpio_init_callback(&gpio_cb, IMU_interrupt_cb, BIT(PIN_1_02));
	// gpio_add_callback(gpio1, &gpio_cb); //code crashes here. TODO: fix this
	ret = gpio_pin_interrupt_configure(gpio1, PIN_1_02, GPIO_INT_EDGE_RISING | GPIO_INT_EDGE_TO_ACTIVE);
	printk("return value of interrupt configure: %x\n", ret);
	
	int value = gpio_pin_get (gpio1, PIN_1_02);
	printk("initial PIN_1_02 value: %x\n", value);
}

//interrupt handler function
void IMU_interrupt_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    interrupt_called = 1;
}

void init_IMU_cs() {
    gpio_pin_configure(gpio1_dev, GPIO_1_CS, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
}

float poll_IMU() {
    if (interrupt_called) {
			// print_unfiltered_readings();
			// printk("max xl_x: %d\n", max_x);
			// printk("interrupt fired\n");
			interrupt_called = 0;
	}

	//Accelerometer
	int8_t rx_buf_xl_x_lower[1];
	int8_t rx_buf_xl_x_upper[1];
	spi_read_reg(ACCEL_X_LOWER, rx_buf_xl_x_lower, 1);
	spi_read_reg(ACCEL_X_UPPER, rx_buf_xl_x_upper, 1);

	int8_t rx_buf_xl_y_lower[1];
	int8_t rx_buf_xl_y_upper[1];
	spi_read_reg(ACCEL_Y_LOWER, rx_buf_xl_y_lower, 1);
	spi_read_reg(ACCEL_Y_UPPER, rx_buf_xl_y_upper, 1);

	int8_t rx_buf_xl_z_lower[1];
	int8_t rx_buf_xl_z_upper[1];
	spi_read_reg(ACCEL_Z_LOWER, rx_buf_xl_z_lower, 1);
	spi_read_reg(ACCEL_Z_UPPER, rx_buf_xl_z_upper, 1);
	
	IMU_readings[XL_X_IND] = ((rx_buf_xl_x_upper[0] << 8 | rx_buf_xl_x_lower[0]) * 0.061) / 1000;
	IMU_readings[XL_Y_IND] = ((rx_buf_xl_y_upper[0] << 8 | rx_buf_xl_y_lower[0]) * 0.061) / 1000;
	IMU_readings[XL_Z_IND] = ((rx_buf_xl_z_upper[0] << 8 | rx_buf_xl_z_lower[0]) * 0.061) / 1000;

	//Gyroscope
	int8_t rx_buf_g_x_lower[1];
	int8_t rx_buf_g_x_upper[1];
	spi_read_reg(G_X_LOWER, rx_buf_g_x_lower, 1);
	spi_read_reg(G_X_UPPER, rx_buf_g_x_upper, 1);

	int8_t rx_buf_g_y_lower[1];
	int8_t rx_buf_g_y_upper[1];
	spi_read_reg(G_Y_LOWER, rx_buf_g_y_lower, 1);
	spi_read_reg(G_Y_UPPER, rx_buf_g_y_upper, 1);

	int8_t rx_buf_g_z_lower[1];
	int8_t rx_buf_g_z_upper[1];
	spi_read_reg(G_Z_LOWER, rx_buf_g_z_lower, 1);
	spi_read_reg(G_Z_UPPER, rx_buf_g_z_upper, 1);

	//method on how to convert raw gyroscope values to dps: https://forum.arduino.cc/t/gyro-raw-to-deg-s-conversion-code-snippet-needed/179022/3
	IMU_readings[G_X_IND] = ((rx_buf_g_x_upper[0] << 8 | rx_buf_g_x_lower[0]) - 2.226) * (8.75 / 1000);
	IMU_readings[G_Y_IND] = ((rx_buf_g_y_upper[0] << 8 | rx_buf_g_y_lower[0]) - 40.722) * (8.75 / 1000);
	IMU_readings[G_Z_IND] = ((rx_buf_g_z_upper[0] << 8 | rx_buf_g_z_lower[0]) - (-209.072)) * (8.75 / 1000);

	// float mag = ema_filter(calc_magnitude(IMU_readings[0], IMU_readings[1], IMU_readings[2]), prev_magnitude);

	//Filter raw IMU readings using SW RC Filter
	float filtered_readings[NUM_AXES];
	for (int i = 0; i < NUM_AXES; i++) {
		//order: xl_x, xl_y, xl_z, g_x, g_y, g_z
		filtered_readings[i] = RCFilter_Update(&lp_filters[i], IMU_readings[i]);
	}

	char xl_x_filtered_buf[10], xl_y_filtered_buf[10], xl_z_filtered_buf[10];
	// snprintf(xl_x_filtered_buf, sizeof(xl_x_filtered_buf), "%f", filtered_readings[XL_X_IND]);
	// snprintf(xl_y_filtered_buf, sizeof(xl_y_filtered_buf), "%f", filtered_readings[XL_Y_IND]);
	// snprintf(xl_z_filtered_buf, sizeof(xl_z_filtered_buf), "%f", filtered_readings[XL_Z_IND]);
	// LOG_INF("xl_x: %s     xl_y: %s     xl_z: %s", xl_x_filtered_buf, xl_y_filtered_buf, xl_z_filtered_buf);

	float mag = calc_magnitude(filtered_readings[XL_X_IND], filtered_readings[XL_Y_IND], filtered_readings[XL_Z_IND]);
	return mag;
}

//helper function to be called in main
void init_RCFilters() {
	for (int i = 0; i < NUM_AXES; i++) {
		RCFilter_Init(&lp_filters[i], cutoff_freqs[i], 0.01f);
	}

	init_butterworth_filter(&filter);
}

void print_unfiltered_readings() {
	char unfiltered_buf[NUM_AXES][10];

	snprintf(unfiltered_buf[XL_X_IND], sizeof(unfiltered_buf[XL_X_IND]), "%f", (float)IMU_readings[XL_X_IND]);
	snprintf(unfiltered_buf[XL_Y_IND], sizeof(unfiltered_buf[XL_Y_IND]), "%f", (float)IMU_readings[XL_Y_IND]);
	snprintf(unfiltered_buf[XL_Z_IND], sizeof(unfiltered_buf[XL_Z_IND]), "%f", (float)IMU_readings[XL_Z_IND]);
	snprintf(unfiltered_buf[G_X_IND], sizeof(unfiltered_buf[G_X_IND]), "%f", (float)IMU_readings[G_X_IND]);
	snprintf(unfiltered_buf[G_Y_IND], sizeof(unfiltered_buf[G_Y_IND]), "%f", (float)IMU_readings[G_Y_IND]);
	snprintf(unfiltered_buf[G_Z_IND], sizeof(unfiltered_buf[G_Z_IND]), "%f", (float)IMU_readings[G_Z_IND]);

	printk("xl_x: %s     xl_y: %s     xl_z: %s     g_x: %s     g_y: %s     g_z: %s\n", 
			unfiltered_buf[XL_X_IND], unfiltered_buf[XL_Y_IND], unfiltered_buf[XL_Z_IND],
			unfiltered_buf[G_X_IND], unfiltered_buf[G_Y_IND], unfiltered_buf[G_Z_IND]);
}

void print_filtered_readings() {
	char filtered_buf[NUM_AXES][10];

	for (int i = 0; i < NUM_AXES; i++) {
		//order: xl_x, xl_y, xl_z, g_x, g_y, g_z
		float filtered_reading = RCFilter_Update(&lp_filters[i], IMU_readings[i]);
		snprintf(filtered_buf[i], sizeof(filtered_buf[i]), "%f", filtered_reading);
	}

	//FILTERED VALUES
	printk("xl_x: %s     xl_y: %s     xl_z: %s     g_x: %s     g_y: %s     g_z: %s\n", 
			filtered_buf[XL_X_IND], filtered_buf[XL_Y_IND], filtered_buf[XL_Z_IND],
			filtered_buf[G_X_IND], filtered_buf[G_Y_IND], filtered_buf[G_Z_IND]);
}

void imu_thread_entry(void *p1, void *p2, void *p3) {
	int* steps = (int*)p1;
	ring_buf samps;
	ring_buf intervals;
	long last_step_time_ms = 0;
	float magnitude = 0;
	char mag_buf[20];
	while (1) {
		magnitude = poll_IMU();
		snprintf(mag_buf, sizeof(mag_buf), "%f", magnitude);
		LOG_INF("magnitude: %s", mag_buf);
		// *steps = *steps + 1;
		if (detect_step(magnitude, &samps, &intervals, &last_step_time_ms)) {
			*steps = *steps + 1;
		}
		k_msleep(SAMPLING_INTERVAL_MS);
	}
}

float calc_magnitude(float x, float y, float z) {
	return (float)sqrt(x * x + y * y + z * z);
}

float ema_filter(float new_mag, float old_mag) {
	return ALPHA * new_mag + (1 - ALPHA) * old_mag;
}

void init_butterworth_filter(ButterworthFilter* filter) {
    filter->x[0] = filter->x[1] = 0.0;
    filter->y[0] = filter->y[1] = 0.0;
}

float butterworth_filter(ButterworthFilter* filter, float input) {
    // Apply the Butterworth filter formula
    float output = A0 * input + A1 * filter->x[0] + A2 * filter->x[1]
                   - B1 * filter->y[0] - B2 * filter->y[1];

    // Update filter state
    filter->x[1] = filter->x[0];
    filter->x[0] = input;
    filter->y[1] = filter->y[0];
    filter->y[0] = output;

    return output;
}

#define BUFFER_SIZE 100
#define MIN_TIME_BETWEEN_STEPS_MS 300  // Minimum time in milliseconds between steps
#define MAX_TIME_BETWEEN_STEPS_MS 1500 // Maximum time in milliseconds between steps
#define RECENT_STEP_COUNT 5  // Number of recent steps to keep track of for interval averaging
#define INITIAL_THRESHOLD 2

// Function to detect steps
bool detect_step(float magnitude, ring_buf* samps, ring_buf* intervals, long* last_step_time_ms) {
	// LOG_INF("Magnitude: %d.%d", (int)magnitude, (int)((magnitude - (int)magnitude)*1000));
	float step_threshold = 1.1;
	//insert reading into samps and calc avg
	insert_val(magnitude, samps);
	float samp_avg = get_rolling_avg(samps);

	uint32_t step_period = k_uptime_get() - *last_step_time_ms;
	long test = 0;

    if (magnitude > step_threshold && step_period > 250) {
		// insert_val(step_period, intervals);
		// float interval_avg = get_rolling_avg(intervals);
		// if (step_period > 0.7 * interval_avg) {
		// 	*last_step_time_ms = k_uptime_get();
		// }
		return true;
	}
	return false;
}