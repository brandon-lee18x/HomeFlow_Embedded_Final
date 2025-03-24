/*
SPI1 pins:
VIN:  vdd
INT1: P102
SCL: PO31
SDA: PO30
CSAG: P107
SDOAG: P108

SPI3 pins:
VIN:  vdd
INT1 (white): P102
CSAG (brown): P107
SCL/SCK (blue): P115
SDA/MOSI (gray): P113
SDOAG/MISO (orange): P114
*/

#ifndef IMU_H
#define IMU_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdbool.h>

#include "rc_filter.h"
#include "ring_buf.h"
#include "mutexes.h"

//relevant registers masks used for IMU
#define CHIPID_REG_READ 0b10001111
#define CTRL_REG1_G_WRITE 0b00010000
#define CTRL_REG1_G_READ 0b10010000
#define ACCEL_X_UPPER 0b10101001
#define ACCEL_X_LOWER 0b10101000
#define ACCEL_Y_UPPER 0b10101011
#define ACCEL_Y_LOWER 0b10101010
#define ACCEL_Z_UPPER 0b10101101
#define ACCEL_Z_LOWER 0b10101100
#define G_X_UPPER 0b10011001
#define G_X_LOWER 0b10011000
#define G_Y_UPPER 0b10011011
#define G_Y_LOWER 0b10011010
#define G_Z_UPPER 0b10011101
#define G_Z_LOWER 0b10011100
#define CTRL_REG1_G_952_ODR 0b11000000
#define INT1_CTRL_READ 0b10001100
#define INT1_CTRL_WRITE 0b00001100
#define INT_IG_XL 0b01000000
#define INT_GEN_CFG_XL_READ 0b10000110
#define INT_GEN_CFG_XL_WRITE 0b00000110
#define XHIE_XL 0b00000010
#define XLIE_XL 0b00000001
#define INT_GEN_THS_X_XL_WRITE 0b00000111
#define INT_GEN_THS_X_XL_READ 0b10000111
#define STATUS_REG_READ 0b10010111
#define INT_GEN_SRC_XL_READ 0b10100110
#define INT_GEN_SRC_XL_WRITE 0b00100110
#define STATUS_REG_READ 0b10010111
#define STATUS_REG_WRITE 0b00010111
#define CTRL_REG6_XL_READ 0b10100000
#define CTRL_REG6_XL_WRITE 0b00100000
#define CTRL_REG6_XL_FS_XL_8G 0b00011000
#define CTRL_REG6_XL_ODR_XL_952 0b11000000
#define FS_XL_4G 0b00010000

#define PIN_1_02 2
#define SPI3_NODE DT_NODELABEL (spi3)
#define MY_GPI01 DT_NODELABEL (gpio1)
#define GPIO_1_CS 7

#define NUM_AXES 6
#define XL_X_IND 0
#define XL_Y_IND 1
#define XL_Z_IND 2
#define G_X_IND 3
#define G_Y_IND 4
#define G_Z_IND 5

#define SAMPLING_INTERVAL_MS 10  // Sampling rate = 1 / SAMPLING_INTERVAL_S
#define ALPHA 0.1

#define A0 0.020083365564211235
#define A1 0.04016673112842247
#define A2 0.020083365564211235
#define B1 -1.5610180758007182
#define B2 0.6413515380575631

// Butterworth filter state
typedef struct {
    float x[2];  // Input samples
    float y[2];  // Output samples
} ButterworthFilter;

extern const struct device *spi3_dev;
extern struct spi_config spi3_cfg;
extern const struct device* gpio0_dev;
extern const struct device *gpio1_dev;
// extern struct gpio_callback gpio_cb;

//order of indices: xl_x, xl_y, xl_z, g_x, g_y, g_z
extern RCFilter lp_filters[NUM_AXES]; 
extern float cutoff_freqs[NUM_AXES];

extern volatile float IMU_readings[NUM_AXES]; //sensor readings in g's and dps
extern volatile int interrupt_called;
extern volatile int max_x;
extern volatile float filtered_imu_data[3];
extern volatile float threshold;

void spi_read_reg(uint8_t reg, uint8_t values[], uint8_t size);
void spi_write_reg(uint8_t reg, uint8_t data, uint8_t size);
void init_IMU();
void init_IMU_interrupts();
void IMU_interrupt_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void init_IMU_cs();
float poll_IMU(); //returns magnitude of IMU readings
void init_RCFilters();
void imu_thread_entry(void *p1, void *p2, void *p3);
float calc_magnitude(float x, float y, float z);
float ema_filter(float new_mag, float old_mag);
void init_butterworth_filter(ButterworthFilter* filter);
float butterworth_filter(ButterworthFilter* filter, float input);
bool detect_step(float magnitude, ring_buf* samps, ring_buf* intervals, long* last_step_time_ms);

#endif