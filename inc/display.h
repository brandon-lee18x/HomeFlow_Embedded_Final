#ifndef DISPLAY_H
#define DISPLAY_H

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>

#define P0_03 3
#define P0_04 4

extern int display_var;
extern const struct device* spi1_dev;
extern const struct device* gpio0_dev;
extern volatile bool cw_detected;
extern volatile bool ccw_detected;

typedef struct {
    float hr;
    int hr_confidence;
    float bos;
    float body_temp;
    int steps;
    float weather_temp;
    float humidity;
    float aqi;
    float pressure;
} sensor_data;

typedef enum {
    HEART_RATE_SCREEN,
    BLOOD_OXYGEN_SCREEN,
    BODY_TEMP_SCREEN,
    ACTIVITY_SCREEN,
    WEATHER_SCREEN,
    WARNING_SCREEN,
    NUM_SCREENS // This should always be the last element
} DisplayState;

// Function to initialize the display
void main_display_init(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t backgroundColor, uint16_t heartColor);

// Function to update the display (could be used for periodic updates or refreshes)
uint16_t main_display_update_test(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t color);

// Set the row address
void st7789v_set_row_address(const struct device *spi_dev, const struct device *gpio_dev, uint16_t start, uint16_t end);

// Set the column address
void st7789v_set_column_address(const struct device *spi_dev, const struct device *gpio_dev, uint16_t start, uint16_t end);

// Resets and prepares the display hardware
void reset_display(const struct device *reset_gpio);

// Sends commands to the display
void st7789v_send_command(const struct device *spi_dev, const struct device *gpio_dev, uint8_t command);

// Sends data to the display
void st7789v_send_data(const struct device *spi_dev, const struct device *gpio_dev, uint8_t *data, size_t len);

// Initializes the ST7789V display controller
void st7789v_init_display(const struct device *spi_dev, const struct device *gpio_dev);

// Draws a filled rectangle on the display
void st7789v_fill_rect(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

// Optimized function to fill a rectangle with a single color
void st7789v_fill_rect_optimized(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

void draw_thicker_heart(const struct device *spi_dev, const struct device *gpio_dev, int centerX, int centerY, int size, uint16_t color, int thickness);

void display_thread_entry(void *p1, void *p2, void *p3);

#endif // DISPLAY_H