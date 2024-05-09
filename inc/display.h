#ifndef DISPLAY_H
#define DISPLAY_H

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

// Function to initialize the display
void main_display_init(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t color);

// Function to update the display (could be used for periodic updates or refreshes)
uint16_t main_display_update_test(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t color);

// Functions related to specific display actions


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

// Write to display memory
void st7789v_write_pixel(const struct device *spi_dev, const struct device *gpio_dev, uint16_t color);

// Draws a pixel at a specified location
void st7789v_draw_pixel(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t color);

// Draws a filled rectangle on the display
void st7789v_fill_rect(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

// Optimized function to fill a rectangle with a single color
void st7789v_fill_rect_optimized(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

#endif // DISPLAY_H