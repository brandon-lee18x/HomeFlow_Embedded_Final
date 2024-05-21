#include "../inc/display.h"
#include "../inc/spi.h"


#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <zephyr/types.h>
#include <stdlib.h>
#include <time.h>



#include <sys/time.h>
#include <stdio.h>

LOG_MODULE_REGISTER(display);

#define SLEEP_TIME_MS_D   100

// Update these definitions according to your pin connections
#define CS_GPIO_PORT_D    DT_LABEL(DT_NODELABEL(gpio0))
#define CS_GPIO_PIN_D     16

#define DC_GPIO_PORT_D    DT_LABEL(DT_NODELABEL(gpio0))
#define DC_GPIO_PIN_D     14

#define RESET_GPIO_PORT_D DT_LABEL(DT_NODELABEL(gpio0))
#define RESET_GPIO_PIN_D  24

#define ST7789V_CMD_SWRESET_D            0x01
#define ST7789V_CMD_SLPOUT_D             0x11
#define ST7789V_CMD_COLMOD_D             0x3A
#define ST7789V_CMD_DISPON_D             0x29

#define ST7789V_CMD_CASET_D 0x2A // Column address set
#define ST7789V_CMD_RASET_D 0x2B // Row address set
#define ST7789V_CMD_RAMWR_D 0x2C // Memory write

#define FONT_WIDTH_D 5
#define FONT_HEIGHT_D 10
#define SCALE_D 3

#define DISPLAY_WIDTH_D 240
#define DISPLAY_HEIGHT_D 320


void reset_display(const struct device *reset_gpio) 
{
    gpio_pin_set(reset_gpio, RESET_GPIO_PIN_D, 1);
    k_sleep(K_MSEC(10));
    gpio_pin_set(reset_gpio, RESET_GPIO_PIN_D, 0);
    k_sleep(K_MSEC(10));
    gpio_pin_set(reset_gpio, RESET_GPIO_PIN_D, 1);
    k_sleep(K_MSEC(120));
}

void st7789v_set_column_address(const struct device *spi_dev, const struct device *gpio_dev, uint16_t start, uint16_t end) {
    uint8_t command = ST7789V_CMD_CASET_D;
    uint8_t data[] = {
        start >> 8, start & 0xFF,
        end >> 8, end & 0xFF
    };

    st7789v_send_command(spi_dev, gpio_dev, command);
    st7789v_send_data(spi_dev, gpio_dev, data, sizeof(data));
}

// Define the function to set the row address
void st7789v_set_row_address(const struct device *spi_dev, const struct device *gpio_dev, uint16_t start, uint16_t end) {
    uint8_t command = ST7789V_CMD_RASET_D;
    uint8_t data[] = {
        start >> 8, start & 0xFF,
        end >> 8, end & 0xFF
    };

    st7789v_send_command(spi_dev, gpio_dev, command);
    st7789v_send_data(spi_dev, gpio_dev, data, sizeof(data));
}

void st7789v_fill_rect(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    uint8_t data[4];

    // Set column address (X)
    data[0] = x >> 8; data[1] = x & 0xFF; // Start column
    data[2] = (x + width - 1) >> 8; data[3] = (x + width - 1) & 0xFF; // End column
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_CASET_D);
    st7789v_send_data(spi_dev, gpio_dev, data, 4);

    // Set row address (Y)
    data[0] = y >> 8; data[1] = y & 0xFF; // Start row
    data[2] = (y + height - 1) >> 8; data[3] = (y + height - 1) & 0xFF; // End row
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_RASET_D);
    st7789v_send_data(spi_dev, gpio_dev, data, 4);

    // Write color to GRAM
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_RAMWR_D);

    // Prepare color data
    uint8_t color_data[2] = {color >> 8, color & 0xFF};

    // Fill the rectangle with the color
    for (uint32_t i = 0; i < height; i++) {
        for (uint32_t j = 0; j < width; j++) {
            st7789v_send_data(spi_dev, gpio_dev, color_data, 2);
        
        }
    }
}


void  st7789v_send_command(const struct device *spi_dev, const struct device *dev, uint8_t command)
{
    gpio_pin_set(dev, DC_GPIO_PIN_D, 0); // Command mode
    gpio_pin_set(dev, CS_GPIO_PIN_D, 0); // Chip select active
    spi_write1(spi_dev, &command, sizeof(command), spi_cfg);
    gpio_pin_set(dev, CS_GPIO_PIN_D, 1); // Chip select inactive
    gpio_pin_set(dev, DC_GPIO_PIN_D, 1);
}

void  st7789v_send_data(const struct device *spi_dev, const struct device *dev, uint8_t *data, size_t len)
{
    gpio_pin_set(dev, DC_GPIO_PIN_D, 1); // Data mode
    gpio_pin_set(dev, CS_GPIO_PIN_D, 0); // Chip select active
    spi_write1(spi_dev, data, len, spi_cfg);
    gpio_pin_set(dev, CS_GPIO_PIN_D, 1); // Chip select inactive
}

void st7789v_init_display(const struct device *spi_dev, const struct device *gpio_dev)
{
    reset_display(gpio_dev);

    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_SWRESET_D);
    k_sleep(K_MSEC(150)); // Datasheet may specify a different time, adjust if needed

    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_SLPOUT_D);
    k_sleep(K_MSEC(500)); // Datasheet may specify a different time, adjust if needed

    // Memory Data Access Control
    uint8_t madctl = 0x00; // Modify according to your display orientation and RGB/BGR mode
    st7789v_send_command(spi_dev, gpio_dev, 0x36);
    st7789v_send_data(spi_dev, gpio_dev, &madctl, sizeof(madctl));

    // Interface Pixel Formatx
    uint8_t colmod = 0x55; // 16-bits per pixel, change if you need a different format
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_COLMOD_D);
    st7789v_send_data(spi_dev, gpio_dev, &colmod, sizeof(colmod));

    // Display Inversion On: may be necessary depending on the display
    st7789v_send_command(spi_dev, gpio_dev, 0x21);

    // st7789v_set_brightness(spi_dev, gpio_dev, desired_brightness)

    // Turn on the display
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_DISPON_D);
    k_sleep(K_MSEC(100)); // Give time for the display to turn on
}

void st7789v_fill_rect_optimized(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    uint8_t command;
    uint8_t data[4];

    // Calculate the end positions
    uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;

    // Set column address
    command = ST7789V_CMD_CASET_D;
    data[0] = x >> 8;
    data[1] = x & 0xFF;
    data[2] = x_end >> 8;
    data[3] = x_end & 0xFF;
    st7789v_send_command(spi_dev, gpio_dev, command);
    st7789v_send_data(spi_dev, gpio_dev, data, 4);

    // Set row address
    command = ST7789V_CMD_RASET_D;
    data[0] = y >> 8;
    data[1] = y & 0xFF;
    data[2] = y_end >> 8;
    data[3] = y_end & 0xFF;
    st7789v_send_command(spi_dev, gpio_dev, command);
    st7789v_send_data(spi_dev, gpio_dev, data, 4);

    // Write to RAM
    command = ST7789V_CMD_RAMWR_D;
    st7789v_send_command(spi_dev, gpio_dev, command);

    // Prepare the buffer for row data
    uint16_t pixels_per_row = width;
    uint8_t row_data[pixels_per_row * 2];  // 2 bytes per pixel

    for (uint16_t i = 0; i < pixels_per_row; i++) {
        row_data[2*i] = color >> 8;
        row_data[2*i + 1] = color & 0xFF;
    }

    // Send row data repeatedly for each row
    for (uint16_t i = 0; i < height; i++) {
        st7789v_send_data(spi_dev, gpio_dev, row_data, sizeof(row_data));
    }
}

void main_display_init(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t backgroundColor, uint16_t heartColor){

    // Configure pins for use with the LCD display
    gpio_pin_configure(gpio0_dev, CS_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);

    gpio_pin_configure(gpio0_dev, DC_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
	
    gpio_pin_configure(gpio0_dev, RESET_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
	
    // Initialize the display
    st7789v_init_display(spi1_dev, gpio0_dev);
	
	st7789v_fill_rect_optimized(spi1_dev, gpio0_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, backgroundColor); // Fills entire display with blue color

    draw_thicker_heart(spi1_dev, gpio0_dev, 120, 160, 400, heartColor, 2); 
	draw_thicker_heart(spi1_dev, gpio0_dev, 120, 160, 400, backgroundColor, 2);
}

// Function to update the display (could be used for periodic updates or refreshes)
uint16_t main_display_update_test(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t color){
    st7789v_fill_rect_optimized(spi1_dev, gpio0_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, color); // Fills entire display with blue color
    return color += 0x1111;
}

const uint8_t font[85][5] = {
    // Digits 0-9
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x03, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    // Uppercase Alphabet A-Z
    {0x7E, 0x09, 0x09, 0x09, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    // Lowercase letters
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x50, 0x48, 0x48, 0x30}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x30, 0x48, 0x48, 0x50, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x78, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (index: 62)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // ! (index: 63)
    {0x00, 0x03, 0x00, 0x03, 0x00}, // " (index: 64)
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // # (index: 65)
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $ (index: 66)
    {0x23, 0x13, 0x08, 0x64, 0x62}, // % (index: 67)
    {0x36, 0x49, 0x56, 0x20, 0x50}, // & (index: 68)
    {0x00, 0x05, 0x03, 0x00, 0x00}, // ' (index: 69)
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // ( (index: 70)
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ) (index: 71)
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // * (index: 72)
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // + (index: 73)
    {0x00, 0x50, 0x30, 0x00, 0x00}, // , (index: 74)
    {0x08, 0x08, 0x08, 0x08, 0x08}, // - (index: 75)
    {0x00, 0x60, 0x60, 0x00, 0x00}, // . (index: 76)
    {0x20, 0x10, 0x08, 0x04, 0x02}, // / (index: 77)
    {0x00, 0x66, 0x00, 0x00, 0x00}, // : (index: 78)
    {0x00, 0x18, 0x00, 0x50, 0x30}, // ; (index: 79) --- CURRENTLY SEMICOLON IS NOT RIGHT ---
    {0x08, 0x14, 0x22, 0x41, 0x00}, // < (index: 80)
    {0x14, 0x14, 0x14, 0x14, 0x14}, // = (index: 81)
    {0x00, 0x41, 0x22, 0x14, 0x08}, // > (index: 82)
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ? (index: 83)
    {0x0E, 0x0A, 0x0A, 0x0E, 0x00}  // ° (index: 84)

};

static char last_displayed_number[12] = "";  // Assuming maximum of 11 digits + null for integers

// Global variable to track the maximum width ever drawn
static uint16_t max_width_num_drawn = 0;

void draw_number(const struct device *spi_dev, const struct device *gpio_dev, int number, uint16_t x, uint16_t y, uint16_t color, uint16_t bg_color, int scale) {
    char num_str[12];  // Buffer to hold the maximum digits for an int and a null character
    snprintf(num_str, sizeof(num_str), "%d", number);  // Convert the number to a string
    size_t num_len = strlen(num_str);

    // Calculate the width for the current number, including spacing between digits
    uint16_t currentWidth = (FONT_WIDTH_D * scale + scale) * num_len - scale;  // Subtract `scale` to avoid extra space after last digit
    uint16_t totalHeight = FONT_HEIGHT_D * scale;

    // Determine the width to clear based on the maximum width drawn previously
    static uint16_t max_width_num_drawn = 0;
    uint16_t widthToClear = max_width_num_drawn > currentWidth ? max_width_num_drawn : currentWidth;
    if (currentWidth > max_width_num_drawn) {
        max_width_num_drawn = currentWidth; // Update max_width_drawn if the current number is wider
    }

    // Allocate a buffer to hold the pixel data for clearing and drawing
    uint8_t *pixel_buffer = (uint8_t *)malloc(widthToClear * totalHeight * 2);
    if (!pixel_buffer) return; // Failed to allocate memory

    // Initialize the buffer with the background color
    for (uint32_t i = 0; i < widthToClear * totalHeight; i++) {
        pixel_buffer[i * 2] = bg_color >> 8;
        pixel_buffer[i * 2 + 1] = bg_color & 0xFF;
    }

    // Draw digits into the buffer, but only within the width necessary for the current number
    for (size_t digit_index = 0; digit_index < num_len; digit_index++) {
        char current_digit = num_str[digit_index] - '0';
        uint16_t digitXOffset = (FONT_WIDTH_D * scale + scale) * digit_index;  // Add `scale` for space between digits
        for (int row = 0; row < FONT_HEIGHT_D; row++) {
            for (int col = 0; col < FONT_WIDTH_D; col++) {
                if (font[current_digit][col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++) {
                        for (int dx = 0; dx < scale; dx++) {
                            int buffer_index = (((row * scale + dy) * widthToClear) + (digitXOffset + col * scale + dx)) * 2;
                            pixel_buffer[buffer_index] = color >> 8;
                            pixel_buffer[buffer_index + 1] = color & 0xFF;
                        }
                    }
                }
            }
        }
    }

    // Set column and row addresses to cover the maximum area ever used
    st7789v_set_column_address(spi_dev, gpio_dev, x, x + widthToClear - 1);
    st7789v_set_row_address(spi_dev, gpio_dev, y, y + totalHeight - 1);
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_RAMWR_D);

    // Send the whole buffer
    st7789v_send_data(spi_dev, gpio_dev, pixel_buffer, widthToClear * totalHeight * 2);

    // Free the buffer
    free(pixel_buffer);
}

static uint16_t max_width_num_drawn_centered = 0;

void draw_number_center(const struct device *spi_dev, const struct device *gpio_dev, int number, uint16_t x, uint16_t y, uint16_t color, uint16_t bg_color, int scale) {
    char num_str[12];  // Buffer to hold the maximum digits for an int and a null character
    snprintf(num_str, sizeof(num_str), "%d", number);  // Convert the number to a string
    size_t num_len = strlen(num_str);

    // Calculate the width for the current number, including spacing between digits
    uint16_t currentWidth = (FONT_WIDTH_D * scale + scale) * num_len - scale;  // Subtract `scale` to avoid extra space after last digit
    uint16_t totalHeight = FONT_HEIGHT_D * scale;

    // Determine the width to clear based on the maximum width drawn previously
    uint16_t widthToClear = max_width_num_drawn_centered > currentWidth ? max_width_num_drawn_centered : currentWidth;
    if (currentWidth > max_width_num_drawn_centered) {
        max_width_num_drawn_centered = currentWidth; // Update max_width_drawn if the current number is wider
    }

    // Allocate a buffer to hold the pixel data for clearing and drawing
    uint8_t *pixel_buffer = (uint8_t *)malloc(widthToClear * totalHeight * 2);
    if (!pixel_buffer) return; // Failed to allocate memory

    // Initialize the buffer with the background color
    for (uint32_t i = 0; i < widthToClear * totalHeight; i++) {
        pixel_buffer[i * 2] = bg_color >> 8;
        pixel_buffer[i * 2 + 1] = bg_color & 0xFF;
    }

    // Adjust the x and y positions to center the number at (x, y)
    uint16_t centeredX = x - (currentWidth / 2);
    uint16_t centeredY = y - (totalHeight / 2);

    // Draw digits into the buffer, but only within the width necessary for the current number
    for (size_t digit_index = 0; digit_index < num_len; digit_index++) {
        char current_digit = num_str[digit_index] - '0';
        uint16_t digitXOffset = (FONT_WIDTH_D * scale + scale) * digit_index;  // Adjust within buffer
        for (int row = 0; row < FONT_HEIGHT_D; row++) {
            for (int col = 0; col < FONT_WIDTH_D; col++) {
                if (font[current_digit][col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++) {
                        for (int dx = 0; dx < scale; dx++) {
                            int buffer_index = (((row * scale + dy) * widthToClear) + (digitXOffset + col * scale + dx)) * 2;
                            pixel_buffer[buffer_index] = color >> 8;
                            pixel_buffer[buffer_index + 1] = color & 0xFF;
                        }
                    }
                }
            }
        }
    }

    // Set column and row addresses to cover the maximum area ever used
    st7789v_set_column_address(spi_dev, gpio_dev, centeredX, centeredX + widthToClear - 1);
    st7789v_set_row_address(spi_dev, gpio_dev, centeredY, centeredY + totalHeight - 1);
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_RAMWR_D);

    // Send the whole buffer
    st7789v_send_data(spi_dev, gpio_dev, pixel_buffer, widthToClear * totalHeight * 2);

    // Free the buffer
    free(pixel_buffer);
}


void draw_number_decimal(const struct device *spi_dev, const struct device *gpio_dev, int number, uint16_t x, uint16_t y, uint16_t color, uint16_t bg_color, int scale) {
    char num_str[12];  // Buffer to hold the maximum digits for an int and a null character
    snprintf(num_str, sizeof(num_str), "%d", number);  // Convert the number to a string
    size_t num_len = strlen(num_str);

    // Calculate the width for the current number, including spacing between digits
    uint16_t currentWidth = (FONT_WIDTH_D * scale + scale) * num_len - scale;  // Subtract `scale` to avoid extra space after last digit
    uint16_t totalHeight = FONT_HEIGHT_D * scale;

    // Determine the width to clear based on the maximum width drawn previously
    static uint16_t max_width_num_drawn = 0;
    uint16_t widthToClear = currentWidth;

    // Allocate a buffer to hold the pixel data for clearing and drawing
    uint8_t *pixel_buffer = (uint8_t *)malloc(widthToClear * totalHeight * 2);
    if (!pixel_buffer) return; // Failed to allocate memory

    // Initialize the buffer with the background color
    for (uint32_t i = 0; i < widthToClear * totalHeight; i++) {
        pixel_buffer[i * 2] = bg_color >> 8;
        pixel_buffer[i * 2 + 1] = bg_color & 0xFF;
    }

    // Draw digits into the buffer, but only within the width necessary for the current number
    for (size_t digit_index = 0; digit_index < num_len; digit_index++) {
        char current_digit = num_str[digit_index] - '0';
        uint16_t digitXOffset = (FONT_WIDTH_D * scale + scale) * digit_index;  // Add `scale` for space between digits
        for (int row = 0; row < FONT_HEIGHT_D; row++) {
            for (int col = 0; col < FONT_WIDTH_D; col++) {
                if (font[current_digit][col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++) {
                        for (int dx = 0; dx < scale; dx++) {
                            int buffer_index = (((row * scale + dy) * widthToClear) + (digitXOffset + col * scale + dx)) * 2;
                            pixel_buffer[buffer_index] = color >> 8;
                            pixel_buffer[buffer_index + 1] = color & 0xFF;
                        }
                    }
                }
            }
        }
    }

    // Set column and row addresses to cover the maximum area ever used
    st7789v_set_column_address(spi_dev, gpio_dev, x, x + widthToClear - 1);
    st7789v_set_row_address(spi_dev, gpio_dev, y, y + totalHeight - 1);
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_RAMWR_D);

    // Send the whole buffer
    st7789v_send_data(spi_dev, gpio_dev, pixel_buffer, widthToClear * totalHeight * 2);

    // Free the buffer
    free(pixel_buffer);
}


static char last_displayed_text[128] = "";  // Assuming a reasonable max length for text


void draw_text(const struct device *spi_dev, const struct device *gpio_dev, const char *text, uint16_t x, uint16_t y, uint16_t color, uint16_t bg_color, int scale) {
    size_t text_len = strlen(text);

    // Calculate the width for the current text, including spacing between characters
    uint16_t currentWidth = (FONT_WIDTH_D * scale + scale) * text_len - scale;  // Subtract `scale` to avoid extra space after the last character
    uint16_t totalHeight = FONT_HEIGHT_D * scale;

    // Allocate a buffer to hold the pixel data for clearing and drawing
    uint8_t *pixel_buffer = (uint8_t *)malloc(currentWidth * totalHeight * 2);
    if (!pixel_buffer) return; // Failed to allocate memory

    // Initialize the buffer with the background color
    for (uint32_t i = 0; i < currentWidth * totalHeight; i++) {
        pixel_buffer[i * 2] = bg_color >> 8;
        pixel_buffer[i * 2 + 1] = bg_color & 0xFF;
    }
    // Draw text into the buffer
    for (size_t char_index = 0; char_index < text_len; char_index++) {
        char current_char = text[char_index];
        uint8_t index;
        if (current_char >= '0' && current_char <= '9') {
            index = current_char - '0';
        } else if (current_char >= 'A' && current_char <= 'Z') {
            index = 10 + (current_char - 'A');
        } else if (current_char >= 'a' && current_char <= 'z') {
            index = 36 + (current_char - 'a');
        } else {
            switch (current_char) {
                case ' ': index = 62; break;
                case '!': index = 63; break;
                case '"': index = 64; break;
                case '#': index = 65; break;
                case '$': index = 66; break;
                case '%': index = 67; break;
                case '&': index = 68; break;
                case '\'': index = 69; break;
                case '(': index = 70; break;
                case ')': index = 71; break;
                case '*': index = 72; break;
                case '+': index = 73; break;
                case ',': index = 74; break;
                case '-': index = 75; break;
                case '.': index = 76; break;
                case '/': index = 77; break;
                case ':': index = 78; break;
                //case ';': index = 79; break;
                case '<': index = 80; break;
                case '=': index = 81; break;
                case '>': index = 82; break;
                case '?': index = 83; break;
                case '@': index = 84; break;
                default:  index = 62; // Default to space if unknown
            }
        }
        uint16_t charXOffset = (FONT_WIDTH_D * scale + scale) * char_index;

        for (int row = 0; row < FONT_HEIGHT_D; row++) {
            for (int col = 0; col < FONT_WIDTH_D; col++) {
                if (font[index][col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++) {
                        for (int dx = 0; dx < scale; dx++) {
                            int buffer_index = (((row * scale + dy) * currentWidth) + (charXOffset + col * scale + dx)) * 2;
                            pixel_buffer[buffer_index] = color >> 8;
                            pixel_buffer[buffer_index + 1] = color & 0xFF;
                        }
                    }
                }
            }
        }
    }

    // Set column and row addresses to cover the text area
    st7789v_set_column_address(spi_dev, gpio_dev, x, x + currentWidth - 1);
    st7789v_set_row_address(spi_dev, gpio_dev, y, y + totalHeight - 1);
    st7789v_send_command(spi_dev, gpio_dev, ST7789V_CMD_RAMWR_D);

    // Send the whole buffer
    st7789v_send_data(spi_dev, gpio_dev, pixel_buffer, currentWidth * totalHeight * 2);

    // Free the buffer
    free(pixel_buffer);

    // Update the last displayed text
    strncpy(last_displayed_text, text, sizeof(last_displayed_text) - 1);
}


void setup_heartrate_screen(const struct device *spi_dev, const struct device *gpio_dev) {
    // Clear the screen by filling it with black color (assuming black is 0x0000)
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, 0x0000);
    
    uint16_t heartColor = 0xFE60;
    const char *health_text = "Health";
    uint16_t health_text_x = 160;
    uint16_t health_text_y = 300;
    draw_text(spi_dev, gpio_dev, health_text, health_text_x, health_text_y, heartColor, 0x0000, 2); // White text, black background

    // Draw the "Heart rate" text
    const char *heart_text = "Heart";
    uint16_t heart_text_x = 20;
    uint16_t heart_text_y = 50;
    draw_text(spi_dev, gpio_dev, heart_text, heart_text_x, heart_text_y, 0xFFFF, 0x0000, 3); // White text, black background

    const char *rate_text = "Rate";
    uint16_t rate_text_x = 28;
    uint16_t rate_text_y = 80;
    draw_text(spi_dev, gpio_dev, rate_text, rate_text_x, rate_text_y, 0xFFFF, 0x0000, 3); // White text, black background

    const char *bpm_text = "(BPM)";
    uint16_t bpm_text_x = 31;
    uint16_t bpm_text_y = 110;
    draw_text(spi_dev, gpio_dev, bpm_text, bpm_text_x, bpm_text_y, 0xFFFF, 0x0000, 2); // White text, black background

    // Draw the "Blood Oxygen" text
    const char *blood_text = "Blood";
    uint16_t blood_text_x = 20;
    uint16_t blood_text_y = 210; 
    draw_text(spi_dev, gpio_dev, blood_text, blood_text_x, blood_text_y, 0xFFFF, 0x0000, 3);

    const char *oxygen_text = "oxygen";
    uint16_t oxygen_text_x = 10;
    uint16_t oxygen_text_y = 240; 
    draw_text(spi_dev, gpio_dev, oxygen_text, oxygen_text_x, oxygen_text_y, 0xFFFF, 0x0000, 3);

    const char *percent_text = "(%)";
    uint16_t percent_text_x = 45;
    uint16_t percent_text_y = 270; 
    draw_text(spi_dev, gpio_dev, percent_text, percent_text_x, percent_text_y, 0xFFFF, 0x0000, 2);

    
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 25, 167, 127, 3, heartColor); // Draw a horizontal line
    draw_thicker_heart(spi_dev, gpio_dev, 175, 164, 190, heartColor, 1); 
}

void update_heart_rate(const struct device *spi_dev, const struct device *gpio_dev, int heart_rate) {
    uint16_t x =  140; // Assuming 3 digits
    uint16_t y = 70;   // Below the "Heartrate" label
    char num_str[4];
   // snprintf(num_str, sizeof(num_str), "%d", heart_rate);
    draw_number(spi_dev, gpio_dev, heart_rate, x, y,  0xFFFF, 0x0000, 5);
}

void update_blood_oxygen(const struct device *spi_dev, const struct device *gpio_dev, int blood_oxygen) {
    uint16_t x = 140; // Assuming 3 digits
    uint16_t y = 230; // Below the "Blood Oxygen" label
    char num_str[4];
    //snprintf(num_str, sizeof(num_str), "%d", blood_oxygen);
    draw_number(spi_dev, gpio_dev, blood_oxygen, x, y,  0xFFFF, 0x0000, 5);
}

// Draw a heart with a thicker outline
void draw_thicker_heart(const struct device *spi_dev, const struct device *gpio_dev, int centerX, int centerY, int size, uint16_t color, int thickness) {
    // Heart parametric equations multiplier
    float t;
    int x, y, dx, dy;
    float scale = size / 100.0;  // Normalize size based on 100 pixels

    for (t = 0; t < 2 * 3.14159; t += 0.01) {
        x = (int)(16 * pow(sin(t), 3) * scale);
        y = (int)(-1 * (13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t)) * scale);

        // Draw thicker outline by iterating through offsets
        for (dx = -thickness; dx <= thickness; dx++) {
            for (dy = -thickness; dy <= thickness; dy++) {
                // Check if the point is within the circular outline thickness
                if (sqrt(dx * dx + dy * dy) <= thickness) {
                    st7789v_fill_rect_optimized(spi_dev, gpio_dev, centerX + x + dx, centerY + y + dy, 1, 1, color);
                }
            }
        }
    }
}


// Helper function to get the ordinal suffix for a given day
const char *get_ordinal_suffix(int day) {
    if (day % 10 == 1 && day != 11) {
        return "st";
    } else if (day % 10 == 2 && day != 12) {
        return "nd";
    } else if (day % 10 == 3 && day != 13) {
        return "rd";
    } else {
        return "th";
    }
}

void get_formatted_time_and_date(char *buffer, size_t buf_len) {
    struct timespec ts;
    struct tm *tm;
    clock_gettime(CLOCK_REALTIME, &ts);  // Get current time
    tm = gmtime(&ts.tv_sec);             // Convert to struct tm

    // Month abbreviations
    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    
    // Format AM or PM
    const char *ampm = (tm->tm_hour >= 12) ? "PM" : "AM";

    // Convert 24h to 12h format
    int hour12 = tm->tm_hour % 12;
    if (hour12 == 0) hour12 = 12;  // Adjust for 12 AM or 12 PM

    const char *ordinal_suffix = get_ordinal_suffix(tm->tm_mday);

    // Print formatted time and date into buffer
    snprintf(buffer, buf_len, "%s %d%s %02d:%02d %s", 
             months[tm->tm_mon], tm->tm_mday, ordinal_suffix, hour12, tm->tm_min, ampm);
}

// Function to display the time and date
void display_time_and_date(const struct device *spi_dev, const struct device *gpio_dev) {
    char datetime[32];
    uint16_t x = 20;  // X coordinate for the text
    uint16_t y = 5;  // Y coordinate for the text
    uint16_t color = 0xFFFF;  // White color
    uint16_t bg_color = 0x0000;  // Black background

    // Get formatted date and time
    get_formatted_time_and_date(datetime, sizeof(datetime));

    // Use your existing function to draw text
    draw_text(spi_dev, gpio_dev, datetime, x, y, color, bg_color, 2); // Assume '2' is the desired scale
}



// Function to draw a single circle layer with a given color
void draw_circle_layer(const struct device *spi_dev, const struct device *gpio_dev, int16_t cx, int16_t cy, int16_t r, uint16_t color) {
    int x = r, y = 0;
    int err = 0;

    while (x >= y) {
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + x, cy + y, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + y, cy + x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - y, cy + x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - x, cy + y, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - x, cy - y, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - y, cy - x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + y, cy - x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + x, cy - y, 1, 1, color);

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }

        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

// Function to draw a thicker circle using concentric layers
void draw_thicker_circle(const struct device *spi_dev, const struct device *gpio_dev, int16_t cx, int16_t cy, int16_t outer_radius, int16_t thickness, uint16_t color) {
    int16_t inner_radius = outer_radius - thickness + 1;

    // Draw concentric circles from inner_radius to outer_radius
    for (int16_t r = outer_radius; r >= inner_radius; r--) {
        draw_circle_layer(spi_dev, gpio_dev, cx, cy, r, color);
    }
}

// Function to create a solid-colored circle with a specified thickness
void create_solid_circle(const struct device *spi_dev, const struct device *gpio_dev, int16_t cx, int16_t cy, int16_t radius, int16_t thickness, uint16_t color) {
    draw_thicker_circle(spi_dev, gpio_dev, cx, cy, radius, thickness, color);
}

void setup_bodytemp_screen(const struct device *spi_dev, const struct device *gpio_dev) {
    // Clear the screen by filling it with black color (assuming black is 0x0000)
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, 0x0000);

    draw_thicker_circle(spi_dev, gpio_dev, 120, 160, 95, 8, 0xFE60);

    uint16_t bodyTempColor = 0xFE60;
    const char *health_text = "Health";
    uint16_t health_text_x = 160;
    uint16_t health_text_y = 300;
    draw_text(spi_dev, gpio_dev, health_text, health_text_x, health_text_y, bodyTempColor, 0x0000, 2); // White text, black background

    // Draw the "Body Temperature" text
    const char *body_text = "Body Temp";
    uint16_t body_text_x = 40;
    uint16_t body_text_y = 30;
    draw_text(spi_dev, gpio_dev, body_text, body_text_x, body_text_y, 0xFFFF, 0x0000, 3); // White text, black background

    const char *degree_text = "@";
    uint16_t degree_text_x = 165;
    uint16_t degree_text_y = 145;
    draw_text(spi_dev, gpio_dev, degree_text, degree_text_x, degree_text_y, 0xFFFF, 0x0000, 2); // White text, black background

    const char *celsius_text = "F";
    uint16_t celsius_text_x = 180;
    uint16_t celsius_text_y = 152;
    draw_text(spi_dev, gpio_dev, celsius_text, celsius_text_x, celsius_text_y, 0xFFFF, 0x0000, 3); // White text, black background
    
    // const char *decimal_text = ".";
    // uint16_t decimal_text_x = 115;
    // uint16_t decimal_text_y = 153;
    // draw_text(spi_dev, gpio_dev, decimal_text, decimal_text_x, decimal_text_y, 0xFFFF, 0x0000, 3); // White text, black background

}
int prev_temp = 0;
int prev_status = -1;

// Define green, orange, and red
uint16_t greenColor = 0x07E0;  // Green in RGB565 (R: 0, G: 63, B: 0)
uint16_t orangeColor = 0xFD20; // Orange in RGB565 (R: 31, G: 40, B: 0)
uint16_t redColor = 0xF800;    // Red in RGB565 (R: 31, G: 0, B: 0)

void update_body_temp(const struct device *spi_dev, const struct device *gpio_dev, int body_temp, int body_temp_decimal) {
    
    uint16_t x = 45; 
    uint16_t y = 145; 
    char num_str[6];
    int current_status = -1;
    uint16_t color = 0xFFFF;
    //snprintf(num_str, sizeof(num_str), "%d", blood_oxygen);
   if (body_temp < 100){
       if (!(prev_temp < 100)){
            draw_number(spi_dev, gpio_dev, prev_temp, 45, 145,  0x0000, 0x0000, 4);
        }
        draw_number_decimal(spi_dev, gpio_dev, body_temp, 69, 145,  0xFFFF, 0x0000, 4);
   } else{
        draw_number(spi_dev, gpio_dev, body_temp, 45, 145,  0xFFFF, 0x0000, 4);
   }
    draw_number_decimal(spi_dev, gpio_dev, body_temp_decimal, 133, 145,  0xFFFF, 0x0000, 4);
    
    if (body_temp < 95){
        current_status = 2;
        color = redColor;
    } else if ((body_temp < 96) || (body_temp == 96 && body_temp_decimal < 5)){
        current_status = 1;
        color = orangeColor;
    } else if ((body_temp < 99) || (body_temp == 99) && body_temp_decimal < 2){
        current_status = 0;
        color = greenColor;
    } else if (body_temp < 100){
        current_status = 1;
        color = orangeColor;
    } else{
        current_status = 2;
        color = redColor;
    }

    const char *decimal_text = ".";
    uint16_t decimal_text_x = 115;
    uint16_t decimal_text_y = 153;
    draw_text(spi_dev, gpio_dev, decimal_text, decimal_text_x, decimal_text_y, 0xFFFF, 0x0000, 3); // White text, black background

    if (current_status != prev_status){
        draw_thicker_circle(spi_dev, gpio_dev, 120, 160, 95, 8, color);
    }

    prev_temp = body_temp;
    prev_status = current_status;
    

}

void setup_activity_screen(const struct device *spi_dev, const struct device *gpio_dev) {
    // Clear the screen by filling it with black color (assuming black is 0x0000)
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, 0x0000);


    uint16_t bodyTempColor = 0xFE60;
    const char *activity_text = "Activity";
    uint16_t activity_text_x = 120;
    uint16_t activity_text_y = 300;
    draw_text(spi_dev, gpio_dev, activity_text, activity_text_x, activity_text_y, bodyTempColor, 0x0000, 2); // White text, black background

    // Draw the "Body Temperature" text
    const char *steps_text = "Total Steps";
    uint16_t steps_text_x = 22;
    uint16_t steps_text_y = 50;
    draw_text(spi_dev, gpio_dev, steps_text, steps_text_x, steps_text_y, 0xFFFF, 0x0000, 3); // White text, black background

    const char *distance_text = "Distance";
    uint16_t distance_text_x = 52;
    uint16_t distance_text_y = 170;
    draw_text(spi_dev, gpio_dev, distance_text, distance_text_x, distance_text_y, 0xFFFF, 0x0000, 3); // White text, black background

    const char *miles_text = "mi";
    uint16_t miles_text_x = 160;
    uint16_t miles_text_y = 222;
    draw_text(spi_dev, gpio_dev, miles_text, miles_text_x, miles_text_y, 0xFFFF, 0x0000, 3); // White text, black background




}

void update_total_steps(const struct device *spi_dev, const struct device *gpio_dev, int total_steps) {
    uint16_t x =  120; 
    uint16_t y = 126;   // Below the "Total Steps" label
    draw_number_center(spi_dev, gpio_dev, total_steps, x, y,  0xFFFF, 0x0000, 5);
}

int prev_distance = 0;
void update_distance(const struct device *spi_dev, const struct device *gpio_dev, int distance, int distance_decimal) {
    uint16_t x =  60; 
    uint16_t y = 190;   // Below the "distance" label
    char num_str[6];
    int current_status = -1;
    uint16_t color = 0xFFFF;
    //snprintf(num_str, sizeof(num_str), "%d", blood_oxygen);
   if (distance < 10){
        draw_number_decimal(spi_dev, gpio_dev, distance, 84, 215,  0xFFFF, 0x0000, 4);
   } else{
        draw_number(spi_dev, gpio_dev, distance, 62, 215,  0xFFFF, 0x0000, 4);
   }
    draw_number_decimal(spi_dev, gpio_dev, distance_decimal, 125, 215,  0xFFFF, 0x0000, 4);

    const char *decimal_text = ".";
    uint16_t decimal_text_x = 108;
    uint16_t decimal_text_y = 223;
    draw_text(spi_dev, gpio_dev, decimal_text, decimal_text_x, decimal_text_y, 0xFFFF, 0x0000, 3); // White text, black background


    prev_distance = distance;
}

void setup_weather_screen(const struct device *spi_dev, const struct device *gpio_dev) {
    // Clear the screen by filling it with black color (assuming black is 0x0000)
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, 0x0000);

    uint16_t label_color = 0xFFFF;  // White color for text
    uint16_t bg_color = 0x0000;     // Black background
    uint16_t value_color = 0xFFFF;  // Green color for values

    // Define the positions for each quadrant
    uint16_t air_temp_x = 40, air_temp_y = 50;
    uint16_t air_quality_x = 160, air_quality_y = 50;
    uint16_t humidity_x = 15, humidity_y = 205;
    uint16_t pressure_x = 130, pressure_y = 205;

    uint16_t bodyTempColor = 0xFE60;
    const char *activity_text = "Weather";
    uint16_t activity_text_x = 150;
    uint16_t activity_text_y = 300;
    draw_text(spi_dev, gpio_dev, activity_text, activity_text_x, activity_text_y, bodyTempColor, 0x0000, 2); // White text, black background


    // Labels for Air Temp
    draw_text(spi_dev, gpio_dev, "Air", air_temp_x, air_temp_y, label_color, bg_color, 2);
    draw_text(spi_dev, gpio_dev, "Temp", air_temp_x - 5, air_temp_y + 25, label_color, bg_color, 2);

    // Labels for Air Quality
    draw_text(spi_dev, gpio_dev, "Air", air_quality_x, air_quality_y, label_color, bg_color, 2);
    draw_text(spi_dev, gpio_dev, "Quality", air_quality_x - 20, air_quality_y + 25, label_color, bg_color, 2);

    // Labels for Humidity
    draw_text(spi_dev, gpio_dev, "Humidity", humidity_x, humidity_y + 10, label_color, bg_color, 2);

    // Labels for Pressure
    draw_text(spi_dev, gpio_dev, "Pressure", pressure_x, pressure_y + 10, label_color, bg_color, 2);

    // Units
    draw_text(spi_dev, gpio_dev, "@", air_temp_x + 44, air_temp_y + 58, value_color, bg_color, 2);
    draw_text(spi_dev, gpio_dev, "F", air_temp_x + 57, air_temp_y + 60, value_color, bg_color, 2);
    draw_text(spi_dev, gpio_dev, "AQI", air_quality_x + 70, air_quality_y + 80, value_color, bg_color, 2);
    draw_text(spi_dev, gpio_dev, "%", 76, 250, value_color, bg_color, 3);
    draw_text(spi_dev, gpio_dev, "inHG", pressure_x + 60, pressure_y + 50, value_color, bg_color, 2);
}

void update_weather_data(const struct device *spi_dev, const struct device *gpio_dev, int temp, int humidity, int aqi, int pressure, int pressure_decimal) {
    uint16_t air_temp_x = 30, air_temp_y = 110;
    uint16_t air_quality_x = 155, air_quality_y = 110;
    uint16_t humidity_x = 26, humidity_y = 250;
    uint16_t pressure_x = 120, pressure_y = 250;

    uint16_t value_color = 0xFFFF;  // Green color for values
    uint16_t bg_color = 0x0000;     // Black background

    // Update values
    draw_number(spi_dev, gpio_dev, temp, air_temp_x, air_temp_y, value_color, bg_color, 3);
    draw_number(spi_dev, gpio_dev, aqi, air_quality_x, air_quality_y, value_color, bg_color, 3);
    draw_number(spi_dev, gpio_dev, humidity, humidity_x, humidity_y, value_color, bg_color, 3);

    // Update pressure with decimal
    draw_number(spi_dev, gpio_dev, pressure, pressure_x, pressure_y, value_color, bg_color, 3);
    draw_number_decimal(spi_dev, gpio_dev, pressure_decimal, pressure_x + 50, pressure_y, value_color, bg_color, 3);

    // Draw the decimal point
    draw_text(spi_dev, gpio_dev, ".", pressure_x + 35, pressure_y, value_color, bg_color, 3);

    // Conditions Warning
    uint16_t warning_x = 45;  // Centered x-coordinate
    uint16_t warning_y = 160; // y-coordinate
    uint16_t red_color = 0xF800;
    uint16_t orange_color = 0xFD20;
    uint16_t light_blue_color = 0x841F;

    if (temp > 100) {
        draw_text(spi_dev, gpio_dev, "High Temp", warning_x, warning_y, red_color, bg_color, 3);
    } else if (temp > 90) {
        draw_text(spi_dev, gpio_dev, "High Temp", warning_x, warning_y, orange_color, bg_color, 3);
    } else if (temp < 30) {
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, warning_x + 140, warning_y, 20, 25, 0x0000);
        draw_text(spi_dev, gpio_dev, "Low Temp", warning_x, warning_y, light_blue_color, bg_color, 3);
    } else if (aqi > 350) {
        draw_text(spi_dev, gpio_dev, "High AQI", warning_x, warning_y, red_color, bg_color, 3);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, warning_x + 140, warning_y, 20, 25, 0x0000);
    } else if (aqi > 150) {
        draw_text(spi_dev, gpio_dev, "High AQI", warning_x, warning_y, orange_color, bg_color, 3);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, warning_x + 140, warning_y, 20, 25, 0x0000);
    } else {
        draw_text(spi_dev, gpio_dev, "All Clear", warning_x, warning_y, greenColor, bg_color, 3);
    }

}

void draw_line_rounded(const struct device *spi_dev, const struct device *gpio_dev, int x0, int y0, int x1, int y1, uint16_t color, int thickness) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;

    for (;;) {
        // Draw the pixel with thickness
        for (int tx = -thickness; tx <= thickness; tx++) {
            for (int ty = -thickness; ty <= thickness; ty++) {
                if (tx * tx + ty * ty <= thickness * thickness) {
                    st7789v_fill_rect_optimized(spi_dev, gpio_dev, x0 + tx, y0 + ty, 1, 1, color);
                }
            }
        }

        if (x0 == x1 && y0 == y1) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 < dy) { err += dx; y0 += sy; }
    }
}

void draw_hollow_triangle_rounded(const struct device *spi_dev, const struct device *gpio_dev, int x, int y, int size, uint16_t color, int thickness) {
    int height = (int)(size * sqrt(3) / 2); // Calculate the height of the equilateral triangle

    // Calculate the vertices of the equilateral triangle
    int x1 = x;
    int y1 = y + height;
    int x2 = x + size / 2;
    int y2 = y;
    int x3 = x + size;
    int y3 = y + height;

    // Draw the sides of the triangle
    draw_line_rounded(spi_dev, gpio_dev, x1, y1, x2, y2, color, thickness);
    draw_line_rounded(spi_dev, gpio_dev, x2, y2, x3, y3, color, thickness);
    draw_line_rounded(spi_dev, gpio_dev, x3, y3, x1, y1, color, thickness);
}

void draw_thicker_circle2(const struct device *spi_dev, const struct device *gpio_dev, int16_t cx, int16_t cy, int16_t outer_radius, int16_t thickness, uint16_t color) {
    int16_t inner_radius = outer_radius - thickness + 1;

    // Draw concentric circles from inner_radius to outer_radius
    for (int16_t r = outer_radius; r >= inner_radius; r--) {
        draw_circle_layer2(spi_dev, gpio_dev, cx, cy, r, color);
    }
}

void draw_circle_layer2(const struct device *spi_dev, const struct device *gpio_dev, int16_t cx, int16_t cy, int16_t r, uint16_t color) {
    int x = r, y = 0;
    int err = 0;

    while (x >= y) {
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + x, cy + y, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + y, cy + x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - y, cy + x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - x, cy + y, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - x, cy - y, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx - y, cy - x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + y, cy - x, 1, 1, color);
        st7789v_fill_rect_optimized(spi_dev, gpio_dev, cx + x, cy - y, 1, 1, color);

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }

        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}


void setup_warning_screen(const struct device *spi_dev, const struct device *gpio_dev, const char *alert) {
    uint16_t borderColor = 0xF800;
    uint16_t whiteColor = 0xFFFF;
    uint16_t backgroundColor = 0x0000;

    if (strcmp(alert, "10000_steps") == 0) {borderColor = 0x07E0;}

    // Clear the screen by filling it with the background color
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, backgroundColor);

    // Draw the red border
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, DISPLAY_WIDTH_D, 15, borderColor); // Top border
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, DISPLAY_HEIGHT_D - 15, DISPLAY_WIDTH_D, 40, borderColor); // Bottom border
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, 0, 0, 15, DISPLAY_HEIGHT_D, borderColor); // Left border
    st7789v_fill_rect_optimized(spi_dev, gpio_dev, DISPLAY_WIDTH_D - 15, 0, 15, DISPLAY_HEIGHT_D, borderColor); // Right border

    // Draw the warning triangle with rounded corners
    int triangle_x = (DISPLAY_WIDTH_D / 2) - 50;
    int triangle_y = 50;
    int triangle_size = 100;
    int thickness = 3; // Adjust as needed for thickness
    draw_hollow_triangle_rounded(spi_dev, gpio_dev, triangle_x, triangle_y, triangle_size, whiteColor, thickness);
    // Draw the exclamation mark inside the triangle
     int exclamation_x = triangle_x + (triangle_size / 2);
    int exclamation_y_top = triangle_y + 35;
    int exclamation_y_bottom = triangle_y + 56;
    draw_line_rounded(spi_dev, gpio_dev, exclamation_x, exclamation_y_top, exclamation_x, exclamation_y_bottom, whiteColor, thickness);

    // Draw the dot of the exclamation mark using a small rounded circle
    int dot_x = exclamation_x;
    int dot_y = triangle_y + 70;
    int dot_radius = 5;
    int thickness2 = 5;
    draw_thicker_circle2(spi_dev, gpio_dev, dot_x, dot_y, dot_radius, thickness2, whiteColor);

    // Draw the word "Alert" in large white font
    const char *alert_text = "Alert";
    uint16_t alert_text_x = (DISPLAY_WIDTH_D / 2) - 40; // Adjust to center the text
    uint16_t alert_text_y = 158;
    draw_text(spi_dev, gpio_dev, alert_text, alert_text_x, alert_text_y, whiteColor, backgroundColor, 3); // White text, black background

    if (alert != NULL) {
        if (strcmp(alert, "high_heart_rate") == 0) {
            display_high_heart_rate_alert(spi_dev, gpio_dev);
        } else if (strcmp(alert, "low_heart_rate") == 0) {
            display_low_heart_rate_alert(spi_dev, gpio_dev);
        } else if (strcmp(alert, "low_blood_oxygen") == 0) {
            display_low_blood_oxygen_alert(spi_dev, gpio_dev);
        } else if (strcmp(alert, "10000_steps") == 0) {
            display_steps_alert(spi_dev, gpio_dev);
        } else if (strcmp(alert, "high_body_temp") == 0) {
            display_high_body_temp_alert(spi_dev, gpio_dev);
        } else if (strcmp(alert, "low_body_temp") == 0) {
            display_low_body_temp_alert(spi_dev, gpio_dev);
        } else if (strcmp(alert, "fall_detected") == 0) {
            display_fall_detected_alert(spi_dev, gpio_dev);
        }
    }


}


void display_alert_message(const struct device *spi_dev, const struct device *gpio_dev, const char *message, int line_number) {
    uint16_t whiteColor = 0xFFFF;
    uint16_t backgroundColor = 0x0000;
    int y_offset = 225;
    if (line_number == 12){ y_offset = 208;}
    else if (line_number == 21){ y_offset = 242;}
    else if (line_number == 123){ y_offset = 195;}
    else if (line_number == 231){ y_offset = 225;}
    else if (line_number == 312){ y_offset = 255;}



    draw_text(spi_dev, gpio_dev, message, (DISPLAY_WIDTH_D / 2) - (strlen(message) * 6 * 3 / 2), y_offset, whiteColor, backgroundColor, 3);
}

void display_high_heart_rate_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "High", 12);
    display_alert_message(spi_dev, gpio_dev, "Heart Rate", 21);
}

void display_low_heart_rate_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "Low", 12);
    display_alert_message(spi_dev, gpio_dev, "Heart Rate", 21);
}

void display_low_blood_oxygen_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "Low", 123);
    display_alert_message(spi_dev, gpio_dev, "Blood", 231);
    display_alert_message(spi_dev, gpio_dev, "Oxygen", 312);
}

void display_steps_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "10,000", 12);
    display_alert_message(spi_dev, gpio_dev, "Steps", 21);
}

void display_high_body_temp_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "High", 12);
    display_alert_message(spi_dev, gpio_dev, "Body Temp", 21);
}

void display_low_body_temp_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "Low", 12);
    display_alert_message(spi_dev, gpio_dev, "Body Temp", 21);
}

void display_fall_detected_alert(const struct device *spi_dev, const struct device *gpio_dev) {
    display_alert_message(spi_dev, gpio_dev, "Fall", 12);
    display_alert_message(spi_dev, gpio_dev, "Detected", 21);
}


uint16_t backgroundColor = 0x0000;
uint16_t textColor = 0xFFFF; 
uint16_t heartColor = 0xFE60;
// Define states for the state machine
typedef enum {
    HEART_RATE_SCREEN,
    BLOOD_OXYGEN_SCREEN,
    BODY_TEMP_SCREEN,
    ACTIVITY_SCREEN,
    WEATHER_SCREEN,
    WARNING_SCREEN,
    NUM_SCREENS // This should always be the last element
} DisplayState;

void display_thread_entry(void *p1, void *p2, void *p3) {
    DisplayState current_state = HEART_RATE_SCREEN;
    bool setup_done = false;
    int heart_rate, blood_oxygen, body_temp, body_temp_decimal, total_steps, distance, distance_decimal, temp, humidity, aqi, pressure, pressure_decimal;
    //bool cw_detected = false;
    //bool ccw_detected = false;

    // Seed the random number generator
    srand(time(NULL));

    main_display_init(spi1_dev, gpio0_dev, backgroundColor, heartColor);

    while (1) {
        // Check if setup is needed for the current state
        if (!setup_done) {
            switch (current_state) {
                case HEART_RATE_SCREEN:
                    setup_heartrate_screen(spi1_dev, gpio0_dev);
                    break;
                case BODY_TEMP_SCREEN:
                    setup_bodytemp_screen(spi1_dev, gpio0_dev);
                    break;
                case ACTIVITY_SCREEN:
                    setup_activity_screen(spi1_dev, gpio0_dev);
                    break;
                case WEATHER_SCREEN:
                    setup_weather_screen(spi1_dev, gpio0_dev);
                    break;
                case WARNING_SCREEN:
                    setup_warning_screen(spi1_dev, gpio0_dev, "fall_detected"); // Example alert
                    break;
                default:
                    break;
            }
            setup_done = true;
        }

        // Update screen every second with random values
        switch (current_state) {
            case HEART_RATE_SCREEN:
                heart_rate = rand() % 40 + 60; // Random heart rate between 60 and 100
                update_heart_rate(spi1_dev, gpio0_dev, heart_rate);
                blood_oxygen = rand() % 10 + 90; // Random blood oxygen between 90 and 100
                update_blood_oxygen(spi1_dev, gpio0_dev, blood_oxygen);
                break;


            case BODY_TEMP_SCREEN:
                body_temp = rand() % 4 + 96; // Random body temp between 96 and 100
                body_temp_decimal = rand() % 10; // Random decimal part
                update_body_temp(spi1_dev, gpio0_dev, body_temp, body_temp_decimal);
                break;

            case ACTIVITY_SCREEN:
                total_steps = rand() % 10000; // Random steps between 0 and 9999
                update_total_steps(spi1_dev, gpio0_dev, total_steps);
                distance = rand() % 10; // Random distance between 0 and 9
                distance_decimal = rand() % 10; // Random decimal part
                update_distance(spi1_dev, gpio0_dev, distance, distance_decimal);
                break;

            case WEATHER_SCREEN:
                temp = rand() % 50 + 50; // Random temperature between 50 and 100
                humidity = rand() % 101; // Random humidity between 0 and 100
                aqi = rand() % 500; // Random AQI between 0 and 500
                pressure = rand() % 40 + 950; // Random pressure between 950 and 990
                pressure_decimal = rand() % 10; // Random decimal part
                update_weather_data(spi1_dev, gpio0_dev, temp, humidity, aqi, pressure, pressure_decimal);
                break;

            case WARNING_SCREEN:
                // Example alert, you can add more detailed alert handling here
                setup_warning_screen(spi1_dev, gpio0_dev, "fall_detected");
                break;

            default:
                break;
        }

        k_msleep(1000); // Wait for 1 second

        // // Change state every 10 seconds
        // static int elapsed_time = 0;
        // elapsed_time += 1000;
        // if (elapsed_time >= 10000) {
        //     current_state = (current_state + 1) % NUM_SCREENS;
        //     setup_done = false;
        //     elapsed_time = 0;
        // }

        // Check for rotary encoder input
        if (cw_detected || ccw_detected) {
            if (current_state == WARNING_SCREEN) {
                current_state = HEART_RATE_SCREEN;
            } else {
                if (cw_detected) {
                    current_state = (current_state + 1) % (NUM_SCREENS - 1); // Skip the WARNING_SCREEN
                } else if (ccw_detected) {
                    current_state = (current_state == 0) ? NUM_SCREENS - 2 : current_state - 1; // Skip the WARNING_SCREEN
                }
            }
            setup_done = false;
            cw_detected = false;
            ccw_detected = false;
        }


        
    }
}