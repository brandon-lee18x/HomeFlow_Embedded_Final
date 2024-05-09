#include "../inc/display.h"
#include "../inc/spi.h"


#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define SLEEP_TIME_MS_D   100

// Update these definitions according to your pin connections
#define CS_GPIO_PORT_D    DT_LABEL(DT_NODELABEL(gpio0))
#define CS_GPIO_PIN_D     16

#define DC_GPIO_PORT_D    DT_LABEL(DT_NODELABEL(gpio0))
#define DC_GPIO_PIN_D     14

#define RESET_GPIO_PORT_D DT_LABEL(DT_NODELABEL(gpio0))
#define RESET_GPIO_PIN_D  24

#define BL_GPIO_PORT_D    DT_LABEL(DT_NODELABEL(gpio0))
#define BL_GPIO_PIN_D     22

#define ST7789V_CMD_SWRESET_D            0x01
#define ST7789V_CMD_SLPOUT_D             0x11
#define ST7789V_CMD_COLMOD_D             0x3A
#define ST7789V_CMD_DISPON_D             0x29

#define ST7789V_CMD_CASET_D 0x2A // Column address set
#define ST7789V_CMD_RASET_D 0x2B // Row address set
#define ST7789V_CMD_RAMWR_D 0x2C // Memory write

#define FONT_WIDTH_D 6
#define FONT_HEIGHT_D 10
#define SCALE_D 5

#define DISPLAY_WIDTH_D 240
#define DISPLAY_HEIGHT_D 320

// const uint8_t font[10][FONT_WIDTH_D] = {
//     {0x7C, 0xA2, 0x92, 0x8A, 0x7C}, // 0
//     {0x00, 0x84, 0xFE, 0x80, 0x00}, // 1
//     {0x84, 0xC2, 0xA2, 0x92, 0x8C}, // 2
//     {0x44, 0x82, 0x92, 0x92, 0x6C}, // 3
//     {0x30, 0x28, 0x24, 0xFE, 0x20}, // 4
//     {0x4E, 0x8A, 0x8A, 0x8A, 0x72}, // 5
//     {0x78, 0x94, 0x92, 0x92, 0x60}, // 6
//     {0x06, 0x02, 0xE2, 0x12, 0x0E}, // 7
//     {0x6C, 0x92, 0x92, 0x92, 0x6C}, // 8
//     {0x0C, 0x92, 0x92, 0x52, 0x3C}, // 9
// };


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

// Define the function to write to the display's GRAM
void st7789v_write_pixel(const struct device *spi_dev, const struct device *gpio_dev, uint16_t color) {
    uint8_t command = ST7789V_CMD_RAMWR_D;
    uint8_t data[] = {
        color >> 8, color & 0xFF
    };

    st7789v_send_command(spi_dev, gpio_dev, command);
    st7789v_send_data(spi_dev, gpio_dev, data, sizeof(data));
}

// Define the function to draw a pixel on the display
void st7789v_draw_pixel(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t color) {
    // Set the column address to the x location
    st7789v_set_column_address(spi_dev, gpio_dev, x, x);
    // Set the row address to the y location
    st7789v_set_row_address(spi_dev, gpio_dev, y, y);
    // Write the color to the GRAM
    st7789v_write_pixel(spi_dev, gpio_dev, color);
}

// void st7789v_draw_number(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint8_t num, uint16_t color) {
//     if (num > 9) return; // We only handle single digits here

//     // Loop through the font bitmap
//     for (int row = 0; row < FONT_HEIGHT_D; ++row) {
//         for (int col = 0; col < FONT_WIDTH_D; ++col) {
//             if (font[num][col] & (1 << row)) {
//                 // Draw each pixel of the font character, scaled up
//                 for (int scaleX = 0; scaleX < SCALE_D; ++scaleX) {
//                     for (int scaleY = 0; scaleY < SCALE_D; ++scaleY) {
//                         st7789v_draw_pixel(spi_dev, gpio_dev, x + (col * SCALE_D) + scaleX, y + (row * SCALE_D) + scaleY, color);
//                     }
//                 }
//             }
//         }
//     }
// }

void st7789v_fill_rect(const struct device *spi_dev, const struct device *gpio_dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    uint8_t data[4];

    // Check if the rectangle is within the bounds of the display
    // if ((x >= 240) || (y >= 320) || (x + width > 240) || (y + height > 320)) {
    //     // Error handling: Rectangle is out of bounds
    //     return;
    // }

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

void main_display_init(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t color){

    // Configure pins for use with the LCD display
    gpio_pin_configure(gpio0_dev, CS_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);

    gpio_pin_configure(gpio0_dev, DC_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
	
    gpio_pin_configure(gpio0_dev, RESET_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
	
    gpio_pin_configure(gpio0_dev, BL_GPIO_PIN_D, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);

    // Initialize the display
    st7789v_init_display(spi1_dev, gpio0_dev);
	
	st7789v_fill_rect(spi1_dev, gpio0_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, 0xFFFF); // Fills entire display with blue color
}

// Function to update the display (could be used for periodic updates or refreshes)
uint16_t main_display_update_test(const struct device *spi1_dev, const struct device *gpio0_dev, uint16_t color){
    st7789v_fill_rect_optimized(spi1_dev, gpio0_dev, 0, 0, DISPLAY_WIDTH_D, DISPLAY_HEIGHT_D, color); // Fills entire display with blue color
    return color += 0x1111;
}