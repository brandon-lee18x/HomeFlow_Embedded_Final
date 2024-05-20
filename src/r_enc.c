#include "../inc/r_enc.h"
#define BUTTON_DEBOUNCE_MS 50

LOG_MODULE_REGISTER(r_enc);

// Global state for the decoder
volatile uint8_t decoder_state = 1;

volatile int button_pressed = 0;

struct gpio_callback gpio_cb, encoder_cb;

void init_enc() {
    if (!gpio0_dev) {
        printk("Cannot find!\n");
        return;
    }
    //configure pins A, B, and BTN to input. Have A & B pulled up, and btn pulled down
	int ret = gpio_pin_configure(gpio0_dev, P0_03, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) {
        printk("Error %d: Could not configure GPIO pin\n", ret);
        return;
	}
    ret = gpio_pin_configure(gpio0_dev, P0_04, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) {
        printk("Error %d: Could not configure GPIO pin\n", ret);
        return;
	}
    //NOTE: P0_05 didn't work for some reason (the intended pin), might be due to HW issues on DK board
    ret = gpio_pin_configure(gpio1_dev, P1_08, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret != 0) {
        printk("Error %d: Could not configure GPIO pin\n", ret);
        return;
	}

    //init code for encoder interrupt
    gpio_pin_interrupt_configure(gpio0_dev, P0_03, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure(gpio0_dev, P0_04, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&encoder_cb, encoder_isr, BIT(P0_03) | BIT(P0_04));
    gpio_add_callback(gpio0_dev, &encoder_cb);

    //init code for button interrupt
    gpio_init_callback(&gpio_cb, click_isr, BIT(P1_08));
	gpio_add_callback(gpio1_dev, &gpio_cb);
	ret = gpio_pin_interrupt_configure(gpio1_dev, P1_08, GPIO_INT_EDGE_RISING | GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error %d: Could not configure interrupt\n", ret);
        return;
    }

    // Decoder output pins
    gpio_pin_configure(gpio0_dev, PIN_A, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpio0_dev, PIN_B, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpio0_dev, PIN_C, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);

    set_decoder_state(gpio0_dev, decoder_state);

    //LED init (P013). LED is active low
    gpio_pin_configure(gpio0_dev, PIN_LED, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);

    printk("rotary encoder initialized\n");
}

// Function to set the decoder state
void set_decoder_state(const struct device *dev, uint8_t state) {
    gpio_pin_set(dev, PIN_A, (state & 0b100) >> 2);
    gpio_pin_set(dev, PIN_B, (state & 0b010) >> 1);
    gpio_pin_set(dev, PIN_C, state & 0b001);
    LOG_INF("Decoder state: %d\n", state);
}

// Encoder GPIO ISR
void encoder_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint8_t pin_a_val = gpio_pin_get(dev, P0_03);
    uint8_t pin_b_val = gpio_pin_get(dev, P0_04);
    // process_encoder_input(dev, pin_a_val, pin_b_val);

    uint8_t encoder_val = (gpio_pin_get(dev, P0_03) << 1) | gpio_pin_get(dev, P0_04); 
    debounce_twist(dev, encoder_val);
}

volatile short state = 0;

void debounce_twist(const struct device *dev, uint8_t enc_val) {
    switch (state) {
        case 0: //enc = 11
            switch (enc_val) {
                case 0b01:
                    state = 1;
                    break;
                case 0b10:
                    state = 4;
                    break;
            }
            break;
        case 1: //enc = 01, cw
            switch (enc_val) {
                case 0b11:
                    state = 0;
                    break;
                case 0b00:
                    state = 2;
                    break;
            }
            break;
        case 2: //enc = 00, cw
            switch (enc_val) {
                case 0b01:
                    state = 1;
                    break;
                case 0b10:
                    state = 3;
                    break;
            }
            break;
        case 3: //enc = 10, cw
            switch (enc_val) {
                case 0b00:
                    state = 2;
                    break;
                case 0b11:
                    //do decoder logic to indicate clockwise turn
                    decoder_state = (decoder_state % 5) + 1;
                    set_decoder_state(dev, decoder_state);
                    state = 0;
                    break;
            }
            break;
        case 4: //enc = 10, ccw
            switch (enc_val) {
                case 0b11:
                    state = 0;
                    break;
                case 0b00:
                    state = 5;
                    break;
            }
            break;
        case 5: //enc = 00, ccw
            switch (enc_val) {
                case 0b10:
                    state = 4;
                    break;
                case 0b01:
                    state = 6;
                    break;
            }
            break;
        case 6: //enc = 01, ccw
            switch (enc_val) {
                case 0b00:
                    state = 5;
                    break;
                case 0b11:
                    //do decoder logic to indicate ccw turn
                    decoder_state = (decoder_state == 1) ? 5 : decoder_state - 1;
                    set_decoder_state(dev, decoder_state);
                    state = 0;
                    break;
            }
            break;
    }
}

//interrupt service routine for button click 
void click_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    button_pressed = 1;
}

void debounce_button(const struct device *dev) {
    k_msleep(BUTTON_DEBOUNCE_MS);
    if (gpio_pin_get(dev, P1_08) == 1) {
        gpio_pin_toggle(dev, PIN_LED);
    }
}

void check_enc_btn() {
    if (button_pressed) {
        debounce_button(gpio0_dev);
        button_pressed = 0;
    }
}