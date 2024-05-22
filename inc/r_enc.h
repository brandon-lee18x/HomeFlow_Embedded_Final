#ifndef R_ENC_H
#define R_ENC_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>

// Assign pins A, B, and BTN to P104, P105, & P106 respectively
#define P0_03 3
#define P0_04 4
#define P1_08 8 // NOTE: P0_05 didn't work for some reason (the intended pin), might be due to HW issues on DK board
#define PIN_A 24
#define PIN_B 25
#define PIN_C 0
#define PIN_LED 13

extern const struct device *gpio0_dev;
extern const struct device *gpio1_dev;
extern volatile bool cw_detected;
extern volatile bool ccw_detected;

//initialize gpios for encoder
void init_enc();
void set_decoder_state(const struct device *dev, uint8_t state);
void process_encoder_input(const struct device *dev, uint8_t pin_a_val, uint8_t pin_b_val);
void encoder_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void click_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void debounce_twist(const struct device *dev, uint8_t enc_val);
void debounce_button(const struct device *dev);
void check_enc_btn();

#endif