#pragma once

#include <zephyr/kernel.h>

extern void init_mutexes();

extern struct k_mutex steps_lock; //for steps
extern struct k_mutex i2c_lock; //for sensor_data struct in main
extern struct k_mutex imu_lock; //for IMU_readings[]