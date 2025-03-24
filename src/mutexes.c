#include "../inc/mutexes.h"

struct k_mutex steps_lock; //for steps
struct k_mutex i2c_lock; //for sensor_data struct in main
struct k_mutex imu_lock; //for IMU_readings[]

void init_mutexes() {
    k_mutex_init(&steps_lock);
    k_mutex_init(&i2c_lock);
    k_mutex_init(&imu_lock); 
}