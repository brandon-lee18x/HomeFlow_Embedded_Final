#include "../inc/bme.h"

void BME688_Init() {
    uint8_t SlaveAddress = 0x76;// BME688 Address
    int ret; // Variable for storing return value of i2c operations
    uint8_t recData[3]; // Buffer for received data

    // Get and set osrs_H
    uint8_t get_osrs_h[1] = {0x72};
    ret = i2c_write(i2c0_dev, get_osrs_h, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor

    uint8_t set_osrs_h[2] = {0x72, (0x01 | (recData[0] & 0xF8))};
    ret = i2c_write(i2c0_dev, set_osrs_h, 2, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    // Get and set osrs_t and osrs_p
    uint8_t get_osrs_t_p[1] = {0x74};
    ret = i2c_write(i2c0_dev, get_osrs_t_p, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor

    uint8_t set_osrs_t[2] = {0x74, (0x54 | (recData[0] & 0x03))};
    ret = i2c_write(i2c0_dev, set_osrs_t, 2, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    // Set gas_wait_0
    uint8_t set_gas_wait_0[2] = {0x64, 0x59};
    ret = i2c_write(i2c0_dev, set_gas_wait_0, 2, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    // Calculating res_heat_x for 300C
    uint8_t get_par_g1[1] = {0xED};
    ret = i2c_write(i2c0_dev, get_par_g1, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_g1 = recData[0];

    uint8_t get_par_g2[1] = {0xEB};
    ret = i2c_write(i2c0_dev, get_par_g2, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_g2 = recData[0] + (recData[1] << 8);

    uint8_t get_par_g3[1] = {0xEE};
    ret = i2c_write(i2c0_dev, get_par_g3, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_g3 = recData[0];

    uint8_t get_res_heat_range[1] = {0x02};
    ret = i2c_write(i2c0_dev, get_res_heat_range, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double res_heat_range = ((recData[0] & 0x30) >> 4);

    uint8_t get_res_heat_val[1] = {0x00};
    ret = i2c_write(i2c0_dev, get_res_heat_val, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double res_heat_val = recData[0];

    // Calculate res_heat_x
    double var1 = (par_g1 / 16.0) + 49.0;
    double var2 = ((par_g2 / 32768.0) * 0.0005) + 0.00235;
    double var3 = par_g3 / 1024.0;
    double var4 = var1 * (1.0 + (var2 * 300.0));
    double var5 = var4 + (var3 * 68.0);
    uint8_t res_heat_x = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + res_heat_range)) * (1.0 / (1.0 + (res_heat_val * 0.002)))) - 25.0));

    uint8_t set_res_heat_0[2] = {0x5A, res_heat_x};
    ret = i2c_write(i2c0_dev, set_res_heat_0, 2, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    // Set nb_conv
    uint8_t read_nb_conv[1] = {0x71};
    ret = i2c_write(i2c0_dev, read_nb_conv, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor

    uint8_t nb_conv_write[2] = {0x71, ((recData[0] & 0xF0) | 0x20)};
    ret = i2c_write(i2c0_dev, nb_conv_write, 2, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor
}


void BME688_Take_Measurement() {
    uint8_t SlaveAddress = 0x76; // BME688 Address
    uint8_t recData[3];
    int ret;

    // Read mode
    uint8_t read_mode[1] = {0x74};
    ret = i2c_write(i2c0_dev, read_mode, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor

    uint8_t sending = recData[0] & ~0x03;
    sending |= 0x01;
    uint8_t take_measurement[2] = {0x74, sending};
    ret = i2c_write(i2c0_dev, take_measurement, 2, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor
}

double BME688_Get_Temp_C() {
    uint8_t SlaveAddress = 0x76; // BME688 Address
    uint8_t recData[3];
    int ret;

    // Get temperature
    uint8_t getTemp[1] = {0x22};
    ret = i2c_write(i2c0_dev, getTemp, 1, SlaveAddress);
    if (ret < 0) return -1.0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 3, SlaveAddress);
    if (ret < 0) return -1.0; // Error reading from sensor

    double t_adc = (recData[1] << 4) + (recData[0] << 12) + (recData[2] >> 4);

    // Get par_t1
    uint8_t get_par_t1[1] = {0xE9};
    ret = i2c_write(i2c0_dev, get_par_t1, 1, SlaveAddress);
    if (ret < 0) return -1.0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return -1.0; // Error reading from sensor
    double par_t1 = recData[0] + (recData[1] << 8);

    // Get par_t2
    uint8_t get_par_t2[1] = {0x8A};
    ret = i2c_write(i2c0_dev, get_par_t2, 1, SlaveAddress);
    if (ret < 0) return -1.0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return -1.0; // Error reading from sensor
    double par_t2 = recData[0] + (recData[1] << 8);

    // Get par_t3
    uint8_t get_par_t3[1] = {0x8C};
    ret = i2c_write(i2c0_dev, get_par_t3, 1, SlaveAddress);
    if (ret < 0) return -1.0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return -1.0; // Error reading from sensor
    double par_t3 = recData[0];

    // Calculate temperature
    double var1f = ((t_adc / 16384.0) - (par_t1 / 1024.0)) * par_t2;
    double var2f = (((t_adc / 131072.0) - (par_t1 / 8192.0)) * ((t_adc / 131072.0) - (par_t1 / 8192.0)) * (par_t3 * 16.0));
    double t_finef = var1f + var2f;
    double final_temp = t_finef / 5120.0;

    return final_temp;
}

double BME688_Get_Humidity() {
    double temp_comp = BME688_Get_Temp_C();

    uint8_t SlaveAddress = 0x76; // BME688 Address in the new format
    uint8_t recData[3];
    int ret; // Variable for storing return value of i2c operations

    // Request humidity data
    uint8_t getHum[1] = {0x25};
    ret = i2c_write(i2c0_dev, getHum, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double h_adc = (recData[0] << 8) + recData[1];

    // Retrieve calibration parameters
    // par_h1
    uint8_t get_par_h1[1] = {0xE2};
    ret = i2c_write(i2c0_dev, get_par_h1, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h1 = (recData[0] & 0x0F) + (recData[1] << 4);

    // par_h2
    uint8_t get_par_h2[1] = {0xE1};
    ret = i2c_write(i2c0_dev, get_par_h2, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h2 = (recData[0] << 4) + ((recData[1] & 0xF0) >> 4);

    // par_h3
    uint8_t get_par_h3[1] = {0xE4};
    ret = i2c_write(i2c0_dev, get_par_h3, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h3 = recData[0];

    // par_h4
    uint8_t get_par_h4[1] = {0xE5};
    ret = i2c_write(i2c0_dev, get_par_h4, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h4 = recData[0];

    // par_h5
    uint8_t get_par_h5[1] = {0xE6};
    ret = i2c_write(i2c0_dev, get_par_h5, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h5 = recData[0];

    // par_h6
    uint8_t get_par_h6[1] = {0xE7};
    ret = i2c_write(i2c0_dev, get_par_h6, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h6 = recData[0];

    // par_h7
    uint8_t get_par_h7[1] = {0xE8};
    ret = i2c_write(i2c0_dev, get_par_h7, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_h7 = recData[0];

    // Calculate compensated humidity
    double var1 = h_adc - (par_h1 * 16.0) + ((par_h3 / 2.0) * temp_comp);
    double var2 = var1 * ((par_h2 / 262144.0) * (1.0 + ((par_h4 / 16384.0) * temp_comp) + ((par_h5 / 1048576.0) * temp_comp * temp_comp)));
    double var3 = par_h6 / 16384.0;
    double var4 = par_h7 / 2097152.0;
    double hum_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    return hum_comp;
}


double BME688_Get_Pressure() {
    double temp_comp = BME688_Get_Temp_C();
    double t_fine = temp_comp * 6500.0;

    uint8_t SlaveAddress = 0x76; // BME688 Address in the new format
    uint8_t recData[3];
    int ret; // Variable for storing return value of i2c operations

    // Request pressure data
    uint8_t getPr[1] = {0x1F};
    ret = i2c_write(i2c0_dev, getPr, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 3, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double p_adc = (recData[0] << 12) + (recData[1] << 4) + (recData[2] >> 4);

    // Retrieve calibration parameters
    // par_p1
    uint8_t get_par_p1[1] = {0x8E};
    ret = i2c_write(i2c0_dev, get_par_p1, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p1 = (recData[1] << 8) + recData[0];

    // par_p2
    uint8_t get_par_p2[1] = {0x90};
    ret = i2c_write(i2c0_dev, get_par_p2, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p2 = (recData[1] << 8) + recData[0];

    // par_p3
    uint8_t get_par_p3[1] = {0x92};
    ret = i2c_write(i2c0_dev, get_par_p3, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p3 = recData[0];

    // par_p4
    uint8_t get_par_p4[1] = {0x94};
    ret = i2c_write(i2c0_dev, get_par_p4, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p4 = (recData[1] << 8) + recData[0];

    // par_p5
    uint8_t get_par_p5[1] = {0x96};
    ret = i2c_write(i2c0_dev, get_par_p5, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p5 = (recData[1] << 8) + recData[0];

    // par_p6
    uint8_t get_par_p6[1] = {0x99};
    ret = i2c_write(i2c0_dev, get_par_p6, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p6 = recData[0];

    // par_p7
    uint8_t get_par_p7[1] = {0x98};
    ret = i2c_write(i2c0_dev, get_par_p7, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p7 = recData[0];

    // par_p8
    uint8_t get_par_p8[1] = {0x9C};
    ret = i2c_write(i2c0_dev, get_par_p8, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p8 = (recData[1] << 8) + recData[0];

    // par_p9
    uint8_t get_par_p9[1] = {0x9E};
    ret = i2c_write(i2c0_dev, get_par_p9, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p9 = (recData[1] << 8) + recData[0];

    // par_p10
    uint8_t get_par_p10[1] = {0xA0};
    ret = i2c_write(i2c0_dev, get_par_p10, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double par_p10 = recData[0];

    // Pressure compensation calculation
    double var1 = (t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * (par_p6 / 131072.0);
    var2 = var2 + (var1 * par_p5 * 2.0);
    var2 = (var2 / 4.0) + (par_p4 * 65536.0);

    var1 = (((par_p3 * var1 * var1) / 16384.0) + (par_p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * par_p1;
    double press_comp = 1048576.0 - p_adc;
    press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;

    var1 = (par_p9 * press_comp * press_comp) / 2147483648.0;
    var2 = press_comp * (par_p8 / 32768.0);
    double var3 = (press_comp / 256.0) * (press_comp / 256.0) * (press_comp / 256.0) * (par_p10 / 131072.0);
    press_comp = press_comp + (var1 + var2 + var3 + (par_p7 * 128.0)) / 16.0;

    return press_comp;
}

double BME688_Get_Gas() {
    uint8_t SlaveAddress = 0x76; // BME688 Address in the new format
    uint8_t recData[2];
    int ret; // Variable for storing return value of i2c operations

    // Check gas status
    uint8_t check[1] = {0x2D};
    ret = i2c_write(i2c0_dev, check, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    // printf("%u\n", (recData[0]&0x30)); // Gas status

    // Get gas range
    uint8_t get_gas_range[1] = {0x2D};
    ret = i2c_write(i2c0_dev, get_gas_range, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 1, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double gas_range = (recData[0] & 0x0F);

    // Get gas ADC
    uint8_t get_gas_adc[1] = {0x2C};
    ret = i2c_write(i2c0_dev, get_gas_adc, 1, SlaveAddress);
    if (ret < 0) return 0; // Error writing to sensor

    ret = i2c_read(i2c0_dev, recData, 2, SlaveAddress);
    if (ret < 0) return 0; // Error reading from sensor
    double gas_adc = (recData[0] << 2) + ((recData[1] & 0xC0) >> 6);

    // Gas resistance calculation
    uint32_t var1 = UINT32_C(262144) >> (int)gas_range;
    int32_t var2 = (int32_t)gas_adc - INT32_C(512);
    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;
    double gas_res = 1000000.0f * (float)var1 / (float)var2;

    return gas_res;
}