#ifndef SPI_H
#define SPI_H

#include <zephyr/types.h>
#include <zephyr/drivers/spi.h>

extern struct spi_config spi_cfg; // Declare the external SPI configuration to be used by other files.

int spi_write1(const struct device *spi_dev, uint8_t *data, size_t data_len, struct spi_config sg);

#endif // SPI_H
