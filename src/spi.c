#include "../inc/spi.h"

// Define the SPI configuration structure
struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
    .frequency = 4000000, // 4 MHz, but adjust based on your board capabilities
    .slave = 0, // Typically, your SPI slave's chip select pin
    .cs = NULL, // Pointer to GPIO descriptor if used for CS, NULL if hardware CS is used
};

// Function to perform an SPI write operation
int spi_write1(const struct device *spi_dev, uint8_t *data, size_t data_len, struct spi_config sg) {
    struct spi_buf buf = {
        .buf = data,
        .len = data_len,
    };

    struct spi_buf_set buf_set = {
        .buffers = &buf,
        .count = 1,
    };

    return spi_transceive(spi_dev, &sg, &buf_set, NULL);
}
