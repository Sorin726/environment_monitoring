#ifndef BME68X_H
#define BME68X_H

#include "hardware/i2c.h"
#include "bme68x.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structure that holds the sensor interface details
typedef struct {
    i2c_inst_t *i2c_inst;
    uint8_t device_address;
} bme68x_i2c_data;

void bme68x_pico_init(struct bme68x_dev *dev);

#ifdef __cplusplus
}
#endif

#endif // BME68X