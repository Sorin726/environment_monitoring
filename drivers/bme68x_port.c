// This file represent the glue code necessary bme68x I2C usage

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bme68x.h"
#include "bme68x_port.h"


// I2C write function implementation
BME68X_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    // Unpack the struct members to obtain the i2c instance and device address
    bme68x_i2c_data *intf_data = (bme68x_i2c_data *)intf_ptr;
    i2c_inst_t *i2c_inst = intf_data->i2c_inst;
    uint8_t device_address = intf_data->device_address;
    // Use the raspberry pi pico write blocking function for now
    uint8_t buffer[length + 1];
    buffer[0] = reg_addr;
    for(uint32_t i = 0; i < length; i++) {
        buffer[i + 1] = reg_data[i];
    }
    int res = i2c_write_blocking(i2c_inst, device_address, buffer, length + 1, false);

    // Check if transaction succeed, return generic -1 for error
    if(res != (int)(length + 1)) {
        return -1;
    }
    else return 0;
}

// I2C read function implementation
BME68X_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    bme68x_i2c_data *intf_data = (bme68x_i2c_data *)intf_ptr;
    i2c_inst_t *i2c_inst = intf_data->i2c_inst;
    uint8_t device_address = intf_data->device_address;
    // Write the address of the register from which we want to read
    int res = i2c_write_blocking(i2c_inst, device_address, &reg_addr, 1, true);
    if(res != 1) {
        return -1;
    }

    // Read the content of the register
    res = i2c_read_blocking(i2c_inst, device_address, reg_data, length, false);
    if(res != (int)length) {
        return -1;
    }
    else return 0;
}

// Function to define the delay
void delay_us(uint32_t period, void *intf_ptr) {
    sleep_us((uint64_t)period);
}

// Function to populate bme68x device structure with the function pointers
void bme68x_pico_init(struct bme68x_dev *dev) {
    dev->intf = BME68X_I2C_INTF;
    dev->read = i2c_read;
    dev->write = i2c_write;
    dev->delay_us = delay_us;
}