#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "drivers/bme68x_port.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3
#define BME68X_I2C_ADDRESS 0x77



int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Initialize the bme68x device
    struct bme68x_dev dev;
    bme68x_pico_init(&dev);
    printf("BME68X device initialized\n");
    printf("%d\n", dev.intf);

    // Define the I2C interface parameters
    bme68x_i2c_data intf_data;
    intf_data.i2c_inst = I2C_PORT;
    intf_data.device_address = BME68X_I2C_ADDRESS;
    dev.intf_ptr = &intf_data;

    // Initialize bme68x device
    int8_t init_res = bme68x_init(&dev);
    printf("BME68X initialized: %d\n", init_res);
    
    // Initialize BSEC library
    bsec_library_return_t besec_init_res = bsec_init();
    printf("BSEC library initialized: %d\n", besec_init_res);

    // Configure the physical sensor
    struct bme68x_conf config;
    config.os_hum = BME68X_OS_1X;
    config.os_pres = BME68X_OS_1X;
    config.os_temp = BME68X_OS_1X;
    config.filter = BME68X_FILTER_OFF;
    int8_t set_conf_res = bme68x_set_conf(&config, &dev);
    printf("BME68X set conf: %d\n", set_conf_res);

    struct bme68x_conf current_config;
    int8_t get_conf_res = bme68x_get_conf(&current_config, &dev);
    printf("BME68X get conf: %d\n", get_conf_res);
    printf("Current config - Humidity OS: %d, Pressure OS: %d, Temperature OS: %d, Filter: %d\n",
           current_config.os_hum, current_config.os_pres, current_config.os_temp, current_config.filter);

    while (true) {
        // Set the device to forced mode to trigger a measurement
        int8_t set_op_res = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
        printf("BME68X set op mode: %d\n", set_op_res);
        struct bme68x_data meas_data;
        uint8_t n_data = 0;

        // Wait for the measurement to complete
        dev.delay_us(bme68x_get_meas_dur(BME68X_FORCED_MODE, &config, &dev), dev.intf_ptr);


        // Read the data
        int8_t get_data_res = bme68x_get_data(BME68X_FORCED_MODE, &meas_data, &n_data, &dev);
        printf("Get data res: %d\n", get_data_res);
        printf("Chip ID: %x\n", dev.chip_id);
        printf("Status: %d\n", meas_data.status);
        printf("Temperature: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%rH, Gas Resistance: %u Ohm\n",
            meas_data.temperature, meas_data.pressure, meas_data.humidity, meas_data.gas_resistance);
        sleep_ms(1000);
    }
}
