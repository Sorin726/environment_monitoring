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

uint64_t get_time_stamp_ns() {
    // Return current time in nanoseconds
    return time_us_64()*1000;
}

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

    // Create a BSEC subscription for the required outputs
    bsec_sensor_configuration_t sensor_config[9];
    uint8_t n_sensors = 9;

    sensor_config[0].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    sensor_config[0].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[1].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    sensor_config[1].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    sensor_config[2].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    sensor_config[3].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[4].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    sensor_config[4].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[5].sensor_id = BSEC_OUTPUT_IAQ;
    sensor_config[5].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[6].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    sensor_config[6].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[7].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    sensor_config[7].sample_rate = BSEC_SAMPLE_RATE_LP;
    sensor_config[8].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    sensor_config[8].sample_rate = BSEC_SAMPLE_RATE_LP;

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    bsec_update_subscription(sensor_config, n_sensors, required_sensor_settings, &n_required_sensor_settings);

    // Set the device to forced mode to trigger a measurement
    int8_t set_op_res = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
    printf("BME68X set op mode: %d\n", set_op_res);

    // Initialize the timestamp outside the loop
    uint64_t time_stamp = get_time_stamp_ns();
    bsec_bme_settings_t sensor_settings;
    bsec_sensor_control(time_stamp, &sensor_settings);
    
    while (true) {
        struct bme68x_data meas_data;
        uint8_t n_data = 0;

        bsec_input_t inputs[4];
        uint8_t n_inputs = 4;
        bsec_output_t outputs[10];
        uint8_t n_outputs = 10;

        inputs[0].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[0].signal = meas_data.temperature;
        inputs[0].time_stamp = get_time_stamp_ns();
        inputs[1].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[1].signal = meas_data.humidity;
        inputs[1].time_stamp = get_time_stamp_ns();
        inputs[2].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[2].signal = meas_data.pressure;
        inputs[2].time_stamp = get_time_stamp_ns();
        inputs[3].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[3].signal = (float)meas_data.gas_resistance;
        inputs[3].time_stamp = get_time_stamp_ns();

        // Implement the main processing loop from the documentation
        time_stamp = get_time_stamp_ns();
        if (time_stamp >= sensor_settings.next_call) {
            bsec_sensor_control(time_stamp, &sensor_settings);
            if (sensor_settings.trigger_measurement == 1) {
                // Wait for the measurement to complete
                dev.delay_us(bme68x_get_meas_dur(BME68X_FORCED_MODE, &config, &dev), dev.intf_ptr);

                // Read the data
                int8_t get_data_res = bme68x_get_data(BME68X_FORCED_MODE, &meas_data, &n_data, &dev);

                if (sensor_settings.process_data |= 0) {
                    bsec_do_steps(inputs, n_inputs, outputs, &n_outputs);
                    for (uint8_t i = 0; i < n_outputs; i++) {
                        switch (outputs[i].sensor_id) {
                            case BSEC_OUTPUT_RAW_TEMPERATURE:
                                printf("Raw Temperature: %.2f C, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_RAW_HUMIDITY:
                                printf("Raw Humidity: %.2f %%rH, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_RAW_PRESSURE:
                                printf("Raw Pressure: %.2f Pa, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                                printf("Heat Compensated Temperature: %.2f C, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                                printf("Heat Compensated Humidity: %.2f %%rH, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_IAQ:
                                printf("IAQ: %.2f, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_STATIC_IAQ:
                                printf("Static IAQ: %.2f, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_CO2_EQUIVALENT:
                                printf("CO2 Equivalent: %.2f ppm, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                                printf("Breath VOC Equivalent: %.2f ppm, Accuracy: %d\n", outputs[i].signal, outputs[i].accuracy);
                                break;
                        }
                
                    }
                }
            }
        }
    // sleep_ms(1000);
    }   
}