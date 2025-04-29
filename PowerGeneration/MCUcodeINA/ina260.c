#include "ina260.h"
#include "driver/i2c.h"
#include "firebase.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define INA260_ADDR 0x40
#define INA260_TIMEOUT_MS 1000
#define USER_ID "SQSdfpXEHzSEW2WU7pkDBzWIA8i1"

static const char *TAG = "INA260";

esp_err_t ina260_read_register(uint8_t reg, uint16_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA260_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA260_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t data[2];
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(INA260_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return ret;
}

static float raw_to_current(uint16_t raw) { return (int16_t)raw * 1.25f / 1000.0f; }
static float raw_to_voltage(uint16_t raw) { return raw * 1.25f / 1000.0f; }
static float raw_to_power(uint16_t raw)   { return raw * 10.0f  / 1000.0f; }

void ina260_init() {
    ESP_LOGI(TAG, "INA260 initialized");
}

void ina260_task(void *arg) {
    while (1) {
        uint16_t raw_current, raw_voltage, raw_power;

        if (ina260_read_register(0x01, &raw_current) == ESP_OK &&
            ina260_read_register(0x02, &raw_voltage) == ESP_OK &&
            ina260_read_register(0x03, &raw_power) == ESP_OK) {

            float current = raw_to_current(raw_current);
            float voltage = raw_to_voltage(raw_voltage);
            float power = raw_to_power(raw_power);

            ESP_LOGI(TAG, "Current: %.3f A | Voltage: %.3f V | Power: %.3f W", current, voltage, power);

            log_ina260_reading(USER_ID, current, voltage, power);
        } else {
            ESP_LOGE(TAG, "Failed to read INA260 registers");
        }

        vTaskDelay(pdMS_TO_TICKS(10000));  // every 10s
    }
}
