

///------------------------test code & main app code-------------------------------------//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
    #include "freertos/queue.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_tls.h"

#include "firebase.h"
#include "cJSON.h"
#include "ina260.h"

// Logging tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_I2C  = "I2C";
static const char *TAG_UART = "UART";
// static const char *TAG_WIFI = "WIFI";

// State variables
// static bool switch1_state = false;
// static bool switch2_state = false;

// GPIO Configuration
#define LED_1              GPIO_NUM_14
#define POWER_LED_PIN      GPIO_NUM_25
#define LIGHT_SIGNAL_1     GPIO_NUM_26
#define LIGHT_SIGNAL_2     GPIO_NUM_27

// UART Configuration
#define UART_BAUD_RATE     115200
#define BUF_SIZE           1024

#define INTERIOR_UART      UART_NUM_1
#define INTERIOR_TXD_PIN   GPIO_NUM_4
#define INTERIOR_RXD_PIN   GPIO_NUM_5
#define INTERIOR_DE_RE_PIN GPIO_NUM_12

#define EXTERIOR_UART      UART_NUM_2
#define EXTERIOR_TXD_PIN   GPIO_NUM_16
#define EXTERIOR_RXD_PIN   GPIO_NUM_17
#define EXTERIOR_DE_RE_PIN GPIO_NUM_13

// I2C Configuration
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_SDA_IO  GPIO_NUM_21
#define I2C_MASTER_SCL_IO  GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 115200

// Firebase
#define USER_ID "SQSdfpXEHzSEW2WU7pkDBzWIA8i1"

void configure_gpio() {
    gpio_reset_pin(POWER_LED_PIN);
    gpio_set_direction(POWER_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_LED_PIN, 1);

    gpio_reset_pin(LED_1);
    gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_1, 0);
}

void uart_rs485_init() {
    const uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(INTERIOR_UART, &uart_config);
    uart_set_pin(INTERIOR_UART, INTERIOR_TXD_PIN, INTERIOR_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(INTERIOR_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
    gpio_set_direction(INTERIOR_DE_RE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(INTERIOR_DE_RE_PIN, 0);

    uart_param_config(EXTERIOR_UART, &uart_config);
    uart_set_pin(EXTERIOR_UART, EXTERIOR_TXD_PIN, EXTERIOR_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(EXTERIOR_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
    gpio_set_direction(EXTERIOR_DE_RE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EXTERIOR_DE_RE_PIN, 0);
}

// void rs485_interior_task(void *arg) {
//     uint8_t data[BUF_SIZE + 1];
//     while (1) {
//         int len = uart_read_bytes(INTERIOR_UART, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             data[len] = '\0';
//             ESP_LOGI(TAG_UART, "[INTERIOR] RS485 Received: %s", data);

//             // ------------------ TEST CODE (Enable this block to test) ------------------
            
//             // gpio_set_level(LIGHT_SIGNAL_1, 1);
//             // update_switch_state(USER_ID, 1, true);
//             // vTaskDelay(pdMS_TO_TICKS(2000));
//             // gpio_set_level(LIGHT_SIGNAL_1, 0);
//             // update_switch_state(USER_ID, 1, false);
            

//             // ------------------ REAL CODE (Enable this block for deployment) ------------------

//             ESP_LOGI(TAG_UART, "[INTERIOR] Initial trigger detected");

//             gpio_set_level(LIGHT_SIGNAL_1, 1);
//             update_switch_state(USER_ID, 1, true);
            
//             // ⚠️ Flush old data to start fresh
//             uart_flush_input(INTERIOR_UART);
            
//             // Initial active period
//             vTaskDelay(pdMS_TO_TICKS(30000));  // initial 30s light on
            
//             while (1) {
//                 ESP_LOGI(TAG_UART, "[INTERIOR] Waiting for retrigger (30s window)");
            
//                 int retrigger = uart_read_bytes(INTERIOR_UART, data, BUF_SIZE, pdMS_TO_TICKS(30000));
//                 if (retrigger > 0) {
//                     ESP_LOGI(TAG_UART, "[INTERIOR] Retrigger detected, extending light on time");
//                     uart_flush_input(INTERIOR_UART);  // clear buffer for next window
//                     vTaskDelay(pdMS_TO_TICKS(30000)); // extend for another 30s
//                     continue;  // keep waiting for more retriggers
//                 } else {
//                     ESP_LOGI(TAG_UART, "[INTERIOR] No retrigger. Turning off light.");
//                     gpio_set_level(LIGHT_SIGNAL_1, 0);
//                     update_switch_state(USER_ID, 1, false);
//                     break;
//                 }
//             }
            
//         //     ESP_LOGI(TAG_UART, "[INTERIOR] Retrigger detected -  inital trigger");
//         //     gpio_set_level(LIGHT_SIGNAL_1, 1);
//         //     update_switch_state(USER_ID, 1, true);
//         //     // vTaskDelay(pdMS_TO_TICKS(570000));
//         //     vTaskDelay(pdMS_TO_TICKS(30000));

//         //     ESP_LOGI(TAG_UART, "[INTERIOR] Checking for retrigger...");
//         //     int retrigger = uart_read_bytes(INTERIOR_UART, data, BUF_SIZE, 30000 / portTICK_PERIOD_MS);
//         //     if (retrigger > 0) {
//         //         ESP_LOGI(TAG_UART, "[INTERIOR] Retrigger detected -  retrigger");
//         //         continue;
//         //     }
//         //     gpio_set_level(LIGHT_SIGNAL_1, 0);
//         //     update_switch_state(USER_ID, 1, false);
//         // }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

void rs485_interior_task(void *arg) {
    uint8_t data[BUF_SIZE + 1];

    while (1) {
        int len = uart_read_bytes(INTERIOR_UART, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG_UART, "[INTERIOR] RS485 Received: %s", data);

            // ------------------ REAL CODE (Deployment version) ------------------
            ESP_LOGI(TAG_UART, "[INTERIOR] Initial trigger detected");

            gpio_set_level(LIGHT_SIGNAL_1, 1);
            update_switch_state(USER_ID, 1, true);

            // Flush old data to start fresh
            uart_flush(INTERIOR_UART);
            uart_flush_input(INTERIOR_UART);

            // Initial active period (30s light on)
            vTaskDelay(pdMS_TO_TICKS(30000));

            while (1) {
                ESP_LOGI(TAG_UART, "[INTERIOR] Waiting for retrigger (30s window)");

                // Flush again before listening
                uart_flush(INTERIOR_UART);
                uart_flush_input(INTERIOR_UART);

                int retrigger = uart_read_bytes(INTERIOR_UART, data, BUF_SIZE, pdMS_TO_TICKS(30000));
                if (retrigger > 0) {
                    ESP_LOGI(TAG_UART, "[INTERIOR] Retrigger detected, extending light on time");
                    vTaskDelay(pdMS_TO_TICKS(30000)); // extend for another 30s
                    continue;
                } else {
                    ESP_LOGI(TAG_UART, "[INTERIOR] No retrigger. Turning off light.");
                    gpio_set_level(LIGHT_SIGNAL_1, 0);
                    update_switch_state(USER_ID, 1, false);
                    break;
                }
            }
        }

        // Small delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void rs485_exterior_task(void *arg) {
    uint8_t data[BUF_SIZE + 1];
    while (1) {
        int len = uart_read_bytes(EXTERIOR_UART, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG_UART, "[EXTERIOR] RS485 Received: %s", data);

            // ------------------ TEST CODE (Enable this block to test) ------------------
            
            gpio_set_level(LIGHT_SIGNAL_2, 1);
            update_switch_state(USER_ID, 2, true);
            vTaskDelay(pdMS_TO_TICKS(2000));
            gpio_set_level(LIGHT_SIGNAL_2, 0);
            update_switch_state(USER_ID, 2, false);
            

            // ------------------ REAL CODE (Enable this block for deployment) ------------------
            // gpio_set_level(LIGHT_SIGNAL_2, 1);
            // update_switch_state(USER_ID, 2, true);
            // vTaskDelay(pdMS_TO_TICKS(570000));

            // ESP_LOGI(TAG_UART, "[EXTERIOR] Checking for retrigger...");
            // int retrigger = uart_read_bytes(EXTERIOR_UART, data, BUF_SIZE, 30000 / portTICK_PERIOD_MS);
            // if (retrigger > 0) {
            //     ESP_LOGI(TAG_UART, "[EXTERIOR] Retrigger detected");
            //     continue;
            // }
            // gpio_set_level(LIGHT_SIGNAL_2, 0);
            // update_switch_state(USER_ID, 2, false);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode           = I2C_MODE_MASTER,
        .sda_io_num     = I2C_MASTER_SDA_IO,
        .scl_io_num     = I2C_MASTER_SCL_IO,
        .sda_pullup_en  = GPIO_PULLUP_ENABLE,
        .scl_pullup_en  = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void i2c_scan() {
    ESP_LOGI(TAG_I2C, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_I2C, "Found I2C device at 0x%02X", addr);
        }
    }
}




void app_main() {
    ESP_LOGI(TAG_MAIN, "Initializing system...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //wifi main 

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Xcover",
            .password = "wifipass",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Wait for connection
    vTaskDelay(pdMS_TO_TICKS(5000));


    //Realtime Database upload test

    initialize_sntp();  // Ensure accurate time before Firebase uploads
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Small delay for time sync

    configure_gpio();
    i2c_master_init();
    i2c_scan();
    ina260_init();

    uart_rs485_init();
    // wifi_init();

    vTaskDelay(pdMS_TO_TICKS(5000));

    //uart triggers
    xTaskCreate(rs485_interior_task, "rs485_interior_task", 8192, NULL, 10, NULL);
    xTaskCreate(rs485_exterior_task, "rs485_exterior_task", 8192, NULL, 10, NULL);

    //ina260
    xTaskCreate(ina260_task, "ina260_task", 4096, NULL, 10, NULL);
    // xTaskCreate(&upload_battery_task, "upload_battery_task", 4096, NULL, 5, NULL);

     // Start Firestore sync tasks
     xTaskCreate(&fetch_firestore_data_task, "fetch_firestore_task", 8192, NULL, 5, NULL);
     //xTaskCreate(&toggle_switch_task, "toggle_switch_task", 4096, NULL, 5, NULL);

     
    ESP_LOGI(TAG_MAIN, "Setup complete.");
}
