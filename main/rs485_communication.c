// #include <stdio.h>
// #include "driver/gpio.h"
// #include "driver/uart.h"
// // #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <string.h>
// #include <stdlib.h>
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "freertos/queue.h"
// #include "sdkconfig.h"

// #define TAG "RS485_ECHO_APP"

// // UART and GPIO Pin Definitions
// #define EXTERIOR_TXD1_PIN (GPIO_NUM_16)   // UART TX1 for exterior
// #define EXTERIOR_RXD1_PIN (GPIO_NUM_14)   // UART RX1 for exterior

// #define INTERIOR_TXD2_PIN (GPIO_NUM_30)   // UART TX2 for interior
// #define INTERIOR_RXD2_PIN (GPIO_NUM_28)   // UART RX2 for interior

// #define EXTERIOR_ENABLE_PIN (GPIO_NUM_26)  // IO4 for THVD1 (exterior)
// #define INTERIOR_ENABLE_PIN (GPIO_NUM_29)  // IO5 for THVD2 (interior)

// #define LIGHT_SIGNAL_1 (GPIO_NUM_33)       // Exterior light signal pin
// #define LIGHT_SIGNAL_2 (GPIO_NUM_36)       // Interior light signal pin

// #define UART_PORT_NUM_0 UART_NUM_0         // Use UART0 for default UART communication
// #define UART_PORT_NUM_1 UART_NUM_1         // Use UART1 for RS485 communication

// #define UART_BAUD_RATE 9600
// #define BUF_SIZE (1024)

// // Function to configure GPIO pins
// void configure_gpio() {
//     gpio_reset_pin(EXTERIOR_ENABLE_PIN);
//     gpio_set_direction(EXTERIOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_level(EXTERIOR_ENABLE_PIN, 0);

//     gpio_reset_pin(INTERIOR_ENABLE_PIN);
//     gpio_set_direction(INTERIOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_level(INTERIOR_ENABLE_PIN, 0);

//     gpio_reset_pin(LIGHT_SIGNAL_1);
//     gpio_set_direction(LIGHT_SIGNAL_1, GPIO_MODE_OUTPUT);
//     gpio_set_level(LIGHT_SIGNAL_1, 0);

//     gpio_reset_pin(LIGHT_SIGNAL_2);
//     gpio_set_direction(LIGHT_SIGNAL_2, GPIO_MODE_OUTPUT);
//     gpio_set_level(LIGHT_SIGNAL_2, 0);
// }

// // Function to initialize UART for both sensors
// void uart_init() {
//     const uart_config_t uart_config = {
//         .baud_rate = UART_BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//     };

//     // Initialize UART0 for exterior sensor communication
//     uart_param_config(UART_PORT_NUM_0, &uart_config);
//     uart_set_pin(UART_PORT_NUM_0, EXTERIOR_TXD1_PIN, EXTERIOR_RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART_PORT_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

//     // Initialize UART1 for interior sensor communication
//     uart_param_config(UART_PORT_NUM_1, &uart_config);
//     uart_set_pin(UART_PORT_NUM_1, INTERIOR_TXD2_PIN, INTERIOR_RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART_PORT_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

//     // Set both UARTs to RS485 half-duplex mode
//     uart_set_mode(UART_PORT_NUM_0, UART_MODE_RS485_HALF_DUPLEX);
//     uart_set_mode(UART_PORT_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
// }

// // Task to handle UART communication and light control for the exterior sensor
// void exterior_sensor_task(void *arg) {
//     uint8_t data[BUF_SIZE];
//     while (1) {
//         int len = uart_read_bytes(UART_PORT_NUM_0, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             ESP_LOGI(TAG, "Exterior sensor triggered, turning on light signal 1");
//             gpio_set_level(LIGHT_SIGNAL_1, 1);
//             vTaskDelay(pdMS_TO_TICKS(600000));  // 10 minutes
//             gpio_set_level(LIGHT_SIGNAL_1, 0);
//         }
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

// // Task to handle UART communication and light control for the interior sensor
// void interior_sensor_task(void *arg) {
//     uint8_t data[BUF_SIZE];
//     while (1) {
//         int len = uart_read_bytes(UART_PORT_NUM_1, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             ESP_LOGI(TAG, "Interior sensor triggered, turning on light signal 2");
//             gpio_set_level(LIGHT_SIGNAL_2, 1);
//             vTaskDelay(pdMS_TO_TICKS(600000));  // 10 minutes
//             gpio_set_level(LIGHT_SIGNAL_2, 0);
//         }
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

// void app_main() {
//     ESP_LOGI(TAG, "Configuring GPIO pins...");
//     configure_gpio();

//     ESP_LOGI(TAG, "Initializing UART...");
//     uart_init();

//     ESP_LOGI(TAG, "Starting exterior and interior sensor tasks...");
//     xTaskCreate(exterior_sensor_task, "exterior_sensor_task", 2048, NULL, 10, NULL);
//     xTaskCreate(interior_sensor_task, "interior_sensor_task", 2048, NULL, 10, NULL);
// }
