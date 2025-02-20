// /* Uart Events Example

//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "driver/uart.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "sdkconfig.h"

// /**
//  * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
// */
// #define TAG "RS485_ECHO_APP"

// // Note: Some pins on target chip cannot be assigned for UART communication.
// // Please refer to documentation for selected board and target to configure pins using Kconfig.
// #define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
// #define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)

// // RTS for RS485 Half-Duplex Mode manages DE/~RE
// #define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)

// // CTS is not used in RS485 Half-Duplex Mode
// #define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)

// #define BUF_SIZE        (127)
// #define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)

// // Read packet timeout
// #define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
// #define ECHO_TASK_STACK_SIZE    (2048)
// #define ECHO_TASK_PRIO          (10)
// #define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)

// // Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
// #define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

// static void echo_send(const int port, const char* str, uint8_t length)
// {
//     if (uart_write_bytes(port, str, length) != length) {
//         ESP_LOGE(TAG, "Send data critical failure.");
//         // add your code to handle sending failure here
//         abort();
//     }
// }

// // An example of echo test with hardware flow control on UART
// static void echo_task(void *arg)
// {
//     const int uart_num = ECHO_UART_PORT;
//     uart_config_t uart_config = {
//         .baud_rate = BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .rx_flow_ctrl_thresh = 122,
//         .source_clk = UART_SCLK_DEFAULT,
//     };

//     // Set UART log level
//     esp_log_level_set(TAG, ESP_LOG_INFO);

//     ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

//     // Install UART driver (we don't need an event queue here)
//     // In this example we don't even use a buffer for sending data.
//     ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

//     // Configure UART parameters
//     ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

//     ESP_LOGI(TAG, "UART set pins, mode and install driver.");

//     // Set UART pins as per KConfig settings
//     ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

//     // Set RS485 half duplex mode
//     ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

//     // Set read timeout of UART TOUT feature
//     ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));

//     // Allocate buffers for UART
//     uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

//     ESP_LOGI(TAG, "UART start recieve loop.\r");
//     echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

//     while (1) {
//         //Read data from UART
//         int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);

//         //Write data back to UART
//         if (len > 0) {
//             echo_send(uart_num, "\r\n", 2);
//             char prefix[] = "RS485 Received: [";
//             echo_send(uart_num, prefix, (sizeof(prefix) - 1));
//             ESP_LOGI(TAG, "Received %u bytes:", len);
//             printf("[ ");
//             for (int i = 0; i < len; i++) {
//                 printf("0x%.2X ", (uint8_t)data[i]);
//                 echo_send(uart_num, (const char*)&data[i], 1);
//                 // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
//                 if (data[i] == '\r') {
//                     echo_send(uart_num, "\n", 1);
//                 }
//             }
//             printf("] \n");
//             echo_send(uart_num, "]\r\n", 3);
//         } else {
//             // Echo a "." to show we are alive while we wait for input
//             echo_send(uart_num, ".", 1);
//             ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
//         }
//     }
//     vTaskDelete(NULL);
// }

// void app_main(void)
// {
//     //A uart read/write example without event queue;
//     xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
// }


#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#define TAG "RS485_ECHO_APP"

// UART and GPIO Pin Definitions
#define EXTERIOR_TXD1_PIN (GPIO_NUM_16)   // UART TX1 for exterior
#define EXTERIOR_RXD1_PIN (GPIO_NUM_14)   // UART RX1 for exterior

#define INTERIOR_TXD2_PIN (GPIO_NUM_30)   // UART TX2 for interior
#define INTERIOR_RXD2_PIN (GPIO_NUM_28)   // UART RX2 for interior

#define EXTERIOR_ENABLE_PIN (GPIO_NUM_26)  // IO4 for THVD1 (exterior)
#define INTERIOR_ENABLE_PIN (GPIO_NUM_29)  // IO5 for THVD2 (interior)

#define LIGHT_SIGNAL_1 (GPIO_NUM_33)       // Exterior light signal pin
#define LIGHT_SIGNAL_2 (GPIO_NUM_36)       // Interior light signal pin

#define UART_PORT_NUM_0 UART_NUM_0         // Use UART0 for default UART communication
#define UART_PORT_NUM_1 UART_NUM_1         // Use UART1 for RS485 communication

#define UART_BAUD_RATE 115200 // rate maybe 115200
#define BUF_SIZE (1024)

// Function to configure GPIO pins
void configure_gpio() {
    gpio_reset_pin(EXTERIOR_ENABLE_PIN);                          // Exterior light configuration
    gpio_set_direction(EXTERIOR_ENABLE_PIN, GPIO_MODE_OUTPUT);    // Exterior light configuration
    gpio_set_level(EXTERIOR_ENABLE_PIN, 0);                       // Exterior light configuration

    gpio_reset_pin(INTERIOR_ENABLE_PIN);                          // interior light configuration
    gpio_set_direction(INTERIOR_ENABLE_PIN, GPIO_MODE_OUTPUT);    // interior light configuration
    gpio_set_level(INTERIOR_ENABLE_PIN, 0);                       // interior light configuration

    gpio_reset_pin(LIGHT_SIGNAL_1);                               // light 1 configuration
    gpio_set_direction(LIGHT_SIGNAL_1, GPIO_MODE_OUTPUT);         // light 1 configuration
    gpio_set_level(LIGHT_SIGNAL_1, 0);                            // light 1 configuration

    gpio_reset_pin(LIGHT_SIGNAL_2);                               // light 2 configuration
    gpio_set_direction(LIGHT_SIGNAL_2, GPIO_MODE_OUTPUT);         // light 2 configuration
    gpio_set_level(LIGHT_SIGNAL_2, 0);                            // light 2 configuration
}

// Function to initialize UART for both sensors
// Function to initialize UART for both sensors
void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };



    // Initialize UART0 for exterior sensor communication
    uart_param_config(UART_PORT_NUM_0, &uart_config);
    uart_set_pin(UART_PORT_NUM_0, EXTERIOR_TXD1_PIN, EXTERIOR_RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Initialize UART1 for interior sensor communication
    uart_param_config(UART_PORT_NUM_1, &uart_config);
    uart_set_pin(UART_PORT_NUM_1, INTERIOR_TXD2_PIN, INTERIOR_RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Set both UARTs to RS485 half-duplex mode
    uart_set_mode(UART_PORT_NUM_0, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_mode(UART_PORT_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
}

// Task to handle UART communication and light control for the exterior sensor
void exterior_sensor_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM_0, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Exterior sensor triggered, turning on light signal 1");
            gpio_set_level(LIGHT_SIGNAL_1, 1);
            vTaskDelay(pdMS_TO_TICKS(600000));  // 10 minutes
            gpio_set_level(LIGHT_SIGNAL_1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to handle UART communication and light control for the interior sensor
void interior_sensor_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM_1, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Interior sensor triggered, turning on light signal 2");
            gpio_set_level(LIGHT_SIGNAL_2, 1);
            vTaskDelay(pdMS_TO_TICKS(600000));  // 10 minutes
            gpio_set_level(LIGHT_SIGNAL_2, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    ESP_LOGI(TAG, "Configuring GPIO pins...");
    configure_gpio();

    ESP_LOGI(TAG, "Initializing UART...");
    uart_init();

    ESP_LOGI(TAG, "Starting exterior and interior sensor tasks...");
    xTaskCreate(exterior_sensor_task, "exterior_sensor_task", 2048, NULL, 10, NULL);
    xTaskCreate(interior_sensor_task, "interior_sensor_task", 2048, NULL, 10, NULL);
}
