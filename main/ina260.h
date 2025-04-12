#ifndef INA260_H
#define INA260_H

#include <stdint.h>
#include "esp_err.h"

void ina260_init(void);
void ina260_task(void *arg);

esp_err_t ina260_read_register(uint8_t reg, uint16_t *value);
float ina260_raw_to_current(uint16_t raw);
float ina260_raw_to_voltage(uint16_t raw);
float ina260_raw_to_power(uint16_t raw);

#endif // INA260_H
