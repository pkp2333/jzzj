#ifndef BUS_SERVO_CONTROL_H
#define BUS_SERVO_CONTROL_H

#include "usart.h"

void bus_servo_control(uint8_t id, uint16_t value, uint16_t time, UART_HandleTypeDef huart);
void bus_servo_Unloading(uint8_t id, uint8_t value, UART_HandleTypeDef huart);
void bus_servo_set_id(uint8_t id, UART_HandleTypeDef huart);
void bus_servo_read(uint8_t id, UART_HandleTypeDef huart);
void BusServo_Init(void);

#endif
