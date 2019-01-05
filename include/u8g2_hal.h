/*
 * u8g2_hal.h
 *
 *  Created on: 28 dic. 2018
 *      Author: portodur
 */

#ifndef U8G2_HAL_H_
#define U8G2_HAL_H_

#include <stdint.h>
#include <stddef.h>
#include "driver/i2c.h"
#include "u8g2.h"
#include "driver/gpio.h"

#define ACK_CHECK_EN   0x1
#define ACK_CHECK_DIS  0x0

i2c_cmd_handle_t handle_i2c;

uint8_t u8g2_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);



#endif /*U8G2_HAL_H_ */
