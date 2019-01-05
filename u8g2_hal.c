/*
 * u8g2_hal.c
 *
 *  Created on: 28 dic. 2018
 *      Author: portodur
 */


#include "u8g2_hal.h"

uint8_t u8g2_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch(msg) {

	case U8X8_MSG_BYTE_SEND: {
		uint8_t* data_ptr = (uint8_t*)arg_ptr;

		while( arg_int > 0 ) {
			ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, *data_ptr, ACK_CHECK_EN));
			data_ptr++;
			arg_int--;
		}
		break;
	}

	case U8X8_MSG_BYTE_START_TRANSFER: {
		uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
		handle_i2c = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(handle_i2c));
		ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));
		break;
	}

	case U8X8_MSG_BYTE_END_TRANSFER: {
		ESP_ERROR_CHECK(i2c_master_stop(handle_i2c));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_1, handle_i2c, 1000 / portTICK_RATE_MS));
		i2c_cmd_link_delete(handle_i2c);
		break;
	}
	}
	return 0;
}

uint8_t u8g2_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch(msg) {

	case U8X8_MSG_DELAY_MILLI:
		vTaskDelay(arg_int/portTICK_PERIOD_MS);
		break;
	}
	return 0;
}
