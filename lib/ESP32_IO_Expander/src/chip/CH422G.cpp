/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "driver/i2c.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"

#include "../private/CheckResult.h"
#include "CH422G.h"

/* Timeout of each I2C communication */
#define I2C_TIMEOUT_MS          (10)

#define IO_COUNT                (8)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL     (0xff)
#define OUT_REG_DEFAULT_VAL     (0xdf)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t base;
    i2c_port_t i2c_num;
    uint32_t i2c_address;
    struct {
        uint8_t direction;
        uint8_t output;
    } regs;
} esp_io_expander_ch422g_t;

static const char *TAG = "ch422g";

static esp_err_t esp_io_expander_new_i2c_ch422g(i2c_port_t i2c_num, uint32_t i2c_address, esp_io_expander_handle_t *handle);

ESP_IOExpander_CH422G::~ESP_IOExpander_CH422G()
{
    if (i2c_need_init) {
        i2c_driver_delete(i2c_id);
    }
    if (handle) {
        del();
    }
}

void ESP_IOExpander_CH422G::begin(void)
{
    CHECK_ERROR_RETURN(esp_io_expander_new_i2c_ch422g(i2c_id, i2c_address, &handle));
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

static esp_err_t esp_io_expander_new_i2c_ch422g(i2c_port_t i2c_num, uint32_t i2c_address, esp_io_expander_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(i2c_num < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid i2c num");
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)calloc(1, sizeof(esp_io_expander_ch422g_t));
    ESP_RETURN_ON_FALSE(ch422g, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    ch422g->base.config.io_count = IO_COUNT;
    ch422g->base.config.flags.dir_out_bit_zero = 1;
    ch422g->i2c_num = i2c_num;
    ch422g->i2c_address = i2c_address;
    ch422g->regs.output = OUT_REG_DEFAULT_VAL;
    ch422g->base.read_input_reg = read_input_reg;
    ch422g->base.write_output_reg = write_output_reg;
    ch422g->base.read_output_reg = read_output_reg;
    ch422g->base.write_direction_reg = write_direction_reg;
    ch422g->base.read_direction_reg = read_direction_reg;
    ch422g->base.del = del;
    ch422g->base.reset = reset;

    esp_err_t ret = ESP_OK;
    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&ch422g->base), err, TAG, "Reset failed");

    *handle = &ch422g->base;
    return ESP_OK;
err:
    free(ch422g);
    return ret;
}

#define CH422G_REG_IN 0x26
static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);	

    uint8_t temp = 0;
	
	ESP_RETURN_ON_ERROR(
        i2c_master_read_from_device(ch422g->i2c_num, ch422g->i2c_address, &temp, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Read input reg failed");
	
    // *INDENT-OFF*
    ESP_RETURN_ON_ERROR(
        i2c_master_read_from_device(ch422g->i2c_num, CH422G_REG_IN, &temp, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Read input reg failed");
    // *INDENT-ON*
    *value = temp;
    return ESP_OK;
}

#define CH422G_REG_OUT 0x38
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);
    value &= 0xff;

	uint8_t out_temp = 0x01;
	ESP_RETURN_ON_ERROR(
        i2c_master_write_to_device(ch422g->i2c_num, ch422g->i2c_address, &out_temp, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Write output reg failed");
		
    uint8_t data = (uint8_t)value;
    ESP_RETURN_ON_ERROR(
        i2c_master_write_to_device(ch422g->i2c_num, CH422G_REG_OUT, &data, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Write output reg failed");
    ch422g->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    *value = ch422g->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);
    value &= 0xff;
    ch422g->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    *value = ch422g->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    free(ch422g);
    return ESP_OK;
}
