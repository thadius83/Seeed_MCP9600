// Project:  MCP9600‑Refactor (ESP‑IDF new I²C driver)
// File:     Seeed_MCP9600.h   Date: 2025‑07‑22
// SPDX-License-Identifier: MIT
#pragma once

#include <driver/i2c_master.h>   // new driver
#include <freertos/FreeRTOS.h>   // pdMS_TO_TICKS()
#include <stdint.h>

/* ───────── type aliases ───────── */
using  s32 = int32_t;   using  u32 = uint32_t;
using  s16 = int16_t;   using  u16 = uint16_t;
using  s8  = int8_t;    using  u8  = uint8_t;

/* ───────── error map ───────── */
enum mcp_err_t : int8_t {
    NO_ERROR      =  0,
    ERROR_PARAM   = -1,
    ERROR_COMM    = -2,
    ERROR_OTHERS  = -128,
};

/* ───────── register map ───────── */
#define HOT_JUNCTION_REG_ADDR               0x00
#define JUNCTION_TEMP_DELTA_REG_ADDR        0x01
#define COLD_JUNCTION_TEMP_REG_ADDR         0x02
#define RAW_ADC_DATA_REG_ADDR               0x03
#define STAT_REG_ADDR                       0x04
#define THERM_SENS_CFG_REG_ADDR             0x05
#define DEVICE_CFG_REG_ADDR                 0x06
#define ALERT1_CFG_REG_ADDR                 0x08
#define TEMP_ALERT1_LIMIT_REG_ADDR          0x10
#define VERSION_ID_REG_ADDR                 0x20
#define DEFAULT_IIC_ADDR                    0x60

/* macros the sketch still uses */
#define THER_TYPE_K               (0x0 << 4)
#define ADC_18BIT_RESOLUTION      (0x0 << 5)

/* ───────── driver class ───────── */
class MCP9600 {
public:
    explicit MCP9600(uint8_t dev_addr = DEFAULT_IIC_ADDR);

    /* attach device to shared bus */
    mcp_err_t begin(i2c_master_bus_handle_t bus,
                    uint32_t                scl_speed_hz = 400000);

    /* original public API */
    mcp_err_t init(uint8_t therm_type);
    mcp_err_t read_version(uint16_t *ver);

    mcp_err_t read_hot_junc(float *value);
    mcp_err_t read_junc_temp_delta(float *value);
    mcp_err_t read_cold_junc(float *value);
    mcp_err_t read_ADC_data(uint8_t *data, uint32_t len);
    mcp_err_t read_status(uint8_t *byte);

    mcp_err_t set_filt_coefficients(uint8_t byte);
    mcp_err_t set_ADC_meas_resolution(uint8_t byte);

    uint16_t  covert_temp_to_reg_form(float temp);

private:
    /* low‑level helpers */
    mcp_err_t write_byte(uint8_t reg, uint8_t val);
    mcp_err_t write_16(uint8_t reg, uint16_t val);
    mcp_err_t read_byte(uint8_t reg, uint8_t *val);
    mcp_err_t read_16(uint8_t reg, uint16_t *val);
    mcp_err_t read_bytes(uint8_t reg, uint8_t *buf, size_t len);

    static mcp_err_t map_err(esp_err_t err);

    i2c_master_dev_handle_t dev_ = nullptr;
    uint8_t                 addr_;
};
