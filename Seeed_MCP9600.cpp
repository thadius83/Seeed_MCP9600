// Project:  MCP9600‑Refactor
// File:     Seeed_MCP9600.cpp   Date: 2025‑07‑22
// Summary:  Implementation for MCP9600 new‑I²C driver

#include "Seeed_MCP9600.h"
#include <string.h>
#include <math.h>

/* ---------------------- ctor / setup ----------------------------------- */
MCP9600::MCP9600(uint8_t dev_addr) : addr_(dev_addr) {}

mcp_err_t MCP9600::begin(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz)
{
    if (!bus) return ERROR_PARAM;

    i2c_device_config_t cfg = {};
    cfg.dev_addr      = addr_;
    cfg.scl_speed_hz  = scl_speed_hz;
    esp_err_t e = i2c_master_bus_add_device(bus, &cfg, &dev_);
    return map_err(e);
}

/* --------------------- public high‑level API --------------------------- */
mcp_err_t MCP9600::init(uint8_t therm_type)
{
    uint16_t ver;
    mcp_err_t ret = read_version(&ver);
    if (ret) return ret;

    /* set thermocouple type (bits[6:4]) */
    uint8_t cfg;
    if ((ret = read_byte(THERM_SENS_CFG_REG_ADDR, &cfg))) return ret;
    cfg = (cfg & 0x8F) | (therm_type & 0x70);
    return write_byte(THERM_SENS_CFG_REG_ADDR, cfg);
}

mcp_err_t MCP9600::read_version(uint16_t *ver)
{
    return read_16(VERSION_ID_REG_ADDR, ver);
}

mcp_err_t MCP9600::read_hot_junc(float *value)
{
    int16_t raw = 0;
    mcp_err_t ret = read_16(HOT_JUNCTION_REG_ADDR, (uint16_t *)&raw);
    if (ret == NO_ERROR) *value = raw / 16.0f;
    return ret;
}

mcp_err_t MCP9600::read_junc_temp_delta(float *value)
{
    int16_t raw = 0;
    mcp_err_t ret = read_16(JUNCTION_TEMP_DELTA_REG_ADDR, (uint16_t *)&raw);
    if (ret == NO_ERROR) *value = raw / 16.0f;
    return ret;
}

mcp_err_t MCP9600::read_cold_junc(float *value)
{
    int16_t raw = 0;
    mcp_err_t ret = read_16(COLD_JUNCTION_TEMP_REG_ADDR, (uint16_t *)&raw);
    if (ret == NO_ERROR) *value = raw / 16.0f;
    return ret;
}

mcp_err_t MCP9600::read_ADC_data(uint8_t *data, uint32_t len)
{
    return read_bytes(RAW_ADC_DATA_REG_ADDR, data, len);
}

mcp_err_t MCP9600::read_status(uint8_t *byte)
{
    return read_byte(STAT_REG_ADDR, byte);
}

/* ------------------------ helpers -------------------------------------- */
uint16_t MCP9600::covert_temp_to_reg_form(float t)
{
    bool neg = t < 0;
    if (neg) t = -t;

    uint16_t integer = static_cast<uint16_t>(t);
    float    frac    = t - integer;

    uint8_t H = integer / 16;
    uint8_t L = ((integer % 16) << 4) | static_cast<uint8_t>(roundf(frac / 0.25f)) << 2;
    if (neg) H |= 0x80;

    return (static_cast<uint16_t>(H) << 8) | L;
}

/* ------------------------ low‑level I²C -------------------------------- */
#define MCP_I2C_TIMEOUT  pdMS_TO_TICKS(20)


mcp_err_t MCP9600::write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return map_err(i2c_master_transmit(dev_, buf, 2, MCP_I2C_TIMEOUT));
}

mcp_err_t MCP9600::write_16(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, uint8_t(val >> 8), uint8_t(val) };
    return map_err(i2c_master_transmit(dev_, buf, 3, MCP_I2C_TIMEOUT));
}

mcp_err_t MCP9600::read_byte(uint8_t reg, uint8_t *val)
{
    return map_err(i2c_master_transmit_receive(dev_, &reg, 1, val, 1, MCP_I2C_TIMEOUT));
}

mcp_err_t MCP9600::read_16(uint8_t reg, uint16_t *val)
{
    uint8_t rx[2];
    esp_err_t e = i2c_master_transmit_receive(dev_, &reg, 1, rx, 2, MCP_I2C_TIMEOUT);
    if (e != ESP_OK) return map_err(e);
    *val = (uint16_t(rx[0]) << 8) | rx[1];
    return NO_ERROR;
}

mcp_err_t MCP9600::read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    return map_err(i2c_master_transmit_receive(dev_, &reg, 1, buf, len, MCP_I2C_TIMEOUT));
}

/* ------------------------- error mapper -------------------------------- */
mcp_err_t MCP9600::map_err(esp_err_t err)
{
    return (err == ESP_OK) ? NO_ERROR : ERROR_COMM;
}
