/**
 * \file            sht3x.c
 * \brief           SHT3x humidity and temperature sensor library implementation
 */

/*
 * Copyright (c) 2024 Pham Nam Hien
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of SHT3x library.
 *
 * Author:          Pham Nam Hien <phamnamhien@gmail.com>
 */
#include "sht3x.h"

/* SHT3x command definitions */
#define SHT3X_CMD_SINGLE_SHOT_HIGH_REP_CS_EN    0x2C06
#define SHT3X_CMD_SINGLE_SHOT_MED_REP_CS_EN     0x2C0D
#define SHT3X_CMD_SINGLE_SHOT_LOW_REP_CS_EN     0x2C10
#define SHT3X_CMD_SINGLE_SHOT_HIGH_REP_CS_DIS   0x2400
#define SHT3X_CMD_SINGLE_SHOT_MED_REP_CS_DIS    0x240B
#define SHT3X_CMD_SINGLE_SHOT_LOW_REP_CS_DIS    0x2416

#define SHT3X_CMD_PERIODIC_0_5_MPS_HIGH_REP     0x2032
#define SHT3X_CMD_PERIODIC_0_5_MPS_MED_REP      0x2024
#define SHT3X_CMD_PERIODIC_0_5_MPS_LOW_REP      0x202F
#define SHT3X_CMD_PERIODIC_1_MPS_HIGH_REP       0x2130
#define SHT3X_CMD_PERIODIC_1_MPS_MED_REP        0x2126
#define SHT3X_CMD_PERIODIC_1_MPS_LOW_REP        0x212D
#define SHT3X_CMD_PERIODIC_2_MPS_HIGH_REP       0x2236
#define SHT3X_CMD_PERIODIC_2_MPS_MED_REP        0x2220
#define SHT3X_CMD_PERIODIC_2_MPS_LOW_REP        0x222B
#define SHT3X_CMD_PERIODIC_4_MPS_HIGH_REP       0x2334
#define SHT3X_CMD_PERIODIC_4_MPS_MED_REP        0x2322
#define SHT3X_CMD_PERIODIC_4_MPS_LOW_REP        0x2329
#define SHT3X_CMD_PERIODIC_10_MPS_HIGH_REP      0x2737
#define SHT3X_CMD_PERIODIC_10_MPS_MED_REP       0x2721
#define SHT3X_CMD_PERIODIC_10_MPS_LOW_REP       0x272A

#define SHT3X_CMD_FETCH_DATA                    0xE000
#define SHT3X_CMD_BREAK                         0x3093
#define SHT3X_CMD_SOFT_RESET                    0x30A2
#define SHT3X_CMD_HEATER_ENABLE                 0x306D
#define SHT3X_CMD_HEATER_DISABLE                0x3066
#define SHT3X_CMD_READ_STATUS                   0xF32D
#define SHT3X_CMD_CLEAR_STATUS                  0x3041

/* CRC parameters */
#define SHT3X_CRC_POLYNOMIAL                    0x31
#define SHT3X_CRC_INIT                          0xFF

/* Timing definitions */
#define SHT3X_RESET_DELAY_MS                    1
#define SHT3X_MEASUREMENT_DELAY_HIGH_MS         15
#define SHT3X_MEASUREMENT_DELAY_MED_MS          6
#define SHT3X_MEASUREMENT_DELAY_LOW_MS          4

/* Private function prototypes */
static sht3x_result_t   prv_write_command(sht3x_t* handle, uint16_t cmd);
static sht3x_result_t   prv_read_data_raw(sht3x_t* handle, uint8_t* data, size_t len);
static uint8_t          prv_calculate_crc(const uint8_t* data, size_t len);
static uint8_t          prv_verify_crc(const uint8_t* data, size_t len, uint8_t crc);
static sht3x_result_t   prv_convert_raw_data(const uint8_t* raw_data, sht3x_data_t* data);
static uint16_t         prv_get_single_shot_command(sht3x_repeatability_t repeatability, uint8_t clock_stretching);
static uint16_t         prv_get_periodic_command(sht3x_frequency_t frequency, sht3x_repeatability_t repeatability);
static uint32_t         prv_get_measurement_delay(sht3x_repeatability_t repeatability);

/**
 * \brief           Initialize SHT3x sensor
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_init(sht3x_t* handle) {
    if (handle == NULL || handle->i2c_write == NULL || handle->i2c_read == NULL || handle->delay == NULL) {
        return SHT3X_ERR_PARAM;
    }

    /* Set default values */
    handle->repeatability = SHT3X_REPEATABILITY_HIGH;
    handle->mode = SHT3X_MODE_SINGLE_SHOT;
    handle->frequency = SHT3X_FREQUENCY_1_MPS;
    handle->is_initialized = 0;

    /* Perform soft reset */
    if (sht3x_reset(handle) != SHT3X_OK) {
        return SHT3X_ERR;
    }

    handle->is_initialized = 1;
    return SHT3X_OK;
}

/**
 * \brief           Deinitialize SHT3x sensor
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_deinit(sht3x_t* handle) {
    if (handle == NULL) {
        return SHT3X_ERR_PARAM;
    }

    /* Stop periodic measurement if running */
    if (handle->mode == SHT3X_MODE_PERIODIC) {
        sht3x_stop_periodic(handle);
    }

    handle->is_initialized = 0;
    return SHT3X_OK;
}

/**
 * \brief           Reset SHT3x sensor
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_reset(sht3x_t* handle) {
    sht3x_result_t result;

    if (handle == NULL) {
        return SHT3X_ERR_PARAM;
    }

    result = prv_write_command(handle, SHT3X_CMD_SOFT_RESET);
    if (result != SHT3X_OK) {
        return result;
    }

    /* Wait for reset completion */
    handle->delay(handle, SHT3X_RESET_DELAY_MS);
    return SHT3X_OK;
}

/**
 * \brief           Set I2C address for SHT3x sensor
 * \param[in]       handle: SHT3x handle
 * \param[in]       addr: I2C address
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_set_addr(sht3x_t* handle, sht3x_addr_t addr) {
    if (handle == NULL) {
        return SHT3X_ERR_PARAM;
    }

    handle->addr = addr;
    return SHT3X_OK;
}

/**
 * \brief           Set measurement repeatability
 * \param[in]       handle: SHT3x handle
 * \param[in]       repeatability: Measurement repeatability
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_set_repeatability(sht3x_t* handle, sht3x_repeatability_t repeatability) {
    if (handle == NULL) {
        return SHT3X_ERR_PARAM;
    }

    handle->repeatability = repeatability;
    return SHT3X_OK;
}

/**
 * \brief           Set periodic measurement frequency
 * \param[in]       handle: SHT3x handle
 * \param[in]       frequency: Measurement frequency
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_set_frequency(sht3x_t* handle, sht3x_frequency_t frequency) {
    if (handle == NULL) {
        return SHT3X_ERR_PARAM;
    }

    handle->frequency = frequency;
    return SHT3X_OK;
}

/**
 * \brief           Start single shot measurement
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_start_single_shot(sht3x_t* handle) {
    uint16_t cmd;
    sht3x_result_t result;

    if (handle == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    cmd = prv_get_single_shot_command(handle->repeatability, 0);
    result = prv_write_command(handle, cmd);
    if (result != SHT3X_OK) {
        return result;
    }

    handle->mode = SHT3X_MODE_SINGLE_SHOT;

    /* Wait for measurement completion */
    handle->delay(handle, prv_get_measurement_delay(handle->repeatability));
    return SHT3X_OK;
}

/**
 * \brief           Start periodic measurement
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_start_periodic(sht3x_t* handle) {
    uint16_t cmd;
    sht3x_result_t result;

    if (handle == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    cmd = prv_get_periodic_command(handle->frequency, handle->repeatability);
    result = prv_write_command(handle, cmd);
    if (result != SHT3X_OK) {
        return result;
    }

    handle->mode = SHT3X_MODE_PERIODIC;
    return SHT3X_OK;
}

/**
 * \brief           Stop periodic measurement
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_stop_periodic(sht3x_t* handle) {
    sht3x_result_t result;

    if (handle == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    result = prv_write_command(handle, SHT3X_CMD_BREAK);
    if (result != SHT3X_OK) {
        return result;
    }

    handle->mode = SHT3X_MODE_SINGLE_SHOT;
    handle->delay(handle, 1);
    return SHT3X_OK;
}

/**
 * \brief           Read measurement data
 * \param[in]       handle: SHT3x handle
 * \param[out]      data: Pointer to data structure
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_read_data(sht3x_t* handle, sht3x_data_t* data) {
    uint8_t raw_data[6];
    sht3x_result_t result;

    if (handle == NULL || data == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    if (handle->mode == SHT3X_MODE_PERIODIC) {
        /* Send fetch data command for periodic mode */
        result = prv_write_command(handle, SHT3X_CMD_FETCH_DATA);
        if (result != SHT3X_OK) {
            return result;
        }
    }

    /* Read 6 bytes of data (temp + humidity + CRCs) */
    result = prv_read_data_raw(handle, raw_data, sizeof(raw_data));
    if (result != SHT3X_OK) {
        return result;
    }

    /* Verify CRC for temperature data */
    if (!prv_verify_crc(&raw_data[0], 2, raw_data[2])) {
        return SHT3X_ERR_CRC;
    }

    /* Verify CRC for humidity data */
    if (!prv_verify_crc(&raw_data[3], 2, raw_data[5])) {
        return SHT3X_ERR_CRC;
    }

    /* Convert raw data to physical values */
    return prv_convert_raw_data(raw_data, data);
}

/**
 * \brief           Check if measurement data is ready
 * \param[in]       handle: SHT3x handle
 * \param[out]      ready: Pointer to ready flag
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_is_data_ready(sht3x_t* handle, uint8_t* ready) {
    uint8_t dummy[6];
    sht3x_result_t result;

    if (handle == NULL || ready == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    *ready = 0;

    if (handle->mode == SHT3X_MODE_PERIODIC) {
        /* Try to fetch data */
        result = prv_write_command(handle, SHT3X_CMD_FETCH_DATA);
        if (result != SHT3X_OK) {
            return result;
        }

        /* Try to read data */
        result = prv_read_data_raw(handle, dummy, sizeof(dummy));
        if (result == SHT3X_OK) {
            *ready = 1;
        }
    } else {
        /* For single shot mode, assume data is ready after measurement delay */
        *ready = 1;
    }

    return SHT3X_OK;
}

/**
 * \brief           Read status register
 * \param[in]       handle: SHT3x handle
 * \param[out]      status: Pointer to status value
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_read_status(sht3x_t* handle, uint16_t* status) {
    uint8_t data[3];
    sht3x_result_t result;

    if (handle == NULL || status == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    result = prv_write_command(handle, SHT3X_CMD_READ_STATUS);
    if (result != SHT3X_OK) {
        return result;
    }

    result = prv_read_data_raw(handle, data, sizeof(data));
    if (result != SHT3X_OK) {
        return result;
    }

    /* Verify CRC */
    if (!prv_verify_crc(&data[0], 2, data[2])) {
        return SHT3X_ERR_CRC;
    }

    *status = ((uint16_t)data[0] << 8) | data[1];
    return SHT3X_OK;
}

/**
 * \brief           Clear status register
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_clear_status(sht3x_t* handle) {
    if (handle == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    return prv_write_command(handle, SHT3X_CMD_CLEAR_STATUS);
}

/**
 * \brief           Enable internal heater
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_enable_heater(sht3x_t* handle) {
    if (handle == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    return prv_write_command(handle, SHT3X_CMD_HEATER_ENABLE);
}

/**
 * \brief           Disable internal heater
 * \param[in]       handle: SHT3x handle
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
sht3x_result_t
sht3x_disable_heater(sht3x_t* handle) {
    if (handle == NULL || !handle->is_initialized) {
        return SHT3X_ERR_PARAM;
    }

    return prv_write_command(handle, SHT3X_CMD_HEATER_DISABLE);
}

/**
 * \brief           Write command to SHT3x sensor
 * \param[in]       handle: SHT3x handle
 * \param[in]       cmd: Command to write
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
static sht3x_result_t
prv_write_command(sht3x_t* handle, uint16_t cmd) {
    uint8_t data[2];

    data[0] = (uint8_t)(cmd >> 8);
    data[1] = (uint8_t)(cmd & 0xFF);

    return handle->i2c_write(handle, handle->addr, data, sizeof(data));
}

/**
 * \brief           Read raw data from SHT3x sensor
 * \param[in]       handle: SHT3x handle
 * \param[out]      data: Buffer to store read data
 * \param[in]       len: Length of data to read
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
static sht3x_result_t
prv_read_data_raw(sht3x_t* handle, uint8_t* data, size_t len) {
    return handle->i2c_read(handle, handle->addr, data, len);
}

/**
 * \brief           Calculate CRC8 checksum
 * \param[in]       data: Data to calculate CRC for
 * \param[in]       len: Length of data
 * \return          CRC8 checksum
 */
static uint8_t
prv_calculate_crc(const uint8_t* data, size_t len) {
    uint8_t crc = SHT3X_CRC_INIT;
    size_t i, j;

    for (i = 0; i < len; ++i) {
        crc ^= data[i];
        for (j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SHT3X_CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * \brief           Verify CRC8 checksum
 * \param[in]       data: Data to verify
 * \param[in]       len: Length of data
 * \param[in]       crc: Expected CRC value
 * \return          `1` if CRC is valid, `0` otherwise
 */
static uint8_t
prv_verify_crc(const uint8_t* data, size_t len, uint8_t crc) {
    return prv_calculate_crc(data, len) == crc;
}

/**
 * \brief           Convert raw sensor data to physical values
 * \param[in]       raw_data: Raw sensor data
 * \param[out]      data: Converted data structure
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
static sht3x_result_t
prv_convert_raw_data(const uint8_t* raw_data, sht3x_data_t* data) {
    uint16_t temp_raw, hum_raw;

    /* Extract temperature and humidity raw values */
    temp_raw = ((uint16_t)raw_data[0] << 8) | raw_data[1];
    hum_raw = ((uint16_t)raw_data[3] << 8) | raw_data[4];

    /* Convert to physical values according to datasheet formulas */
    data->temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    data->humidity = 100.0f * ((float)hum_raw / 65535.0f);

    /* Clamp humidity to valid range */
    if (data->humidity > 100.0f) {
        data->humidity = 100.0f;
    } else if (data->humidity < 0.0f) {
        data->humidity = 0.0f;
    }

    return SHT3X_OK;
}

/**
 * \brief           Get single shot measurement command
 * \param[in]       repeatability: Measurement repeatability
 * \param[in]       clock_stretching: Clock stretching enabled flag
 * \return          Command value
 */
static uint16_t
prv_get_single_shot_command(sht3x_repeatability_t repeatability, uint8_t clock_stretching) {
    if (clock_stretching) {
        switch (repeatability) {
            case SHT3X_REPEATABILITY_HIGH:
                return SHT3X_CMD_SINGLE_SHOT_HIGH_REP_CS_EN;
            case SHT3X_REPEATABILITY_MEDIUM:
                return SHT3X_CMD_SINGLE_SHOT_MED_REP_CS_EN;
            case SHT3X_REPEATABILITY_LOW:
                return SHT3X_CMD_SINGLE_SHOT_LOW_REP_CS_EN;
            default:
                return SHT3X_CMD_SINGLE_SHOT_HIGH_REP_CS_EN;
        }
    } else {
        switch (repeatability) {
            case SHT3X_REPEATABILITY_HIGH:
                return SHT3X_CMD_SINGLE_SHOT_HIGH_REP_CS_DIS;
            case SHT3X_REPEATABILITY_MEDIUM:
                return SHT3X_CMD_SINGLE_SHOT_MED_REP_CS_DIS;
            case SHT3X_REPEATABILITY_LOW:
                return SHT3X_CMD_SINGLE_SHOT_LOW_REP_CS_DIS;
            default:
                return SHT3X_CMD_SINGLE_SHOT_HIGH_REP_CS_DIS;
        }
    }
}

/**
 * \brief           Get periodic measurement command
 * \param[in]       frequency: Measurement frequency
 * \param[in]       repeatability: Measurement repeatability
 * \return          Command value
 */
static uint16_t
prv_get_periodic_command(sht3x_frequency_t frequency, sht3x_repeatability_t repeatability) {
    switch (frequency) {
        case SHT3X_FREQUENCY_0_5_MPS:
            switch (repeatability) {
                case SHT3X_REPEATABILITY_HIGH:
                    return SHT3X_CMD_PERIODIC_0_5_MPS_HIGH_REP;
                case SHT3X_REPEATABILITY_MEDIUM:
                    return SHT3X_CMD_PERIODIC_0_5_MPS_MED_REP;
                case SHT3X_REPEATABILITY_LOW:
                    return SHT3X_CMD_PERIODIC_0_5_MPS_LOW_REP;
                default:
                    return SHT3X_CMD_PERIODIC_0_5_MPS_HIGH_REP;
            }
        case SHT3X_FREQUENCY_1_MPS:
            switch (repeatability) {
                case SHT3X_REPEATABILITY_HIGH:
                    return SHT3X_CMD_PERIODIC_1_MPS_HIGH_REP;
                case SHT3X_REPEATABILITY_MEDIUM:
                    return SHT3X_CMD_PERIODIC_1_MPS_MED_REP;
                case SHT3X_REPEATABILITY_LOW:
                    return SHT3X_CMD_PERIODIC_1_MPS_LOW_REP;
                default:
                    return SHT3X_CMD_PERIODIC_1_MPS_HIGH_REP;
            }
        case SHT3X_FREQUENCY_2_MPS:
            switch (repeatability) {
                case SHT3X_REPEATABILITY_HIGH:
                    return SHT3X_CMD_PERIODIC_2_MPS_HIGH_REP;
                case SHT3X_REPEATABILITY_MEDIUM:
                    return SHT3X_CMD_PERIODIC_2_MPS_MED_REP;
                case SHT3X_REPEATABILITY_LOW:
                    return SHT3X_CMD_PERIODIC_2_MPS_LOW_REP;
                default:
                    return SHT3X_CMD_PERIODIC_2_MPS_HIGH_REP;
            }
        case SHT3X_FREQUENCY_4_MPS:
            switch (repeatability) {
                case SHT3X_REPEATABILITY_HIGH:
                    return SHT3X_CMD_PERIODIC_4_MPS_HIGH_REP;
                case SHT3X_REPEATABILITY_MEDIUM:
                    return SHT3X_CMD_PERIODIC_4_MPS_MED_REP;
                case SHT3X_REPEATABILITY_LOW:
                    return SHT3X_CMD_PERIODIC_4_MPS_LOW_REP;
                default:
                    return SHT3X_CMD_PERIODIC_4_MPS_HIGH_REP;
            }
        case SHT3X_FREQUENCY_10_MPS:
            switch (repeatability) {
                case SHT3X_REPEATABILITY_HIGH:
                    return SHT3X_CMD_PERIODIC_10_MPS_HIGH_REP;
                case SHT3X_REPEATABILITY_MEDIUM:
                    return SHT3X_CMD_PERIODIC_10_MPS_MED_REP;
                case SHT3X_REPEATABILITY_LOW:
                    return SHT3X_CMD_PERIODIC_10_MPS_LOW_REP;
                default:
                    return SHT3X_CMD_PERIODIC_10_MPS_HIGH_REP;
            }
        default:
            return SHT3X_CMD_PERIODIC_1_MPS_HIGH_REP;
    }
}

/**
 * \brief           Get measurement delay based on repeatability
 * \param[in]       repeatability: Measurement repeatability
 * \return          Delay in milliseconds
 */
static uint32_t
prv_get_measurement_delay(sht3x_repeatability_t repeatability) {
    switch (repeatability) {
        case SHT3X_REPEATABILITY_HIGH:
            return SHT3X_MEASUREMENT_DELAY_HIGH_MS;
        case SHT3X_REPEATABILITY_MEDIUM:
            return SHT3X_MEASUREMENT_DELAY_MED_MS;
        case SHT3X_REPEATABILITY_LOW:
            return SHT3X_MEASUREMENT_DELAY_LOW_MS;
        default:
            return SHT3X_MEASUREMENT_DELAY_HIGH_MS;
    }
}

