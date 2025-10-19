/**
 * \file            sht3x.h
 * \brief           SHT3x humidity and temperature sensor library
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
#ifndef SHT3X_HDR_H
#define SHT3X_HDR_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \brief           SHT3x result enumeration
 */
typedef enum {
    SHT3X_OK = 0x00,                           /*!< Everything OK */
    SHT3X_ERR,                                 /*!< Generic error */
    SHT3X_ERR_PARAM,                           /*!< Invalid parameters */
    SHT3X_ERR_I2C,                             /*!< I2C communication error */
    SHT3X_ERR_CRC,                             /*!< CRC check failed */
    SHT3X_ERR_TIMEOUT,                         /*!< Operation timeout */
    SHT3X_ERR_NOT_READY,                       /*!< Sensor not ready */
} sht3x_result_t;

/**
 * \brief           SHT3x I2C address options
 */
typedef enum {
    SHT3X_ADDR_A = 0x44,                       /*!< ADDR pin connected to logic low */
    SHT3X_ADDR_B = 0x45,                       /*!< ADDR pin connected to logic high */
} sht3x_addr_t;

/**
 * \brief           SHT3x measurement repeatability
 */
typedef enum {
    SHT3X_REPEATABILITY_HIGH = 0x00,           /*!< High repeatability */
    SHT3X_REPEATABILITY_MEDIUM,                /*!< Medium repeatability */
    SHT3X_REPEATABILITY_LOW,                   /*!< Low repeatability */
} sht3x_repeatability_t;

/**
 * \brief           SHT3x measurement mode
 */
typedef enum {
    SHT3X_MODE_SINGLE_SHOT = 0x00,             /*!< Single shot mode */
    SHT3X_MODE_PERIODIC,                       /*!< Periodic measurement mode */
} sht3x_mode_t;

/**
 * \brief           SHT3x periodic measurement frequency
 */
typedef enum {
    SHT3X_FREQUENCY_0_5_MPS = 0x00,            /*!< 0.5 measurements per second */
    SHT3X_FREQUENCY_1_MPS,                     /*!< 1 measurement per second */
    SHT3X_FREQUENCY_2_MPS,                     /*!< 2 measurements per second */
    SHT3X_FREQUENCY_4_MPS,                     /*!< 4 measurements per second */
    SHT3X_FREQUENCY_10_MPS,                    /*!< 10 measurements per second */
} sht3x_frequency_t;

/**
 * \brief           SHT3x data structure
 */
typedef struct {
    float temperature;                          /*!< Temperature in degrees Celsius */
    float humidity;                             /*!< Relative humidity in percent */
} sht3x_data_t;

/**
 * \brief           Forward declaration of SHT3x handle
 */
struct sht3x;
typedef struct sht3x sht3x_t;

/**
 * \brief           I2C write function prototype
 * \param[in]       handle: SHT3x handle
 * \param[in]       addr: I2C device address
 * \param[in]       data: Data to write
 * \param[in]       len: Length of data
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
typedef sht3x_result_t (*sht3x_i2c_write_fn)(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len);

/**
 * \brief           I2C read function prototype
 * \param[in]       handle: SHT3x handle
 * \param[in]       addr: I2C device address
 * \param[out]      data: Buffer to read data into
 * \param[in]       len: Length of data to read
 * \return          \ref SHT3X_OK on success, member of \ref sht3x_result_t otherwise
 */
typedef sht3x_result_t (*sht3x_i2c_read_fn)(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len);

/**
 * \brief           Delay function prototype
 * \param[in]       handle: SHT3x handle
 * \param[in]       ms: Delay time in milliseconds
 */
typedef void (*sht3x_delay_fn)(sht3x_t* handle, uint32_t ms);

/**
 * \brief           SHT3x handle structure
 */
struct sht3x {
    sht3x_i2c_write_fn      i2c_write;          /*!< I2C write function */
    sht3x_i2c_read_fn       i2c_read;           /*!< I2C read function */
    sht3x_delay_fn          delay;              /*!< Delay function */
    void*                   user_data;          /*!< User data pointer */
    sht3x_addr_t            addr;               /*!< I2C device address */
    sht3x_repeatability_t   repeatability;     /*!< Measurement repeatability */
    sht3x_mode_t            mode;               /*!< Measurement mode */
    sht3x_frequency_t       frequency;          /*!< Periodic measurement frequency */
    uint8_t                 is_initialized;     /*!< Initialization flag */
};

/* Function prototypes */
sht3x_result_t  sht3x_init(sht3x_t* handle);
sht3x_result_t  sht3x_deinit(sht3x_t* handle);
sht3x_result_t  sht3x_reset(sht3x_t* handle);
sht3x_result_t  sht3x_set_addr(sht3x_t* handle, sht3x_addr_t addr);
sht3x_result_t  sht3x_set_repeatability(sht3x_t* handle, sht3x_repeatability_t repeatability);
sht3x_result_t  sht3x_set_frequency(sht3x_t* handle, sht3x_frequency_t frequency);
sht3x_result_t  sht3x_start_single_shot(sht3x_t* handle);
sht3x_result_t  sht3x_start_periodic(sht3x_t* handle);
sht3x_result_t  sht3x_stop_periodic(sht3x_t* handle);
sht3x_result_t  sht3x_read_data(sht3x_t* handle, sht3x_data_t* data);
sht3x_result_t  sht3x_is_data_ready(sht3x_t* handle, uint8_t* ready);
sht3x_result_t  sht3x_read_status(sht3x_t* handle, uint16_t* status);
sht3x_result_t  sht3x_clear_status(sht3x_t* handle);
sht3x_result_t  sht3x_enable_heater(sht3x_t* handle);
sht3x_result_t  sht3x_disable_heater(sht3x_t* handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SHT3X_HDR_H */


