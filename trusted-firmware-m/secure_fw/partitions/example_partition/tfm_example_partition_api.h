/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __TFM_EXAMPLE_PARTITION_API_H__
#define __TFM_EXAMPLE_PARTITION_API_H__

#include <stdint.h>
#include <stddef.h>

#include "psa/error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Makes a psa_call to the example partition.
 *
 * \param[in] arg  Example parameter
 *
 * \return Returns error code as specified in \ref psa_status_t
 */
/* psa_status_t psa_example_service(uint32_t arg); */

/**
 * \brief Calculate SHA-256 hash of a given input message.
 *
 * \param[in]   input           Message to be hashed
 * \param[in]   input_length    Length of message in bytes
 * \param[out]  hash            Buffer to which calculated hash is
 *                              written into
 * \param[in]   hash_buf_size   Size of hash buffer in bytes
 * \param[out]  hash_len        Actual size of calculated hash in bytes
 *
 * \return Returns error code as specified in \ref psa_status_t
 */
psa_status_t psa_example_hash(const uint8_t *input,
                             size_t input_length,
				             uint8_t *hash,
                             size_t hash_buf_size,
				             size_t *hash_len);

/**
 * \brief Read magnetometer (LSM303) data.
 *
 * \param[out]  data            Buffer to which magnetometer data is
 *                              written into
 * \param[out]   data_size      Size of magnetometer data in bytes
 *
 * \return Returns error code as specified in \ref psa_status_t
 */
psa_status_t example_read_lsm303(uint8_t *data,
                                size_t data_size);

/**
 * \brief Run secure inference to get the sine value of input
 *
 * \param[in]   input               Angle in degrees
 * \param[in]   input_length        Length of input in bytes
 * \param[out]  sine_value_buf      Buffer to which calculated sine value
 *                                  is written into
 * \param[in]   sine_value_buf_len  Size of sine_value_buf in bytes
 *
 * \return Returns error code as specified in \ref psa_status_t
 */
psa_status_t psa_example_tflm_hello(const float *input,
                                    size_t input_length,
                                    float *sine_value_buf,
                                    size_t sine_value_buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __TFM_EXAMPLE_PARTITION_API_H__ */
