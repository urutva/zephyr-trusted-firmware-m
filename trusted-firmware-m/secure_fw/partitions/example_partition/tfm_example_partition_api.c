/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "tfm_example_partition_api.h"

#include "psa/client.h"
#include "psa_manifest/sid.h"

/* Provide an API to the non-secure side to invoke the secure service. */
/*
psa_status_t psa_example_service(uint32_t arg)
{
    psa_status_t status;
    psa_handle_t handle;
    psa_invec in_vec[] = {
        { .base = &arg, .len = sizeof(arg) },
    };

    handle = psa_connect(TFM_EXAMPLE_SERVICE_SID, TFM_EXAMPLE_SERVICE_VERSION);
    if (!PSA_HANDLE_IS_VALID(handle)) {
        return PSA_HANDLE_TO_ERROR(handle);
    }

    status = psa_call(handle, PSA_IPC_CALL, in_vec, 1, NULL, 0);

    psa_close(handle);

    return status;
} */

psa_status_t psa_example_hash(const uint8_t *input,
                             size_t input_length,
				             uint8_t *hash,
                             size_t hash_buf_size,
				             size_t *hash_len)
{
    psa_status_t status;
    psa_handle_t handle;
    psa_invec in_vec[] = {
        { .base = input, .len = input_length },
    };

    psa_outvec out_vec[] = {
        { .base = hash, .len = hash_buf_size },
        { .base = hash_len, .len = sizeof(size_t) },
    };

    handle = psa_connect(TFM_EXAMPLE_HASH_SID, TFM_EXAMPLE_HASH_VERSION);
    if (!PSA_HANDLE_IS_VALID(handle)) {
        return PSA_HANDLE_TO_ERROR(handle);
    }

    status = psa_call(handle, PSA_IPC_CALL, in_vec, 1, out_vec, 2);

    psa_close(handle);

    return status;
}

psa_status_t example_read_lsm303(uint8_t *data,
                                size_t data_size)
{
    psa_status_t status;
    psa_handle_t handle;

    psa_outvec out_vec[] = {
        { .base = data, .len = data_size },
    };

    handle = psa_connect(TFM_EXAMPLE_READ_LSM303_SID, TFM_EXAMPLE_READ_LSM303_VERSION);
    if (!PSA_HANDLE_IS_VALID(handle)) {
        return PSA_HANDLE_TO_ERROR(handle);
    }

    status = psa_call(handle, PSA_IPC_CALL, NULL, 0, out_vec, 1);

    psa_close(handle);

    return status;
}

psa_status_t psa_example_tflm_hello(const float *input,
                                    size_t input_length,
                                    float *sine_value_buf,
                                    size_t sine_value_buf_len)
{
    psa_status_t status;
    psa_handle_t handle;
    psa_invec in_vec[] = {
        { .base = input, .len = input_length },
    };

    psa_outvec out_vec[] = {
        { .base = sine_value_buf, .len = sine_value_buf_len },
    };

    handle = psa_connect(TFM_EXAMPLE_TFLM_HELLO_SID, TFM_EXAMPLE_TFLM_HELLO_VERSION);
    if (!PSA_HANDLE_IS_VALID(handle)) {
        return PSA_HANDLE_TO_ERROR(handle);
    }

    status = psa_call(handle, PSA_IPC_CALL, in_vec, 1, out_vec, 1);

    psa_close(handle);

    return status;
}
