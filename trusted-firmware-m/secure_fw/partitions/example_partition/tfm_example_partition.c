/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdint.h>

#include "psa/service.h"
#include "psa/crypto.h"
#include "tfm_crypto_defs.h"
#include "psa_manifest/tfm_example_partition.h"
#include "log/tfm_log.h"
#include "tfm/tfm_spm_services.h"
#include "tfm_plat_test.h"

/**
 * \brief An example service implementation that prints out an argument from the
 *        client and then starts a timer.
 */
static void tfm_example_service(void)
{
    psa_status_t status;
    uint32_t arg;
    psa_msg_t msg;

    /* Retrieve the message corresponding to the example service signal */
    status = psa_get(TFM_EXAMPLE_SERVICE_SIGNAL, &msg);
    if (status != PSA_SUCCESS) {
        return;
    }

    /* Decode the message */
    switch (msg.type) {
    case PSA_IPC_CONNECT:
    case PSA_IPC_DISCONNECT:
        /* This service does not require any setup or teardown on connect or
         * disconnect, so just reply with success.
         */
        status = PSA_SUCCESS;
        break;
    case PSA_IPC_CALL:
        if (msg.in_size[0] != sizeof(arg)) {
            status = PSA_ERROR_PROGRAMMER_ERROR;
            break;
        }

        /* Print arg from client */
        psa_read(msg.handle, 0, &arg, sizeof(arg));
        // LOG_MSG("[Example partition] Service called! arg=%p\r\n", arg);

        /* Start timer. The interrupt triggered when it expires will be handled
         * by tfm_example_timer_handler().
         */
        // tfm_plat_test_secure_timer_start();
        // LOG_MSG("[Example partition] Timer started...\r\n");

        status = PSA_SUCCESS;
        break;
    default:
        /* Invalid message type */
        status = PSA_ERROR_PROGRAMMER_ERROR;
        break;
    }

    /* Reply with the message result status to unblock the client */
    psa_reply(msg.handle, status);
}

/**
 * \brief An example service implementation that calculates hash of a given
 *        message.
 */

#define SHA256_BLOCK_SIZE 64

static void tfm_example_hash(void)
{
    psa_status_t status;
    uint8_t input[SHA256_BLOCK_SIZE];  //Block size of SHA-256
    size_t input_len;
    size_t hash_len;
    psa_hash_operation_t hash_operation;
    psa_msg_t msg;

    /* Retrieve the message corresponding to the example service signal */
    status = psa_get(TFM_EXAMPLE_HASH_SIGNAL, &msg);
    if (status != PSA_SUCCESS) {
        return;
    }

    input_len = msg.in_size[0];

    /* Decode the message */
    switch (msg.type) {
    case PSA_IPC_CONNECT:
    case PSA_IPC_DISCONNECT:
        /* This service does not require any setup or teardown on connect or
         * disconnect, so just reply with success.
         */
        status = PSA_SUCCESS;
        break;
    case PSA_IPC_CALL:
        // LOG_MSG("[Example partition] Calculating SHA-256! \r\n");
        hash_operation = psa_hash_operation_init();

        struct tfm_crypto_pack_iovec iov = {
            .sfn_id = TFM_CRYPTO_HASH_SETUP_SID,
            .alg = PSA_ALG_SHA_256,
            .op_handle = hash_operation.handle,
        };

        psa_invec in_vec[] = {
            {.base = &iov, .len = sizeof(struct tfm_crypto_pack_iovec)},
        };
        psa_outvec out_vec[] = {
            {.base = &(hash_operation.handle), .len = sizeof(uint32_t)},
        };

        // psa_hash_setup(&hash_operation, PSA_ALG_SHA_256);
        tfm_crypto_hash_setup(in_vec, 1, out_vec, 1);

        // LOG_MSG("[Example partition] completed tfm_crypto_hash_setup! \r\n");

        /* Calculate SHA-256 hash of input */
        while(input_len > SHA256_BLOCK_SIZE) {
            psa_read(msg.handle, 0, &input, SHA256_BLOCK_SIZE);

            struct tfm_crypto_pack_iovec iov = {
                .sfn_id = TFM_CRYPTO_HASH_UPDATE_SID,
                .op_handle = hash_operation.handle,
            };

            psa_invec in_vec[] = {
                {.base = &iov, .len = sizeof(struct tfm_crypto_pack_iovec)},
                {.base = input, .len = SHA256_BLOCK_SIZE},
            };

            psa_outvec out_vec[] = {
                {.base = &(hash_operation.handle), .len = sizeof(uint32_t)},
            };

            // psa_read(msg.handle, 0, &input, SHA256_BLOCK_SIZE);
            // LOG_MSG("[Example partition] calling tfm_crypto_hash_update 1! \r\n");
            tfm_crypto_hash_update(in_vec, 2, out_vec, 1);
            // psa_hash_update(&hash_operation, input, SHA256_BLOCK_SIZE);
            input_len -= SHA256_BLOCK_SIZE;
        }

        if (input_len > 0) {
            psa_read(msg.handle, 0, input, input_len);

            struct tfm_crypto_pack_iovec iov = {
                .sfn_id = TFM_CRYPTO_HASH_UPDATE_SID,
                .op_handle = hash_operation.handle,
            };

            psa_invec in_vec[] = {
                {.base = &iov, .len = sizeof(struct tfm_crypto_pack_iovec)},
                {.base = input, .len = input_len},
            };

            psa_outvec out_vec[] = {
                {.base = &(hash_operation.handle), .len = sizeof(uint32_t)},
            };

            // psa_read(msg.handle, 0, &input, input_len);
            // psa_hash_update(&hash_operation, input, input_len);
            // LOG_MSG("[Example partition] calling tfm_crypto_hash_update 2! \r\n");
            tfm_crypto_hash_update(in_vec, 2, out_vec, 1);
        }

        struct tfm_crypto_pack_iovec iov1 = {
            .sfn_id = TFM_CRYPTO_HASH_FINISH_SID,
            .op_handle = hash_operation.handle,
        };

        psa_invec in_vec1[] = {
            {.base = &iov1, .len = sizeof(struct tfm_crypto_pack_iovec)},
        };
        psa_outvec out_vec1[] = {
            {.base = &(hash_operation.handle), .len = sizeof(uint32_t)},
            {.base = input, .len = SHA256_BLOCK_SIZE},
        };

        // LOG_MSG("[Example partition] calling tfm_crypto_hash_finish! \r\n");

        tfm_crypto_hash_finish(in_vec1, 1, out_vec1, 2);

        // psa_hash_finish(&hash_operation,
		// 			    input, SHA256_BLOCK_SIZE, &hash_len);

        // LOG_MSG("[Example partition] hash_len: %d! \r\n", out_vec1[1].len);
        psa_write(msg.handle, 0, input, out_vec1[1].len);
        psa_write(msg.handle, 1, &out_vec1[1].len, sizeof(out_vec1[1].len));

        // LOG_MSG("[Example partition] Completed SHA-256 calculation! \r\n");

        status = PSA_SUCCESS;
        break;
    default:
        /* Invalid message type */
        status = PSA_ERROR_PROGRAMMER_ERROR;
        break;
    }

    /* Reply with the message result status to unblock the client */
    psa_reply(msg.handle, status);
}

/**
 * \brief An example interrupt handler.
 */
static void tfm_example_timer_handler(void)
{
    /* Stop timer */
    // LOG_MSG("[Example partition] Timer interrupt handler...\r\n");
    tfm_plat_test_secure_timer_stop();
    // LOG_MSG("[Example partition] Timer stopped...\r\n");
    /* Inform the SPM that the timer interrupt has been handled */
    psa_eoi(TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ);
}

/**
 * \brief The example partition's entry function.
 */
void tfm_example_partition_main(void)
{
    psa_signal_t signals;

    /* Enable timer IRQ */
#ifndef TFM_PSA_API
    tfm_enable_irq(TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ);
#endif

    /* Continually wait for one or more of the partition's RoT Service or
     * interrupt signals to be asserted and then handle the asserted signal(s).
     */
    while (1) {
        signals = psa_wait(PSA_WAIT_ANY, PSA_BLOCK);

        if (signals & TFM_EXAMPLE_SERVICE_SIGNAL) {
            tfm_example_service();
        }
        if (signals & TFM_EXAMPLE_HASH_SIGNAL) {
            tfm_example_hash();
        }
        if (signals & TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ) {
            tfm_example_timer_handler();
        }
    }
}
