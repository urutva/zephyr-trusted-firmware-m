/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdint.h>

#include "psa/service.h"
#include "psa/crypto.h"
#include "psa_manifest/tfm_example_partition.h"
#include "tfm_sp_log.h"
#include "tfm/tfm_spm_services.h"
#include "tfm_plat_test.h"
#include "target_cfg.h"

#include "Driver_I2C.h"
#include "platform_regs.h"

#include "main_functions.h"

/* I2C driver name for LSM303 peripheral */
extern ARM_DRIVER_I2C LSM303_DRIVER;

/* I2C address of LSM303 peripheral */
extern const uint8_t lsm303_addr;

// This constant represents the range of x values our model was trained on,
// which is from 0 to (2 * Pi). We approximate Pi to avoid requiring additional
// libraries.
extern const float kXrange;

/**
 * \brief An example service implementation that calculates SHA-256 hash of a
 *  given message.
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
        if (msg.out_size[0] != PSA_HASH_SIZE(PSA_ALG_SHA_256) ||
            msg.out_size[1] != sizeof(size_t)) {
            status = PSA_ERROR_PROGRAMMER_ERROR;
            break;
        }

        LOG_INFFMT("[Example partition] Calculating SHA-256 hash! \r\n");
        hash_operation = psa_hash_operation_init();
        psa_hash_setup(&hash_operation, PSA_ALG_SHA_256);

        /* Calculate SHA-256 hash of input */
        while(input_len > SHA256_BLOCK_SIZE) {
            psa_read(msg.handle, 0, &input, SHA256_BLOCK_SIZE);
            psa_hash_update(&hash_operation, input, SHA256_BLOCK_SIZE);
            input_len -= SHA256_BLOCK_SIZE;
        }

        if (input_len > 0) {
            psa_read(msg.handle, 0, &input, input_len);
            psa_hash_update(&hash_operation, input, input_len);
        }

        psa_hash_finish(&hash_operation,
					    input, SHA256_BLOCK_SIZE, &hash_len);

        psa_write(msg.handle, 0, input, hash_len);
        psa_write(msg.handle, 1, &hash_len, sizeof(hash_len));

        LOG_INFFMT("[Example partition] Done! \r\n");

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
 * \brief Send data to I2C peripheral
 */

static psa_status_t i2c_send_data(uint8_t* i2c_data,
                                 uint32_t i2c_data_len,
                                 bool xfer_pending)
{
    psa_status_t status = PSA_SUCCESS;

    LSM303_DRIVER.MasterTransmit (lsm303_addr, i2c_data, i2c_data_len, xfer_pending);

    /* Wait until transfer completed */
    while (LSM303_DRIVER.GetStatus().busy);
    /* Check if all data transferred */
    if (LSM303_DRIVER.GetDataCount () != i2c_data_len) {
        LOG_INFFMT("[Example partition] Master transmit data count didn't match...\r\n");
        status = PSA_ERROR_HARDWARE_FAILURE;
    }

    return status;
}

/**
 * \brief Receive data from I2C peripheral
 */

static psa_status_t i2c_receive_data(uint8_t* i2c_data,
                                    uint32_t i2c_data_len,
                                    bool xfer_pending)
{
    psa_status_t status = PSA_SUCCESS;

    LSM303_DRIVER.MasterReceive (lsm303_addr, i2c_data, i2c_data_len, false);

    /* Wait until transfer completed */
    while (LSM303_DRIVER.GetStatus().busy);
    /* Check if all data transferred */
    if (LSM303_DRIVER.GetDataCount () != i2c_data_len) {
        LOG_INFFMT("[Example partition] Master receive data count didn't match...\r\n");
        status = PSA_ERROR_HARDWARE_FAILURE;
    }

    return status;
}

/**
 * \brief Convert LSM303 data
 */
static inline double lsm303_data_to_double(const uint16_t data)
{
	int32_t tmp_data1;
    int32_t tmp_data2;

    tmp_data1 = data / 1100;
	tmp_data2 = (1000000 * data / 1100) % 1000000;
    return (double)tmp_data1 + (double)tmp_data2 / 1000000;
}

/**
 * \brief Read from LSM303 peripheral
 */
static void tfm_example_read_lsm303(uint8_t hw_initialised)
{
    psa_status_t status;
    psa_msg_t msg;

    uint8_t i2c_reg_addr;
    uint8_t i2c_reg_data[6] = {0};
    // double lsm303_data[3] = {0};

    /* Retrieve the message corresponding to the example service signal */
    status = psa_get(TFM_EXAMPLE_READ_LSM303_SIGNAL, &msg);
    if (status != PSA_SUCCESS) {
        return;
    }

    /* Decode the message */
    switch (msg.type) {
    /* Any setup or teardown on IPC connect or disconnect goes here. If
     * nothing then just reply with success.
     */
    case PSA_IPC_CONNECT:
    case PSA_IPC_DISCONNECT:
        /* This service does not require any setup or teardown on connect or
         * disconnect, so just reply with success.
         */
        status = PSA_SUCCESS;
        break;

    case PSA_IPC_CALL:
        // Check size of outvec parameter
        if (msg.out_size[0] != sizeof(i2c_reg_data)) {
            status = PSA_ERROR_PROGRAMMER_ERROR;
            break;
        }

        /* Hardware init failed, return failure status to unblock the client */
        if(!hw_initialised) {
            psa_reply(msg.handle, PSA_ERROR_HARDWARE_FAILURE);
        }

        /* Read from LSM303 peripheral */
        LOG_INFFMT("[Example partition] Start reading LSM303 peripheral...\r\n");

        /* Check data ready */
        i2c_reg_addr = 0x9;
        i2c_send_data(&i2c_reg_addr, sizeof(i2c_reg_addr), true);

        i2c_receive_data(&i2c_reg_data[0], 1, false);
        if(!(i2c_reg_data[0] & 1)) {
            LOG_INFFMT("[Example partition] Sensor data not available...\r\n");
            status = PSA_ERROR_INSUFFICIENT_DATA;
            break;
        }

        /* Read magnetic sensor data */
        i2c_reg_addr = 0x3;
        i2c_send_data(&i2c_reg_addr, sizeof(i2c_reg_addr), true);

        i2c_receive_data(i2c_reg_data, sizeof(i2c_reg_data), false);

        // lsm303_data[0] = lsm303_data_to_double((uint16_t)((i2c_reg_data[0] << 8) | i2c_reg_data[1]));
        // lsm303_data[1] = lsm303_data_to_double((uint16_t)((i2c_reg_data[4] << 8) | i2c_reg_data[5]));
        // lsm303_data[2] = lsm303_data_to_double((uint16_t)((i2c_reg_data[2] << 8) | i2c_reg_data[3]));

        psa_write(msg.handle, 0, i2c_reg_data, sizeof(i2c_reg_data));
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
 * \brief Run inference using Tensorflow lite-micro
 */
void tfm_example_tflm_hello(void)
{
    psa_status_t status;
    psa_msg_t msg;

    float x_value, y_value;

    /* Retrieve the message corresponding to the TFLM hello service signal */
    status = psa_get(TFM_EXAMPLE_TFLM_HELLO_SIGNAL, &msg);
    if (status != PSA_SUCCESS) {
        return;
    }

    /* Decode the message */
    switch (msg.type) {
    /* Any setup or teardown on IPC connect or disconnect goes here. If
     * nothing then just reply with success.
     */
    case PSA_IPC_CONNECT:
    case PSA_IPC_DISCONNECT:
        /* This service does not require any setup or teardown on connect or
         * disconnect, so just reply with success.
         */
        status = PSA_SUCCESS;
        break;

    case PSA_IPC_CALL:
        // Check size of invec/outvec parameter
        if (msg.in_size[0] != sizeof(x_value) ||
            msg.out_size[0] != sizeof(x_value)) {

            status = PSA_ERROR_PROGRAMMER_ERROR;
            break;
        }

        psa_read(msg.handle, 0, &x_value, sizeof(x_value));

        if (kXrange < x_value < 0.0f) {
            status = PSA_ERROR_PROGRAMMER_ERROR;
            break;
        }

        /* Run inference */
        LOG_INFFMT("[Example partition] Starting secure inferencing...\r\n");
        y_value = loop(x_value);

        psa_write(msg.handle, 0, &y_value, sizeof(y_value));
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
 * \brief Templated example service.
 */
// static void tfm_example_service(void)
// {
//     psa_status_t status;
//     psa_msg_t msg;

//     /* Retrieve the message corresponding to the example service signal */
//     status = psa_get(TFM_EXAMPLE_SERVICE_SIGNAL, &msg);
//     if (status != PSA_SUCCESS) {
//         return;
//     }

//     /* Decode the message */
//     switch (msg.type) {
//     /* Any setup or teardown on IPC connect or disconnect goes here. If
//      * nothing then just reply with success.
//      */
//     case PSA_IPC_CONNECT:
//     case PSA_IPC_DISCONNECT:
//         status = PSA_SUCCESS;
//         break;
//
//     /* Service implementation goes here */
//     case PSA_IPC_CALL:
//         /* Start timer. The interrupt triggered when it expires will be handled
//          * by tfm_example_timer_handler().
//          */
//         tfm_plat_test_secure_timer_start();
//         LOG_INFFMT("[Example partition] Timer started...\r\n");

//         status = PSA_SUCCESS;
//         break;
//     default:
//         /* Invalid message type */
//         status = PSA_ERROR_PROGRAMMER_ERROR;
//         break;
//     }

//     /* Reply with the message result status to unblock the client */
//     psa_reply(msg.handle, status);
// }

/**
 * \brief A templated example interrupt handler.
 */
// static void tfm_example_timer_handler(void)
// {
//     /* Stop timer */
//     tfm_plat_test_secure_timer_stop();
//     /* Inform the SPM that the timer interrupt has been handled */
//     psa_eoi(TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ);
// }

/**
 * \brief The example partition's entry function.
 */
void tfm_example_partition_main(void)
{
    psa_signal_t signals;
    uint8_t i2c_reg_data[2] = {0};
    uint8_t lsm303_init_completed = 1;

    /* Enable timer IRQ */
// #ifdef TFM_PSA_API
//     psa_irq_enable(TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ);
// #else
//     tfm_enable_irq(TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ);
// #endif

    /* LSM303 DLHC */
    LOG_INFFMT("[Example partition] Configuring I2C3 as secure peripheral...\r\n");
    ppc_configure_to_secure(PPC_SP_APB_PPC_EXP1, CMSDK_I2C3_APB_PPC_POS);

    LOG_INFFMT("[Example partition] Initialising I2C bus...\r\n");
    LSM303_DRIVER.Initialize(NULL);
    LSM303_DRIVER.PowerControl (ARM_POWER_FULL);
    LSM303_DRIVER.Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
    LSM303_DRIVER.Control      (ARM_I2C_BUS_CLEAR, 0);

    /* Set magnetometer output data rate */
    i2c_reg_data[0] = 0x0;
    i2c_reg_data[1] = 0x0;
    if(i2c_send_data(i2c_reg_data, sizeof(i2c_reg_data), false) != PSA_SUCCESS) {
        lsm303_init_completed = 0;
    }

    /* Set magnetometer full scale range */
    i2c_reg_data[0] = 0x1;
    i2c_reg_data[1] = 1 << 5;
    if(i2c_send_data(i2c_reg_data, sizeof(i2c_reg_data), false) != PSA_SUCCESS) {
        lsm303_init_completed = 0;
    }

    /* Continuous update */
    i2c_reg_data[0] = 0x2;
    i2c_reg_data[1] = 0x0;
    if(i2c_send_data(i2c_reg_data, sizeof(i2c_reg_data), false) != PSA_SUCCESS) {
        lsm303_init_completed = 0;
    }

    LOG_INFFMT("[Example partition] Initialisation of I2C bus completed\r\n");

    // Tensorflow lite-micro initialisation
    setup();

    LOG_INFFMT("[Example partition] TFLM initalisation completed\r\n");

    /* Continually wait for one or more of the partition's RoT Service or
     * interrupt signals to be asserted and then handle the asserted signal(s).
     */
    while (1) {
        signals = psa_wait(PSA_WAIT_ANY, PSA_BLOCK);

        if (signals & TFM_EXAMPLE_HASH_SIGNAL) {
            tfm_example_hash();
        } else if (signals & TFM_EXAMPLE_READ_LSM303_SIGNAL) {
            tfm_example_read_lsm303(lsm303_init_completed);
        } else if (signals & TFM_EXAMPLE_TFLM_HELLO_SIGNAL) {
            tfm_example_tflm_hello();
        }
        // /* Check the signal received from SPM and call the respective
        //  * service.
        //  */
        // if (signals & TFM_EXAMPLE_SERVICE_SIGNAL) {
        //     tfm_example_service();
        // }
        // if (signals & TFM_EXAMPLE_SIGNAL_TIMER_0_IRQ) {
        //     tfm_example_timer_handler();
        // }
    }
}
