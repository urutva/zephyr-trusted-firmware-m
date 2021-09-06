/*
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __CMSIS_DRIVER_CONFIG_H__
#define __CMSIS_DRIVER_CONFIG_H__

#include "platform_retarget_dev.h"
#include "RTE_Device.h"
#include "target_cfg.h"

#ifdef RTE_USART0
#define UART0_DEV       ARM_UART0_DEV_NS
#endif

#ifdef RTE_USART1
#ifdef SECURE_UART1
#define UART1_DEV       ARM_UART1_DEV_S
#else
#define UART1_DEV       ARM_UART1_DEV_NS
#endif
#endif

#define SECURE_I2C0

#ifdef RTE_I2C0
#ifdef SECURE_I2C0
#define I2C0_DEV       I2C0_SBCON_DEV_S
#else
#define I2C0_DEV       I2C0_SBCON_DEV_NS
#endif
#endif

#define SECURE_I2C1

#ifdef RTE_I2C1
#ifdef SECURE_I2C1
#define I2C1_DEV       I2C1_SBCON_DEV_S
#else
#define I2C1_DEV       I2C1_SBCON_DEV_NS
#endif
#endif

#define SECURE_I2C2

#ifdef RTE_I2C2
#ifdef SECURE_I2C2
#define I2C2_DEV       I2C2_SBCON_DEV_S
#else
#define I2C2_DEV       I2C2_SBCON_DEV_NS
#endif
#endif

#define SECURE_I2C3

#ifdef RTE_I2C3
#ifdef SECURE_I2C3
#define I2C3_DEV       I2C3_SBCON_DEV_S
#else
#define I2C3_DEV       I2C3_SBCON_DEV_NS
#endif
#endif

#endif  /* __CMSIS_DRIVER_CONFIG_H__ */
