/*
 * Copyright (c) 2020 Particle Industries, Inc.
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

#pragma once

//-----------------------------------------------------------------------------
// Tracker platform
//-----------------------------------------------------------------------------

#if PLATFORM_ID != PLATFORM_TRACKER
#error "Platform not supported"
#endif

//
// Tracker types
//
#define TRACKER_MODEL_BARE_SOM_DEFAULT        (0xffff)
#define TRACKER_MODEL_BARE_SOM                (0x0000)
#define TRACKER_MODEL_EVAL                    (0x0001)
#define TRACKER_MODEL_TRACKERONE              (0x0002)


//
// Variables that can be passed in through compile flags (not Workbench)
//
#ifndef TRACKER_PRODUCT_ID
#define TRACKER_PRODUCT_ID                    (PLATFORM_ID)
#endif

#ifndef TRACKER_PRODUCT_VERSION
#define TRACKER_PRODUCT_VERSION               (17)
#endif


//
// Pin and interface mapping
//
#define BMI160_SPI_INTERFACE                  (SPI1)
#define BMI160_SPI_CS_PIN                     (SEN_CS)
#define BMI160_INT_PIN                        (SEN_INT)
#define BMI160_INT_MODE                       (FALLING)

#define UBLOX_SPI_INTERFACE                   (SPI1)
#define UBLOX_CS_PIN                          (GPS_CS)
#define UBLOX_PWR_EN_PIN                      (GPS_PWR)
#define UBLOX_RESETN_PIN                      (GPS_RST)
#define UBLOX_TX_READY_MCU_PIN                (GPS_INT)
#define UBLOX_TX_READY_GPS_PIN                (14) // PIO 14 is EXTINT on GPS Module

#define ESP32_SPI_INTERFACE                   (SPI1)
#define ESP32_CS_PIN                          (WIFI_CS)
#define ESP32_BOOT_MODE_PIN                   (WIFI_BOOT)
#define ESP32_PWR_EN_PIN                      (WIFI_EN)
#define ESP32_INT_PIN                         (WIFI_INT)

#define MCP_CAN_SPI_INTERFACE                 (SPI1)
#define MCP_CAN_CS_PIN                        (CAN_CS)
#define MCP_CAN_STBY_PIN                      (CAN_STBY)
#define MCP_CAN_PWR_EN_PIN                    (CAN_PWR)
#define MCP_CAN_RESETN_PIN                    (CAN_RST)
#define MCP_CAN_INT_PIN                       (CAN_INT)

#define RTC_AM1805_I2C_INSTANCE               (Wire1)
#define RTC_AM1805_I2C_ADDR                   (HAL_PLATFORM_EXTERNAL_RTC_I2C_ADDR)

#define TRACKER_THERMISTOR                    (A0)
#define TRACKER_USER_BUTTON                   (D1)
#define TRACKER_GNSS_LOCK_LED                 (D2)

//#define RTC_WDT_DISABLE
