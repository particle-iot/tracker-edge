/*
 * Copyright (c) 2022 Particle Industries, Inc.
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

#include "Particle.h"

namespace {

// General constants and defaults
const float ACCEL_FULL_RANGE = 32768.0;
const uint8_t INVALID_I2C_ADDRESS = 0x7F;
const int BMI270_ACCEL_RANGE_DEFAULT = 2; // g
const float BMI270_ACCEL_RATE_DEFAULT = 100.0; // Hertz
const unsigned int BMI270_I2C_IDLE_TIME = 400; // microseconds, idle time between I2C write accesses
const unsigned int BMI270_SPI_IDLE_TIME = 470; // microseconds, idle time between SPI write accesses
const unsigned long BMI270_SPI_SELECT_TIME = 10; // milliseconds, time to wait for I2C to SPI selection
const unsigned long BMI270_ACC_PMU_CMD_TIME = 4; // milliseconds, PMU mode of accelerometer to normal or low power
const unsigned long BMI270_GYRO_PMU_CMD_TIME = 80; // milliseconds, PMU mode of gyroscope to normal or fast start-up
const unsigned long BMI270_SOFT_RESET_CMD_TIME = 1; // milliseconds, soft reset time
const unsigned int BMI270_INT_LATCH_CLEAR_TIME = 400; // microseconds, idle time between I2C write accesses

// Resister addresses
enum Bmi270Register: uint8_t {
    CHIPID_ADDR             = 0x00,
    ERR_REG_ADDR            = 0x02,
    PMU_STATUS_ADDR         = 0x03,
    MAG_DATA_START_ADDR     = 0x04,
    RHALL_DATA_START_ADDR   = 0x0a,
    GYRO_DATA_START_ADDR    = 0x0c,
    ACCEL_DATA_START_ADDR   = 0x12,
    STATUS_ADDR             = 0x1b,
    INT_STATUS_0_ADDR       = 0x1c,
    INT_STATUS_1_ADDR       = 0x1d,
    INT_STATUS_2_ADDR       = 0x1e,
    INT_STATUS_3_ADDR       = 0x1f,
    ACC_CONF_ADDR           = 0x40,
    ACC_RANGE_ADDR          = 0x41,
    INT_EN_0_ADDR           = 0x50,
    INT_EN_1_ADDR           = 0x51,
    INT_EN_2_ADDR           = 0x52,
    INT_OUT_CTRL_ADDR       = 0x53,
    INT_LATCH_ADDR          = 0x54,
    INT_MAP_0_ADDR          = 0x55,
    INT_MAP_1_ADDR          = 0x56,
    INT_MAP_2_ADDR          = 0x57,
    INT_DATA_0_ADDR         = 0x58,
    INT_DATA_1_ADDR         = 0x59,
    INT_LOWHIGH_0_ADDR      = 0x5a,
    INT_LOWHIGH_1_ADDR      = 0x5b,
    INT_LOWHIGH_2_ADDR      = 0x5c,
    INT_LOWHIGH_3_ADDR      = 0x5d,
    INT_LOWHIGH_4_ADDR      = 0x5e,
    INT_MOTION_0_ADDR       = 0x5f,
    INT_MOTION_1_ADDR       = 0x60,
    INT_MOTION_2_ADDR       = 0x61,
    INT_MOTION_3_ADDR       = 0x62,
    SELF_TEST_ADDR          = 0x6d,
    CMD_ADDR                = 0x7e,
    SPI_MODE_ADDR           = 0x7f,
};


// PMU_STATUS register
#define PMU_STATUS_ACC_SHIFT            (4)
#define PMU_STATUS_ACC_MASK             (0x3 << (PMU_STATUS_ACC_SHIFT))

#define PMU_STATUS_GYRO_SHIFT           (2)
#define PMU_STATUS_GYRO_MASK            (0x3 << (PMU_STATUS_GYRO_SHIFT))

#define PMU_STATUS_MAG_SHIFT            (0)
#define PMU_STATUS_MAG_MASK             (0x3 << (PMU_STATUS_MAG_SHIFT))


// ACC_CONF and ACC_RANGE registers
enum Bmi270AccelUnderSample: uint8_t {
    ACCEL_USAMPLE_OFF       = 0x00,
    ACCEL_USAMPLE_ON        = (1 << 7),
};

#define ACC_CONF_USAMPLE_SHIFT          (7)
#define ACC_CONF_USAMPLE_MASK           (0x1 << (ACC_CONF_USAMPLE_SHIFT))

#define ACC_CONF_BWP_SHIFT              (4)
#define ACC_CONF_BWP_MASK               (0x7 << (ACC_CONF_BWP_SHIFT))

const int ACCEL_CONF_BWP_NORMAL = 2;

#define ACC_CONF_ODR_SHIFT              (0)
#define ACC_CONF_ODR_MASK               (0xf << (ACC_CONF_ODR_SHIFT))

const float ACCEL_RATE_MAX = 1600.0f;
const float ACCEL_RATE_ODR_PERCENT = 100.0f;
const int ACCEL_RATE_ODR_BIT_MIRROR = 8;

enum Bmi270AccelRange: uint8_t {
    ACCEL_RANGE_2G          = BMI2_ACC_RANGE_2G,
    ACCEL_RANGE_4G          = BMI2_ACC_RANGE_4G,
    ACCEL_RANGE_8G          = BMI2_ACC_RANGE_8G,
    ACCEL_RANGE_16G         = BMI2_ACC_RANGE_16G,
};

const float ACCEL_RANGE_2G_F = 2.0f;
const float ACCEL_RANGE_4G_F = 4.0f;
const float ACCEL_RANGE_8G_F = 8.0f;
const float ACCEL_RANGE_16G_F = 16.0f;

#define ACC_RANGE_VAL_SHIFT             (0)
#define ACC_RANGE_VAL_MASK              (0xf << (ACC_RANGE_VAL_SHIFT))


// INT_EN_0 through INT_EN_2 registers
#define INT_EN_0_FLAT_SHIFT             (7)
#define INT_EN_0_FLAT_MASK              (0x1 << (INT_EN_0_FLAT_SHIFT))

#define INT_EN_0_ORIENT_SHIFT           (6)
#define INT_EN_0_ORIENT_MASK            (0x1 << (INT_EN_0_ORIENT_SHIFT))

#define INT_EN_0_S_TAP_SHIFT            (5)
#define INT_EN_0_S_TAP_MASK             (0x1 << (INT_EN_0_S_TAP_SHIFT))

#define INT_EN_0_D_TAP_SHIFT            (4)
#define INT_EN_0_D_TAP_MASK             (0x1 << (INT_EN_0_D_TAP_SHIFT))

#define INT_EN_0_ANYMO_Z_SHIFT          (2)
#define INT_EN_0_ANYMO_Z_MASK           (0x1 << (INT_EN_0_ANYMO_Z_SHIFT))

#define INT_EN_0_ANYMO_Y_SHIFT          (1)
#define INT_EN_0_ANYMO_Y_MASK           (0x1 << (INT_EN_0_ANYMO_Y_SHIFT))

#define INT_EN_0_ANYMO_X_SHIFT          (0)
#define INT_EN_0_ANYMO_X_MASK           (0x1 << (INT_EN_0_ANYMO_X_SHIFT))

#define INT_EN_1_FIFO_W_SHIFT           (6)
#define INT_EN_1_FIFO_W_MASK            (0x1 << (INT_EN_1_FIFO_W_SHIFT))

#define INT_EN_1_FIFO_F_SHIFT           (5)
#define INT_EN_1_FIFO_F_MASK            (0x1 << (INT_EN_1_FIFO_F_SHIFT))

#define INT_EN_1_DATA_SHIFT             (4)
#define INT_EN_1_DATA_MASK              (0x1 << (INT_EN_1_DATA_SHIFT))

#define INT_EN_1_LOW_G_SHIFT            (3)
#define INT_EN_1_LOW_G_MASK             (0x1 << (INT_EN_1_LOW_G_SHIFT))

#define INT_EN_1_HIGH_G_Z_SHIFT         (2)
#define INT_EN_1_HIGH_G_Z_MASK          (0x1 << (INT_EN_1_HIGH_G_Z_SHIFT))

#define INT_EN_1_HIGH_G_Y_SHIFT         (1)
#define INT_EN_1_HIGH_G_Y_MASK          (0x1 << (INT_EN_1_HIGH_G_Y_SHIFT))

#define INT_EN_1_HIGH_G_X_SHIFT         (0)
#define INT_EN_1_HIGH_G_X_MASK          (0x1 << (INT_EN_1_HIGH_G_X_SHIFT))

#define INT_EN_2_STEP_SHIFT             (3)
#define INT_EN_2_STEP_MASK              (0x1 << (INT_EN_2_STEP_SHIFT))

#define INT_EN_2_NOMO_Z_SHIFT           (2)
#define INT_EN_2_NOMO_Z_MASK            (0x1 << (INT_EN_2_NOMO_Z_SHIFT))

#define INT_EN_2_NOMO_Y_SHIFT           (1)
#define INT_EN_2_NOMO_Y_MASK            (0x1 << (INT_EN_2_NOMO_Y_SHIFT))

#define INT_EN_2_NOMO_X_SHIFT           (0)
#define INT_EN_2_NOMO_X_MASK            (0x1 << (INT_EN_2_NOMO_X_SHIFT))


// INT_OUT_CTRL register
#define INT_OUT_CTRL_INT2_OE_SHIFT      (7)
#define INT_OUT_CTRL_INT2_OE_MASK       (0x1 << (INT_OUT_CTRL_INT2_OE_SHIFT))

#define INT_OUT_CTRL_INT2_OD_SHIFT      (6)
#define INT_OUT_CTRL_INT2_OD_MASK       (0x1 << (INT_OUT_CTRL_INT2_OD_SHIFT))

#define INT_OUT_CTRL_INT2_LVL_SHIFT     (5)
#define INT_OUT_CTRL_INT2_LVL_MASK      (0x1 << (INT_OUT_CTRL_INT2_LVL_SHIFT))

#define INT_OUT_CTRL_INT2_EDGE_SHIFT    (4)
#define INT_OUT_CTRL_INT2_EDGE_MASK     (0x1 << (INT_OUT_CTRL_INT2_EDGE_SHIFT))

#define INT_OUT_CTRL_INT1_OE_SHIFT      (3)
#define INT_OUT_CTRL_INT1_OE_MASK       (0x1 << (INT_OUT_CTRL_INT1_OE_SHIFT))

#define INT_OUT_CTRL_INT1_OD_SHIFT      (2)
#define INT_OUT_CTRL_INT1_OD_MASK       (0x1 << (INT_OUT_CTRL_INT1_OD_SHIFT))

#define INT_OUT_CTRL_INT1_LVL_SHIFT     (1)
#define INT_OUT_CTRL_INT1_LVL_MASK      (0x1 << (INT_OUT_CTRL_INT1_LVL_SHIFT))

#define INT_OUT_CTRL_INT1_EDGE_SHIFT    (0)
#define INT_OUT_CTRL_INT1_EDGE_MASK     (0x1 << (INT_OUT_CTRL_INT1_EDGE_SHIFT))

enum Bmi270InterruptPinDrive: uint8_t {
    IRQ_DRIVE_PUSH_PULL                 = 0x0,
    IRQ_DRIVE_OPEN_DRAIN                = 0x1,
};

enum Bmi270InterruptPinLevel: uint8_t {
    IRQ_LEVEL_ACTIVE_LOW                = 0x0,
    IRQ_LEVEL_ACTIVE_HIGH               = 0x1,
};

enum Bmi270InterruptPinEdge: uint8_t {
    IRQ_EDGE_LEVEL                      = 0x0,
    IRQ_EDGE_EDGE                       = 0x1,
};


// INT_LATCH register
#define INT_LATCH_INT2_INPUT_EN_SHIFT   (5)
#define INT_LATCH_INT2_INPUT_EN_MASK    (0x1 << (INT_LATCH_INT2_INPUT_EN_SHIFT))

#define INT_LATCH_INT1_INPUT_EN_SHIFT   (4)
#define INT_LATCH_INT1_INPUT_EN_MASK    (0x1 << (INT_LATCH_INT1_INPUT_EN_SHIFT))

#define INT_LATCH_MODE_SHIFT            (0)
#define INT_LATCH_MODE_MASK             (0xf << (INT_LATCH_MODE_SHIFT))

enum Bmi270InterruptLatchMode: uint8_t {
    IRQ_LATCH_UNLATCHED                 = 0x0,
    IRQ_LATCH_312_5_US                  = 0x1,
    IRQ_LATCH_625_US                    = 0x2,
    IRQ_LATCH_1_25_MS                   = 0x3,
    IRQ_LATCH_2_5_MS                    = 0x4,
    IRQ_LATCH_5_MS                      = 0x5,
    IRQ_LATCH_10_MS                     = 0x6,
    IRQ_LATCH_20_MS                     = 0x7,
    IRQ_LATCH_40_MS                     = 0x8,
    IRQ_LATCH_80_MS                     = 0x9,
    IRQ_LATCH_160_MS                    = 0xa,
    IRQ_LATCH_320_MS                    = 0xb,
    IRQ_LATCH_640_MS                    = 0xc,
    IRQ_LATCH_1_28_S                    = 0xd,
    IRQ_LATCH_2_56_S                    = 0xe,
    IRQ_LATCH_LATCHED                   = 0xf,
};


// INT_MAP_0 through INT_MAP_2 registers
#define INT_MAP_0_INT1_FLAT_SHIFT       (7)
#define INT_MAP_0_INT1_FLAT_MASK        (0x1 << (INT_MAP_0_INT1_FLAT_SHIFT))

#define INT_MAP_0_INT1_ORIENT_SHIFT     (6)
#define INT_MAP_0_INT1_ORIENT_MASK      (0x1 << (INT_MAP_0_INT1_ORIENT_SHIFT))

#define INT_MAP_0_INT1_S_TAP_SHIFT      (5)
#define INT_MAP_0_INT1_S_TAP_MASK       (0x1 << (INT_MAP_0_INT1_S_TAP_SHIFT))

#define INT_MAP_0_INT1_D_TAP_SHIFT      (4)
#define INT_MAP_0_INT1_D_TAP_MASK       (0x1 << (INT_MAP_0_INT1_D_TAP_SHIFT))

#define INT_MAP_0_INT1_NO_MOT_SHIFT     (3)
#define INT_MAP_0_INT1_NO_MOT_MASK      (0x1 << (INT_MAP_0_INT1_NO_MOT_SHIFT))

#define INT_MAP_0_INT1_ANYS_MOT_SHIFT   (2)
#define INT_MAP_0_INT1_ANYS_MOT_MASK    (0x1 << (INT_MAP_0_INT1_ANYS_MOT_SHIFT))

#define INT_MAP_0_INT1_HIGH_G_SHIFT     (1)
#define INT_MAP_0_INT1_HIGH_G_MASK      (0x1 << (INT_MAP_0_INT1_HIGH_G_SHIFT))

#define INT_MAP_0_INT1_LOW_G_SHIFT      (0)
#define INT_MAP_0_INT1_LOW_G_MASK       (0x1 << (INT_MAP_0_INT1_LOW_G_SHIFT))

#define INT_MAP_1_INT1_DATA_SHIFT       (7)
#define INT_MAP_1_INT1_DATA_MASK        (0x1 << (INT_MAP_1_INT1_DATA_SHIFT))

#define INT_MAP_1_INT1_FIFO_W_SHIFT     (6)
#define INT_MAP_1_INT1_FIFO_W_MASK      (0x1 << (INT_MAP_1_INT1_FIFO_W_SHIFT))

#define INT_MAP_1_INT1_FIFO_F_SHIFT     (5)
#define INT_MAP_1_INT1_FIFO_F_MASK      (0x1 << (INT_MAP_1_INT1_FIFO_F_SHIFT))

#define INT_MAP_1_INT1_PMU_SHIFT        (4)
#define INT_MAP_1_INT1_PMU_MASK         (0x1 << (INT_MAP_1_INT1_PMU_SHIFT))

#define INT_MAP_1_INT2_DATA_SHIFT       (3)
#define INT_MAP_1_INT2_DATA_MASK        (0x1 << (INT_MAP_1_INT2_DATA_SHIFT))

#define INT_MAP_1_INT2_FIFO_W_SHIFT     (2)
#define INT_MAP_1_INT2_FIFO_W_MASK      (0x1 << (INT_MAP_1_INT2_FIFO_W_SHIFT))

#define INT_MAP_1_INT2_FIFO_F_SHIFT     (1)
#define INT_MAP_1_INT2_FIFO_F_MASK      (0x1 << (INT_MAP_1_INT2_FIFO_F_SHIFT))

#define INT_MAP_1_INT2_PMU_SHIFT        (0)
#define INT_MAP_1_INT2_PMU_MASK         (0x1 << (INT_MAP_1_INT2_PMU_SHIFT))

#define INT_MAP_2_INT2_FLAT_SHIFT       (7)
#define INT_MAP_2_INT2_FLAT_MASK        (0x1 << (INT_MAP_2_INT2_FLAT_SHIFT))

#define INT_MAP_2_INT2_ORIENT_SHIFT     (6)
#define INT_MAP_2_INT2_ORIENT_MASK      (0x1 << (INT_MAP_2_INT2_ORIENT_SHIFT))

#define INT_MAP_2_INT2_S_TAP_SHIFT      (5)
#define INT_MAP_2_INT2_S_TAP_MASK       (0x1 << (INT_MAP_2_INT2_S_TAP_SHIFT))

#define INT_MAP_2_INT2_D_TAP_SHIFT      (4)
#define INT_MAP_2_INT2_D_TAP_MASK       (0x1 << (INT_MAP_2_INT2_D_TAP_SHIFT))

#define INT_MAP_2_INT2_NO_MOT_SHIFT     (3)
#define INT_MAP_2_INT2_NO_MOT_MASK      (0x1 << (INT_MAP_2_INT2_NO_MOT_SHIFT))

#define INT_MAP_2_INT2_ANYS_MOT_SHIFT   (2)
#define INT_MAP_2_INT2_ANYS_MOT_MASK    (0x1 << (INT_MAP_2_INT2_ANYS_MOT_SHIFT))

#define INT_MAP_2_INT2_HIGH_G_SHIFT     (1)
#define INT_MAP_2_INT2_HIGH_G_MASK      (0x1 << (INT_MAP_2_INT2_HIGH_G_SHIFT))

#define INT_MAP_2_INT2_LOW_G_SHIFT      (0)
#define INT_MAP_2_INT2_LOW_G_MASK       (0x1 << (INT_MAP_2_INT2_LOW_G_SHIFT))


// INT_LOWHIGH_0 through INT_LOWHIGH_4 registers
#define INTLH_0_LOW_DUR_MASK            (0xff)

#define INTLH_1_LOW_TH_MASK             (0xff)

#define INTLH_2_HIGH_HYST_SHIFT         (5)
#define INTLH_2_HIGH_HYST_MASK          (0x3 << (INTLH_2_HIGH_HYST_SHIFT))

#define INTLH_2_LOW_HYST_SHIFT          (0)
#define INTLH_2_LOW_HYST_MASK           (0x3 << (INTLH_2_LOW_HYST_SHIFT))

const uint8_t INTLH_2_HIGH_HYST_MAX = 0x3;
const float INTLH_2_HIGH_HYST_RES = 16.0f;

#define INTLH_3_HIGH_DUR_MASK           (0xff)

const float INTLH_3_HIGH_DUR_MIN = 0.0025;
const float INTLH_3_HIGH_DUR_MAX = 0.640;

#define INTLH_4_HIGH_TH_MASK            (0xff)

const uint8_t INTLH_4_HIGH_TH_MAX = INTLH_4_HIGH_TH_MASK;
const float INTLH_4_HIGH_TH_LOWEST_RES = 256.0f;
const float INTLH_4_HIGH_TH_RES = 512.0f;


// INT_MOTION_0 through INT_MOTION_3 registers
#define INTMO_0_SLO_NO_DUR_SHIFT        (2)
#define INTMO_0_SLO_NO_DUR_MASK         (0x3f << (INTMO_0_SLO_NO_DUR_SHIFT))

#define INTMO_0_ANYM_DUR_SHIFT          (0)
#define INTMO_0_ANYM_DUR_MASK           (0x3 << (INTMO_0_ANYM_DUR_SHIFT))

const unsigned INTMO_0_ANYM_DUR_MIN = 1;
const unsigned INTMO_0_ANYM_DUR_MAX = 4;

#define INTMO_1_ANYM_TH_MASK            (0xff)

const uint8_t INTMO_1_ANYM_TH_MAX = INTMO_1_ANYM_TH_MASK;
const float INTMO_1_ANYM_TH_LOWEST_RES = 512.0f;
const float INTMO_1_ANYM_TH_RES = 1024.0f;

#define INTMO_3_SIG_MOT_PROOF_SHIFT     (4)
#define INTMO_3_SIG_MOT_PROOF_MASK      (0x3 << (INTMO_3_SIG_MOT_PROOF_SHIFT))

#define INTMO_3_SIG_MOT_SKIP_SHIFT      (2)
#define INTMO_3_SIG_MOT_SKIP_MASK       (0x3 << (INTMO_3_SIG_MOT_SKIP_SHIFT))

#define INTMO_3_SIG_MOT_SEL_SHIFT       (1)
#define INTMO_3_SIG_MOT_SEL_MASK        (0x1 << (INTMO_3_SIG_MOT_SEL_SHIFT))

#define INTMO_3_NO_MOT_SEL_SHIFT        (0)
#define INTMO_3_NO_MOT_SEL_MASK         (0x1 << (INTMO_3_NO_MOT_SEL_SHIFT))


// CMD_ADDR register
enum Bmi270Command: uint8_t {
    CMD_ACC_PMU_MODE_SUSPEND    = 0x10,
    CMD_ACC_PMU_MODE_NORMAL     = 0x11,
    CMD_ACC_PMU_MODE_LOW        = 0x12,
    CMD_INT_RESET               = 0xb1,
    CMD_SOFT_RESET              = 0xb6,
};


} // anonymous namespace