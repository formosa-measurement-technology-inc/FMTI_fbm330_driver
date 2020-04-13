/*
 * Copyright (C) 2015 Formosa Measurement Technology Inc. Ltd. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef __FBM330_H__
#define __FBM330_H__

#include <stdio.h>
#include <stdint.h>
#include <NUC123.h> //MCU specific header files

/* Data bus definition */
#define I2C
//#define SPI

#define DEVICE_NAME     "FBM330"
#define FBM330_CHIP_ID  0x42
//#define DEBUG_FBM330  //Enable debug mode

/**
 * { I2C 7bit address setting for fbm330 }
 * 	If SDO pin is pulled low, I2C address is 6C.
 * 	If SDO pin is pulled high, I2C address is 6D.
 */
#define FBM330_I2C_SLAVE_ADDR 0x6C

/* Define the oversampling rate setting of fbm330.
 * Range of setting:
 * {osr_1024, osr_2048, osr_4096, osr_8192, osr_16384}
 */
#define OVERSAMPLING_RATE_DEFAULT  osr_8192

/* Control registers address*/
#define FBM330_SPI_CTRL_REG	0x00
#define FBM330_SPI_CTRL_REG_SDO_ACTIVE_POS	(0)
#define FBM330_SPI_CTRL_REG_SDO_ACTIVE_MSK	(0x81 << FBM330_SPI_CTRL_REG_SDO_ACTIVE_POS)
#define FBM330_SPI_CTRL_REG_SDO_ACTIVE_EN	(0x81 << FBM330_SPI_CTRL_REG_SDO_ACTIVE_POS)
#define FBM330_SPI_CTRL_REG_SDO_ACTIVE_DIS	(0 << << FBM330_SPI_CTRL_REG_SDO_ACTIVE_POS)
#define FBM330_TAKE_MEAS_REG	0xf4
#define FBM330_READ_MEAS_REG_U	0xf6
#define FBM330_READ_MEAS_REG_L	0xf7
#define FBM330_READ_MEAS_REG_XL	0xf8
#define FBM330_SOFTRESET_REG    0xe0
#define FBM330_CHIP_ID_REG	  0x6b
#define FBM330_VERSION_REG	  0xa5
#define FBM330_P_CONFIG_REG	  0xa6
#define FBM330_P_CONFIG_REG_GAIN_POS (3)
#define FBM330_P_CONFIG_REG_GAIN_MAK (7 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X1 (0 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X2 (1 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X4 (2 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X8 (3 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X16 (4 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X32 (5 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X64 (6 << FBM330_P_CONFIG_REG_GAIN_POS)
#define FBM330_P_CONFIG_REG_GAIN_X128 (7 << FBM330_P_CONFIG_REG_GAIN_POS)

/* CMD list */
#define FBM330_MEAS_TEMP		        0x2e /* 2.5ms wait for measurement */
#define FBM330_MEAS_PRESS_OVERSAMP_0	0x34 /* 2.5ms wait for measurement */
#define FBM330_MEAS_PRESS_OVERSAMP_1	0x74 /* 3.7ms wait for measurement */
#define FBM330_MEAS_PRESS_OVERSAMP_2	0xb4 /* 6ms wait for measurement */
#define FBM330_MEAS_PRESS_OVERSAMP_3	0xf4 /* 10.7ms wait for measurement */
#define FBM330_SOFTRESET_CMD            0xb6

#define FBM330_CONVERSION_usTIME_OSR1024   2500  /*us*/
#define FBM330_CONVERSION_usTIME_OSR2048   3700  /*us*/
#define FBM330_CONVERSION_usTIME_OSR4096   6000  /*us*/
#define FBM330_CONVERSION_usTIME_OSR8192   10700 /*us*/
#define FBM330_CONVERSION_usTIME_OSR16384   20500 /*us*/

/* Calibration registers */
#define FBM330_CALIBRATION_DATA_START0	 0xaa /* Calibraton data address
                                      	       * {0xf1, 0xd0, 0xbb:0xaa} */
#define FBM330_CALIBRATION_DATA_START1   0xd0
#define FBM330_CALIBRATION_DATA_START2   0xf1
#define FBM330_CALIBRATION_DATA_LENGTH	 20 /* bytes */

#ifdef SPI
#define FBM330_SPI_WRITE 0x00
#define FBM330_SPI_READ 0x80
#define FBM330_SPI_1BYTE 0x00
#define FBM330_SPI_2BYTE 0x20
#define FBM330_SPI_3BYTE 0x40
#define FBM330_SPI_4BYTE 0x60
#endif

extern volatile uint32_t TMR0_Ticks;
extern volatile uint32_t fbm330_update_rdy;

struct fbm330_calibration_data {
	int32_t C0, C1, C2, C3, C4, C5, C6, C7, \
	C8, C9, C10, C11, C12, C13;
};

enum fbm330_osr {
	osr_1024 = 0x0,
	osr_2048 = 0x1,
	osr_4096 = 0x2,
	osr_8192 = 0x3,
	osr_16384 = 0x4
};

enum fbm330_hw_version {
    hw_ver_a14n = 0x0,
    hw_ver_a11k = 0x1,
	hw_ver_unknown = 0xFF
};

struct fbm330_data {
	struct fbm330_calibration_data calibration;
	enum fbm330_osr oversampling_rate;	
	enum  fbm330_hw_version hw_ver;
	uint8_t chip_id;
	uint8_t cmd_start_p;
	uint8_t cmd_start_t;
	uint32_t cnvTime_temp; //unit:us
	uint32_t cnvTime_press; //unit:us
	uint32_t raw_temperature;
	uint32_t raw_pressure;
	int32_t real_temperature; //unit:0.01 degree Celsisu
	int32_t real_pressure; //unit: Pa
	/* bus read function pointer */
	uint8_t (*bus_read)(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
	/* bus write function pointer */
	uint8_t (*bus_write)(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
	/* delay function pointer */
	void (*delay_usec)(uint32_t us);
};

int8_t fbm330_init(void);
int32_t fbm330_read_raw_t(void);
int32_t fbm330_read_raw_p(void);
void fbm330_read_data(int32_t *real_pressure, int32_t *real_temperature);
float fbm330_read_temperature(void);
float fbm330_read_pressure(void);
void fbm330_update_data(void);

#endif
