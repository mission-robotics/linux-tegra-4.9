/*
 * imx304_mode_tbls.h - imx304 sensor mode tables
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __IMX304_I2C_TABLES__
#define __IMX304_I2C_TABLES__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

#define IMX304_TABLE_WAIT_MS	0
#define IMX304_TABLE_END	1
#define IMX304_MAX_RETRIES	3
#define IMX304_WAIT_MS_STOP	1
#define IMX304_WAIT_MS_START	3
#define IMX304_WAIT_MS_STREAM	10

/* #define INIT_ET_INSETTING 1 */

#define imx304_reg struct reg_8

static imx304_reg imx304_start[] = {
	{0x3000, 0x00},
	{IMX304_TABLE_WAIT_MS, IMX304_WAIT_MS_START},
	{0x300A, 0x00},
	{IMX304_TABLE_WAIT_MS, IMX304_WAIT_MS_STREAM},
	{ IMX304_TABLE_END, 0x00}
};

static imx304_reg imx304_stop[] = {
	{0x300A, 0x01},
	{0x3000, 0x01},
	{IMX304_TABLE_WAIT_MS, IMX304_WAIT_MS_STOP},
	{IMX304_TABLE_END, 0x00}
};

static imx304_reg tp_colorbars[] = {

	{IMX304_TABLE_WAIT_MS, IMX304_WAIT_MS_STOP},
	{IMX304_TABLE_END, 0x00}
};

/*
 * All pixel, 4096x3000@15.1fps
 * Total number of pixels 4304x3042
 * Total data rate 2.376Gbps, 4ch, 594Mbps/ch
 * A/D conversion 12
 */
static  imx304_reg imx304_4096x3000_15fps[] = {
	{0x3001, 0xD0},
	{0x3002, 0xAA},
	{0x3005, 0x20}, // SETBLVDS, 4ch LVDS
	{0x300B, 0x01}, // Global shutter(trigger mode)
	{0x300C, 0x01},
	{0x300D, 0x00}, // WINMODE All-pixel mode, HMODE All-pixel
	{0x300E, 0x03}, // H/V direction inverted
	{0x3010, 0xE2}, // VMAX, 3042 line
	{0x3011, 0x0B}, //
	{0x3014, 0x4E}, // HMAX
	{0x3015, 0x06}, //
	{0x3016, 0x01},
	{0x3018, 0x01},
	{0x301C, 0x30}, // OPORTSEL, 4ch LVDS
	{0x3036, 0xC0}, // XHS/XVS pin setting(Normal output)
	{0x3079, 0x80},
	{0x3089, 0x10}, // INCKSEL0, 37.125MHZ
	{0x308A, 0x02}, // INCKSEL1, 37.125MHZ
	{0x308B, 0x10}, // INCKSEL2, 37.125MHZ
	{0x308C, 0x02}, // INCKSEL3, 37.125MHZ
	{0x308D, 0x14}, // SHS
	{0x308E, 0x00}, //
	{0x308F, 0x00}, //
	{0x3090, 0x0A},
	{0x3094, 0x0A},
	{0x3098, 0x0A},
	{0x309E, 0x08},
	{0x30A0, 0x06},
	{0x30AE, 0x01}, // Fast trigger mode
	{0x30AF, 0x0C},
	{0x3165, 0x40},
	{0x3166, 0x00},
	{0x3204, 0x00}, // GAIN
	{0x3205, 0x00}, //
	{0x3212, 0x08}, // GAINDLY
	{0x3226, 0x03},
	{0x3254, 0xF0}, // BLKLEVEL
	{0x3255, 0x00}, //
	{0x3518, 0x78},
	{0x3519, 0x0C},
	{0x3D70, 0x1E},
	{0x3D71, 0x00},
	{0x3D72, 0x67},
	{0x3D73, 0x01},
	{0x3D74, 0x1E},
	{0x3D75, 0x00},
	{0x3D76, 0x67},
	{0x3D77, 0x01},
	{0x4002, 0x20},
	{0x4003, 0x55},
	{0x4017, 0x03},
	{0x401E, 0x03},
	{0x403D, 0x24},
	{0x4040, 0x09},
	{0x4041, 0x6A},
	{0x404A, 0xC0},
	{0x4056, 0x18},
	{0x4094, 0x06},

	{IMX304_TABLE_END, 0x00}
};

enum {
	IMX304_MODE_4096X3000_15FPS,

	IMX304_MODE_START_STREAM,
	IMX304_MODE_STOP_STREAM,
	IMX304_MODE_TEST_PATTERN
};

static imx304_reg *mode_table[] = {
	[IMX304_MODE_4096X3000_15FPS] = imx304_4096x3000_15fps,

	[IMX304_MODE_START_STREAM] = imx304_start,
	[IMX304_MODE_STOP_STREAM] = imx304_stop,
	[IMX304_MODE_TEST_PATTERN] = tp_colorbars,
};

static const int imx304_15fps[] = {
	15,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt imx304_frmfmt[] = {
	{{4096, 3018}, imx304_15fps, 1, 0, IMX304_MODE_4096X3000_15FPS},
	/* Add modes with no device tree support after below */
};
#endif /* __IMX304_I2C_TABLES__ */
