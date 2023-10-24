/*
 * Copyright (c) 2019, The Imaging Source Europe GmbH.  All rights reserved.
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

#ifndef __TIS_IMX390_H__
#define __TIS_IMX390_H__

#define tis_imx390_reg struct reg_8


#define TIS_IMX390_MIN_GAIN 0
#define TIS_IMX390_MAX_GAIN 100

#define TIS_IMX390_MIN_FRAME_LENGTH    (1125u)
#define TIS_IMX390_MAX_FRAME_LENGTH    (0x1ffff)
#define TIS_IMX390_MAX_COARSE_DIFF	   (9u)
#define TIS_IMX390_MIN_EXPOSURE_COARSE (1u)
#define TIS_IMX390_MAX_EXPOSURE_COARSE \
	(TIS_IMX390_MAX_FRAME_LENGTH-TIS_IMX390_MAX_COARSE_DIFF)
#define TIS_IMX390_DEFAULT_FRAME_LENGTH    (1125u)
#define TIS_IMX390_DEFAULT_EXPOSURE_COARSE \
	(TIS_IMX390_DEFAULT_FRAME_LENGTH-TIS_IMX390_MAX_COARSE_DIFF)

#define TIS_IMX390_MIN_SHS1_1080P_HDR    (5u)
#define TIS_IMX390_MIN_SHS2_1080P_HDR    (82u)
#define TIS_IMX390_MAX_SHS2_1080P_HDR    (TIS_IMX390_MAX_FRAME_LENGTH - 5)
#define TIS_IMX390_MAX_SHS1_1080P_HDR    (TIS_IMX390_MAX_SHS2_1080P_HDR / 16)

#define TIS_IMX390_VMAX_ADDR        0x2008
#define TIS_IMX390_SHS1_ADDR        0x000c
#define TIS_IMX390_SHS2_ADDR        0x0010
#define TIS_IMX390_REGHOLD_ADDR     0x0008

#define TIS_IMX390_AGAIN_SP1H		0x0018
#define TIS_IMX390_AGAIN_SP1L		0x001A

#define TIS_IMX390_ANALOG_GAIN_SP1H_ADDR    0x0018
#define TIS_IMX390_ANALOG_GAIN_SP1L_ADDR    0x001A

#define TIS_IMX390_WDC_OUTSEL		0x00F9
#define TIS_IMX390_WDC_THR_FRM_SEL	0x00FA
#define TIS_IMX390_PWL_THRU			0x013B

#define TIS_IMX390_REG_MODE_VMAX	0x2008
#define TIS_IMX390_REG_MODE_HMAX	0x200C
#define TIS_IMX390_REG_FMAX			0x0090
#define TIS_IMX390_REG_SHS1			0x000c
#define TIS_IMX390_REG_SHS2			0x0010

#define TIS_IMX390_REG_APPLICATION_LOCK	0x03C0
#define TIS_IMX390_REG_CROP_ON			0x0078
#define TIS_IMX390_REG_CROP_H_OFFSET	0x007C
#define TIS_IMX390_REG_CROP_V_OFFSET	0x0080
#define TIS_IMX390_REG_CROP_H_SIZE		0x3410
#define TIS_IMX390_REG_CROP_V_SIZE		0x3418

struct tis_imx390_priv
{
	struct
	{
		struct v4l2_ctrl *output_sel;
		struct v4l2_ctrl *gain_offset;
		struct v4l2_ctrl *exposure_ratio;
		struct v4l2_ctrl *subtract_smpg_height;
	} ctrl;
};

#define TIS_IMX390_PRIV(tis_sensor) ((struct tis_imx390_priv*)tis_sensor->priv)

#endif