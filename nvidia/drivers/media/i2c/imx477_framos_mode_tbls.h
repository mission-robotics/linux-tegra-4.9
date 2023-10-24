/*
 * imx477_mode_tbls.h - imx477 sensor mode tables
 *
 * Copyright (c) 2019. FRAMOS.  All rights reserved.
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

#ifndef __IMX477_TABLES__
#define __IMX477_TABLES__

/**
 * Image sensor registers as described in the IMX477 register map
 */

#define MASTER_SLAVE_SEL                0x3041
#define MODE_SEL                        0x0100
#define DT_PEDESTAL_HIGH                0x0008
#define DT_PEDESTAL_LOW                 0x0009
#define IMG_ORIENTATION_V_H             0x0101
#define SW_RESET                        0x0103
#define CSI_DT_FMT_H                    0x0112
#define CSI_DT_FMT_L                    0x0113
#define CSI_LANE_MODE                   0x0114
#define DPHY_CTRL                       0x0808
#define XVS_IO_CTRL                     0x3040
#define EXTOUT_EN                       0x4B81
#define EXCK_FREQ_HIGH                  0x0136
#define EXCK_FREQ_LOW                   0x0137

#define FRM_LENGTH_LINES_HIGH           0x0340
#define FRM_LENGTH_LINES_LOW            0x0341
#define LINE_LENGTH_PCK_HIGH            0x0342
#define LINE_LENGTH_PCK_LOW             0x0343
#define POWER_SAVE_ENABLE               0x3F50
#define LINE_LENGTH_INCK_HIGH           0x3F56
#define LINE_LENGTH_INCK_LOW            0x3F57
#define EBD_SIZE_V                      0xBCF1
#define GYROEN                          0x3237
#define FRM_LENGTH_CTL                  0x0350
#define LSC_CALC_MODE                   0x3804
#define ADBIT_MODE                      0x3F0D
#define MC_MODE                         0x3F0B
#define PRSH_LENGTH_LINES_HIGH          0x3F39
#define PRSH_LENGTH_LINES_MID           0x3F3A
#define PRSH_LENGTH_LINES_LOW           0x3F3B
#define COARSE_INTEG_TIME_HIGH          0x0202
#define COARSE_INTEG_TIME_LOW           0x0203
#define CIT_LSHIFT                      0x3100
#define IVT_PREPLLCK_DIV                0x0305
#define IVT_PLL_MPY_HIGH                0x0306
#define IVT_PLL_MPY_LOW                 0x0307
#define PLL_MULT_DRIV                   0x0310
#define IVT_SYCK_DIV                    0x0303
#define IVT_PXCK_DIV                    0x0301
#define IOP_SYCK_DIV                    0x030B
#define IOP_PXCK_DIV                    0x0309
#define REQ_LINK_BIT_RATE_MBPS_HH       0x0820
#define REQ_LINK_BIT_RATE_MBPS_HL       0x0821
#define REQ_LINK_BIT_RATE_MBPS_LH       0x0822
#define REQ_LINK_BIT_RATE_MBPS_LL       0x0823

#define X_ADD_STA_HIGH                  0x0344
#define X_ADD_STA_LOW                   0x0345
#define Y_ADD_STA_HIGH                  0x0346
#define Y_ADD_STA_LOW                   0x0347
#define X_ADD_END_HIGH                  0x0348
#define X_ADD_END_LOW                   0x0349
#define Y_ADD_END_HIGH                  0x034A
#define Y_ADD_END_LOW                   0x034B

#define X_OUT_SIZE_HIGH                 0x034C
#define X_OUT_SIZE_LOW                  0x034D
#define Y_OUT_SIZE_HIGH                 0x034E
#define Y_OUT_SIZE_LOW                  0x034F

#define BINNING_MODE                    0x0900
#define BINNING_TYPE_H_V                0x0901
#define BINNING_WEIGHTING               0x0902

#define FLL_LSHIFT                      0x3210
#define ANA_GAIN_GLOBAL_HIGH            0x0204
#define ANA_GAIN_GLOBAL_LOW             0x0205

#define DPGA_USE_GLOBAL_GAIN            0x3FF9
#define DIG_GAIN_GR_HIGH                0x020E
#define DIG_GAIN_GR_LOW                 0x020F

#define SCALE_MODE                      0x0401
#define SCALE_M_HIGH                    0x0404
#define SCALE_M_LOW                     0x0405
#define DIG_CROP_IMAGE_WIDTH_HIGH       0x040C
#define DIG_CROP_IMAGE_WIDTH_LOW        0x040D
#define DIG_CROP_IMAGE_HEIGHT_HIGH      0x040E
#define DIG_CROP_IMAGE_HEIGHT_LOW       0x040F
#define GRP_PARAM_HOLD                  0x0104
#define IOP_PREPLLCK_DIV                0x030D
#define IOP_PLL_MPY_HIGH                0x030E
#define IOP_PLL_MPY_LOW                 0x030F

#define TP_MODE                         0x0601

#define FRAME_BLANKSTOP_CL              0xE000
#define TEMP_SEN_CTL                    0x0138
#define TEMP_SEN_OUT                    0x013A

#define FLASH_STRB_ADJ                  0x0C12
#define FLASH_STRB_START_POINT_HIGH     0x0C14
#define FLASH_STRB_START_POINT_LOW      0x0C15
#define TFLASH_STRB_DLY_RS_CTL_HIGH     0x0C16
#define TFLASH_STRB_DLY_RS_CTL_LOW      0x0C17
#define TFLASH_STRB_WIDT_H_RS_CTL_HIGH  0x0C18
#define TFLASH_STRB_WIDT_H_RS_CTL_LOW   0x0C19
#define FLASH_MD_RS                     0x0C1A
#define FLASH_TRIG_RS                   0x0C1B
#define FLASH_STAT                      0x0C1C
#define TFLASH_STRB_WIDT2_H_RS_CTL_HIGH 0x0C26
#define TFLASH_STRB_WIDT2_H_RS_CTL_LOW  0x0C27
#define TFLASH_STRB_WIDT_L_RS_CTL_HIGH  0x0C28
#define TFLASH_STRB_WIDT_L_RS_CTL_LOW   0x0C29
#define TFLASH_STRB_CNT_RS_CTL          0x0C2A

#define MANUAL_DATA_PEDESTAL_VALUE_HIGH 0x3032
#define MANUAL_DATA_PEDESTAL_VALUE_LOW  0x3033
#define MANUAL_DATA_PEDESTAL_EN         0x3030

#define PDAF_CTRL1_SPC                  0x3E35
#define PDAF_CTRL1_DCC                  0x3E36
#define PDAF_CTRL1_PDD                  0x3E37

#define TP_MODE                         0x0601
#define SLAVE_ADD_EN_2ND                0x3010
#define SLAVE_ADD_ACKEN_2ND             0x3011

/**
 * Resolutions of implemented frame modes
 */

#define IMX477_DEFAULT_WIDTH            4056
#define IMX477_DEFAULT_HEIGHT           3040
#define IMX477_CROP_3840x2160_WIDTH     3840
#define IMX477_CROP_3840x2160_HEIGHT    2160
#define IMX477_MODE_SCALE_HV3_WIDTH     1280
#define IMX477_MODE_SCALE_HV3_HEIGHT    720
#define IMX477_MODE_CROP_BINNING_H2V2_WIDTH  1920
#define IMX477_MODE_CROP_BINNING_H2V2_HEIGHT 1080

/**
 * Special values for the write table function
 */
#define IMX477_TABLE_WAIT_MS 0
#define IMX477_TABLE_END     1
#define IMX477_WAIT_MS       10

/**
 * Minimal value of frame length is resolution height + 52
 * Minimal value for scaling modes is full pixel mode height + 52
 *
 * Determined from the default value of FRM_LENGTH_LINES register
 * and empirically confirmed
 */
#define IMX477_MIN_FRAME_LENGTH_DELTA 52

typedef struct reg_8 imx477_reg;

/**
 * Tables for the write table function
 */

static const imx477_reg imx477_start[] = {

    {MODE_SEL,             0x01},

    {IMX477_TABLE_WAIT_MS, IMX477_WAIT_MS},
    {IMX477_TABLE_END,     0x00}
};


static const imx477_reg imx477_stop[] = {

    {MODE_SEL,             0x00},
    {SCALE_MODE,           0x00},
    {BINNING_MODE,         0x00},

    {IMX477_TABLE_WAIT_MS, IMX477_WAIT_MS},
    {IMX477_TABLE_END,     0x00}
};

static const imx477_reg imx477_8bit_mode[] = {

    {MODE_SEL,             0x00},

    {ADBIT_MODE,           0x00},
    {CSI_DT_FMT_L,         0x08},
    {CSI_DT_FMT_H,         0x08},
    {IOP_PXCK_DIV,         0x08},

    {LINE_LENGTH_PCK_HIGH, 0x11},
    {LINE_LENGTH_PCK_LOW,  0xA0},

    {IMX477_TABLE_WAIT_MS, IMX477_WAIT_MS},
    {IMX477_TABLE_END,     0x00}
};

static const imx477_reg imx477_10bit_mode[] = {

    {MODE_SEL,             0x00},

    {ADBIT_MODE,           0x00},
    {CSI_DT_FMT_L,         0x0A},
    {CSI_DT_FMT_H,         0x0A},
    {IOP_PXCK_DIV,         0x0A},

    {LINE_LENGTH_PCK_HIGH, 4512 >> 8},
    {LINE_LENGTH_PCK_LOW,  4512 & 0xFF},

    {IMX477_TABLE_WAIT_MS, IMX477_WAIT_MS},
    {IMX477_TABLE_END,     0x00}
};

static const imx477_reg imx477_12bit_mode[] = {

    {MODE_SEL,             0x00},

    {ADBIT_MODE,           0x01},
    {CSI_DT_FMT_L,         0x0C},
    {CSI_DT_FMT_H,         0x0C},
    {IOP_PXCK_DIV,         0x0C},

    {LINE_LENGTH_PCK_HIGH, 0x18},
    {LINE_LENGTH_PCK_LOW,  0x50},

    {IMX477_TABLE_WAIT_MS, IMX477_WAIT_MS},
    {IMX477_TABLE_END,     0x00}
};

static const imx477_reg imx477_init_settings[] = {

    {IVT_PREPLLCK_DIV,        0x04},
    {IOP_SYCK_DIV,            0x01},

    {FRM_LENGTH_CTL,          0x01},

    {PLL_MULT_DRIV,           0x00},

    {DPHY_CTRL,               0x00},

    {PDAF_CTRL1_SPC,          0x00},
    {PDAF_CTRL1_DCC,          0x01},
    {PDAF_CTRL1_PDD,          0x00},

    {MANUAL_DATA_PEDESTAL_EN, 0x01},
    {CSI_LANE_MODE,           0x03},

    {MC_MODE,                 0x00},
    {MASTER_SLAVE_SEL,        0x01},

    {GYROEN,                  0x01},
    {DPGA_USE_GLOBAL_GAIN,    0x01},

    {IMX477_TABLE_WAIT_MS,    IMX477_WAIT_MS},
    {IMX477_TABLE_END,        0x0000}
};

static const imx477_reg imx477_global_settings[] = {

    {0x0808,	0x02},
    {0xE07A,	0x01},
    {0xE000,	0x00},
    {0x4AE9,	0x18},
    {0x4AEA,	0x08},
    {0xF61C,	0x04},
    {0xF61E,	0x04},
    {0x4AE9,	0x21},
    {0x4AEA,	0x80},
    {0x38A8,	0x1F},
    {0x38A9,	0xFF},
    {0x38AA,	0x1F},
    {0x38AB,	0xFF},
    {0x420B,	0x01},
    {0x55D4,	0x00},
    {0x55D5,	0x00},
    {0x55D6,	0x07},
    {0x55D7,	0xFF},
    {0x55E8,	0x07},
    {0x55E9,	0xFF},
    {0x55EA,	0x00},
    {0x55EB,	0x00},
    {0x574C,	0x07},
    {0x574D,	0xFF},
    {0x574E,	0x00},
    {0x574F,	0x00},
    {0x5754,	0x00},
    {0x5755,	0x00},
    {0x5756,	0x07},
    {0x5757,	0xFF},
    {0x5973,	0x04},
    {0x5974,	0x01},
    {0x5D13,	0xC3},
    {0x5D14,	0x58},
    {0x5D15,	0xA3},
    {0x5D16,	0x1D},
    {0x5D17,	0x65},
    {0x5D18,	0x8C},
    {0x5D1A,	0x06},
    {0x5D1B,	0xA9},
    {0x5D1C,	0x45},
    {0x5D1D,	0x3A},
    {0x5D1E,	0xAB},
    {0x5D1F,	0x15},
    {0x5D21,	0x0E},
    {0x5D22,	0x52},
    {0x5D23,	0xAA},
    {0x5D24,	0x7D},
    {0x5D25,	0x57},
    {0x5D26,	0xA8},
    {0x5D37,	0x5A},
    {0x5D38,	0x5A},
    {0x5D77,	0x7F},
    {0x7B7C,	0x00},
    {0x7B7D,	0x00},
    {0x8D1F,	0x00},
    {0x8D27,	0x00},
    {0x9004,	0x03},
    {0x9200,	0x50},
    {0x9201,	0x6C},
    {0x9202,	0x71},
    {0x9203,	0x00},
    {0x9204,	0x71},
    {0x9205,	0x01},
    {0x9371,	0x6A},
    {0x9373,	0x6A},
    {0x9375,	0x64},
    {0x990C,	0x00},
    {0x990D,	0x08},
    {0x9956,	0x8C},
    {0x9957,	0x64},
    {0x9958,	0x50},
    {0x9A48,	0x06},
    {0x9A49,	0x06},
    {0x9A4A,	0x06},
    {0x9A4B,	0x06},
    {0x9A4C,	0x06},
    {0x9A4D,	0x06},
    {0xA001,	0x0A},
    {0xA003,	0x0A},
    {0xA005,	0x0A},
    {0xA006,	0x01},
    {0xA007,	0xC0},
    {0xA009,	0xC0},
    {0x5078,	0x00}, //for IMX477-AAPK

    {IMX477_TABLE_WAIT_MS,    IMX477_WAIT_MS},
    {IMX477_TABLE_END,        0x0000}
};

static const imx477_reg imx477_imageQuality_settings[] = {

    {0x3D8A,	0x01},
    {0x7B3B,	0x01},
    {0x7B4C,	0x00},
    {0x9905,	0x00},
    {0x9907,	0x00},
    {0x9909,	0x00},
    {0x990B,	0x00},
    {0x9944,	0x3C},
    {0x9947,	0x3C},
    {0x994A,	0x8C},
    {0x994B,	0x50},
    {0x994C,	0x1B},
    {0x994D,	0x8C},
    {0x994E,	0x50},
    {0x994F,	0x1B},
    {0x9950,	0x8C},
    {0x9951,	0x1B},
    {0x9952,	0x0A},
    {0x9953,	0x8C},
    {0x9954,	0x1B},
    {0x9955,	0x0A},
    {0x9A13,	0x04},
    {0x9A14,	0x04},
    {0x9A19,	0x00},
    {0x9A1C,	0x04},
    {0x9A1D,	0x04},
    {0x9A26,	0x05},
    {0x9A27,	0x05},
    {0x9A2C,	0x01},
    {0x9A2D,	0x03},
    {0x9A2F,	0x05},
    {0x9A30,	0x05},
    {0x9A41,	0x00},
    {0x9A46,	0x00},
    {0x9A47,	0x00},
    {0x9C17,	0x35},
    {0x9C1D,	0x31},
    {0x9C29,	0x50},
    {0x9C3B,	0x2F},
    {0x9C41,	0x6B},
    {0x9C47,	0x2D},
    {0x9C4D,	0x40},
    {0x9C6B,	0x00},
    {0x9C71,	0xC8},
    {0x9C73,	0x32},
    {0x9C75,	0x04},
    {0x9C7D,	0x2D},
    {0x9C83,	0x40},
    {0x9C94,	0x3F},
    {0x9C95,	0x3F},
    {0x9C96,	0x3F},
    {0x9C97,	0x00},
    {0x9C98,	0x00},
    {0x9C99,	0x00},
    {0x9C9A,	0x3F},
    {0x9C9B,	0x3F},
    {0x9C9C,	0x3F},
    {0x9CA0,	0x0F},
    {0x9CA1,	0x0F},
    {0x9CA2,	0x0F},
    {0x9CA3,	0x00},
    {0x9CA4,	0x00},
    {0x9CA5,	0x00},
    {0x9CA6,	0x1E},
    {0x9CA7,	0x1E},
    {0x9CA8,	0x1E},
    {0x9CA9,	0x00},
    {0x9CAA,	0x00},
    {0x9CAB,	0x00},
    {0x9CAC,	0x09},
    {0x9CAD,	0x09},
    {0x9CAE,	0x09},
    {0x9CBD,	0x50},
    {0x9CBF,	0x50},
    {0x9CC1,	0x50},
    {0x9CC3,	0x40},
    {0x9CC5,	0x40},
    {0x9CC7,	0x40},
    {0x9CC9,	0x0A},
    {0x9CCB,	0x0A},
    {0x9CCD,	0x0A},
    {0x9D17,	0x35},
    {0x9D1D,	0x31},
    {0x9D29,	0x50},
    {0x9D3B,	0x2F},
    {0x9D41,	0x6B},
    {0x9D47,	0x42},
    {0x9D4D,	0x5A},
    {0x9D6B,	0x00},
    {0x9D71,	0xC8},
    {0x9D73,	0x32},
    {0x9D75,	0x04},
    {0x9D7D,	0x42},
    {0x9D83,	0x5A},
    {0x9D94,	0x3F},
    {0x9D95,	0x3F},
    {0x9D96,	0x3F},
    {0x9D97,	0x00},
    {0x9D98,	0x00},
    {0x9D99,	0x00},
    {0x9D9A,	0x3F},
    {0x9D9B,	0x3F},
    {0x9D9C,	0x3F},
    {0x9D9D,	0x1F},
    {0x9D9E,	0x1F},
    {0x9D9F,	0x1F},
    {0x9DA0,	0x0F},
    {0x9DA1,	0x0F},
    {0x9DA2,	0x0F},
    {0x9DA3,	0x00},
    {0x9DA4,	0x00},
    {0x9DA5,	0x00},
    {0x9DA6,	0x1E},
    {0x9DA7,	0x1E},
    {0x9DA8,	0x1E},
    {0x9DA9,	0x00},
    {0x9DAA,	0x00},
    {0x9DAB,	0x00},
    {0x9DAC,	0x09},
    {0x9DAD,	0x09},
    {0x9DAE,	0x09},
    {0x9DC9,	0x0A},
    {0x9DCB,	0x0A},
    {0x9DCD,	0x0A},
    {0x9E17,	0x35},
    {0x9E1D,	0x31},
    {0x9E29,	0x50},
    {0x9E3B,	0x2F},
    {0x9E41,	0x6B},
    {0x9E47,	0x2D},
    {0x9E4D,	0x40},
    {0x9E6B,	0x00},
    {0x9E71,	0xC8},
    {0x9E73,	0x32},
    {0x9E75,	0x04},
    {0x9E94,	0x0F},
    {0x9E95,	0x0F},
    {0x9E96,	0x0F},
    {0x9E97,	0x00},
    {0x9E98,	0x00},
    {0x9E99,	0x00},
    {0x9EA0,	0x0F},
    {0x9EA1,	0x0F},
    {0x9EA2,	0x0F},
    {0x9EA3,	0x00},
    {0x9EA4,	0x00},
    {0x9EA5,	0x00},
    {0x9EA6,	0x3F},
    {0x9EA7,	0x3F},
    {0x9EA8,	0x3F},
    {0x9EA9,	0x00},
    {0x9EAA,	0x00},
    {0x9EAB,	0x00},
    {0x9EAC,	0x09},
    {0x9EAD,	0x09},
    {0x9EAE,	0x09},
    {0x9EC9,	0x0A},
    {0x9ECB,	0x0A},
    {0x9ECD,	0x0A},
    {0x9F17,	0x35},
    {0x9F1D,	0x31},
    {0x9F29,	0x50},
    {0x9F3B,	0x2F},
    {0x9F41,	0x6B},
    {0x9F47,	0x42},
    {0x9F4D,	0x5A},
    {0x9F6B,	0x00},
    {0x9F71,	0xC8},
    {0x9F73,	0x32},
    {0x9F75,	0x04},
    {0x9F94,	0x0F},
    {0x9F95,	0x0F},
    {0x9F96,	0x0F},
    {0x9F97,	0x00},
    {0x9F98,	0x00},
    {0x9F99,	0x00},
    {0x9F9A,	0x2F},
    {0x9F9B,	0x2F},
    {0x9F9C,	0x2F},
    {0x9F9D,	0x00},
    {0x9F9E,	0x00},
    {0x9F9F,	0x00},
    {0x9FA0,	0x0F},
    {0x9FA1,	0x0F},
    {0x9FA2,	0x0F},
    {0x9FA3,	0x00},
    {0x9FA4,	0x00},
    {0x9FA5,	0x00},
    {0x9FA6,	0x1E},
    {0x9FA7,	0x1E},
    {0x9FA8,	0x1E},
    {0x9FA9,	0x00},
    {0x9FAA,	0x00},
    {0x9FAB,	0x00},
    {0x9FAC,	0x09},
    {0x9FAD,	0x09},
    {0x9FAE,	0x09},
    {0x9FC9,	0x0A},
    {0x9FCB,	0x0A},
    {0x9FCD,	0x0A},
    {0xA14B,	0xFF},
    {0xA151,	0x0C},
    {0xA153,	0x50},
    {0xA155,	0x02},
    {0xA157,	0x00},
    {0xA1AD,	0xFF},
    {0xA1B3,	0x0C},
    {0xA1B5,	0x50},
    {0xA1B9,	0x00},
    {0xA24B,	0xFF},
    {0xA257,	0x00},
    {0xA2AD,	0xFF},
    {0xA2B9,	0x00},
    {0xB21F,	0x04},
    {0xB35C,	0x00},
    {0xB35E,	0x08},

    {IMX477_TABLE_WAIT_MS,    IMX477_WAIT_MS},
    {IMX477_TABLE_END,        0x0000}
};

static const imx477_reg mode_4056x3040[] = {

    {FRM_LENGTH_LINES_HIGH,      (IMX477_DEFAULT_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) >> 8},
    {FRM_LENGTH_LINES_LOW,       (IMX477_DEFAULT_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) & 0xFF},
    {SCALE_MODE,                 0x00},

    {X_ADD_STA_HIGH,             0 >> 8},
    {X_ADD_STA_LOW,              0 & 0xFF},
    {Y_ADD_STA_HIGH,             0 >> 8},
    {Y_ADD_STA_LOW,              0 & 0xFF},
    {X_ADD_END_HIGH,             (IMX477_DEFAULT_WIDTH - 1) >> 8},
    {X_ADD_END_LOW,              (IMX477_DEFAULT_WIDTH - 1) & 0xFF},
    {Y_ADD_END_HIGH,             (IMX477_DEFAULT_HEIGHT - 1) >> 8},
    {Y_ADD_END_LOW,              (IMX477_DEFAULT_HEIGHT - 1) & 0xFF},

    {DIG_CROP_IMAGE_WIDTH_HIGH,  IMX477_DEFAULT_WIDTH >> 8},
    {DIG_CROP_IMAGE_WIDTH_LOW,   IMX477_DEFAULT_WIDTH & 0xFF},
    {DIG_CROP_IMAGE_HEIGHT_HIGH, IMX477_DEFAULT_HEIGHT >> 8},
    {DIG_CROP_IMAGE_HEIGHT_LOW,  IMX477_DEFAULT_HEIGHT & 0xFF},

    {X_OUT_SIZE_HIGH,            IMX477_DEFAULT_WIDTH >> 8},
    {X_OUT_SIZE_LOW,             IMX477_DEFAULT_WIDTH & 0xFF},

    {Y_OUT_SIZE_HIGH,            IMX477_DEFAULT_HEIGHT >> 8},
    {Y_OUT_SIZE_LOW,             IMX477_DEFAULT_HEIGHT & 0xFF},

    {IMX477_TABLE_WAIT_MS,       IMX477_WAIT_MS},
    {IMX477_TABLE_END,           0x0000}
};

static const imx477_reg mode_3840x2160[] = {

    {FRM_LENGTH_LINES_HIGH,      (IMX477_CROP_3840x2160_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) >> 8},
    {FRM_LENGTH_LINES_LOW,       (IMX477_CROP_3840x2160_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) & 0xFF},
    {SCALE_MODE,                 0x00},

    {X_ADD_STA_HIGH,             108 >> 8},
    {X_ADD_STA_LOW,              108 & 0xFF},
    {Y_ADD_STA_HIGH,             440 >> 8},
    {Y_ADD_STA_LOW,              440 & 0xFF},
    {X_ADD_END_HIGH,             (3948 - 1) >> 8},
    {X_ADD_END_LOW,              (3948 - 1) & 0xFF},
    {Y_ADD_END_HIGH,             (2600 - 1) >> 8},
    {Y_ADD_END_LOW,              (2600 - 1) & 0xFF},

    {DIG_CROP_IMAGE_WIDTH_HIGH,  IMX477_CROP_3840x2160_WIDTH >> 8},
    {DIG_CROP_IMAGE_WIDTH_LOW,   IMX477_CROP_3840x2160_WIDTH & 0xFF},
    {DIG_CROP_IMAGE_HEIGHT_HIGH, IMX477_CROP_3840x2160_HEIGHT >> 8},
    {DIG_CROP_IMAGE_HEIGHT_LOW,  IMX477_CROP_3840x2160_HEIGHT & 0xFF},

    {X_OUT_SIZE_HIGH,            IMX477_CROP_3840x2160_WIDTH >> 8},
    {X_OUT_SIZE_LOW,             IMX477_CROP_3840x2160_WIDTH & 0xFF},

    {Y_OUT_SIZE_HIGH,            IMX477_CROP_3840x2160_HEIGHT >> 8},
    {Y_OUT_SIZE_LOW,             IMX477_CROP_3840x2160_HEIGHT & 0xFF},

    {IMX477_TABLE_WAIT_MS,       IMX477_WAIT_MS},
    {IMX477_TABLE_END,           0x0000}
};

static const imx477_reg mode_scale_hv3[] = {

    {FRM_LENGTH_LINES_HIGH,      (IMX477_DEFAULT_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) >> 8},
    {FRM_LENGTH_LINES_LOW,       (IMX477_DEFAULT_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) & 0xFF},

    {SCALE_MODE,                 0x02},
    {SCALE_M_HIGH,               48 >> 8},
    {SCALE_M_LOW,                48 & 0xFF},

    {X_ADD_STA_HIGH,             108 >> 8},
    {X_ADD_STA_LOW,              108 & 0xFF},
    {Y_ADD_STA_HIGH,             440 >> 8},
    {Y_ADD_STA_LOW,              440 & 0xFF},
    {X_ADD_END_HIGH,             (108 + IMX477_MODE_SCALE_HV3_WIDTH*3 - 1) >> 8},
    {X_ADD_END_LOW,              (108 + IMX477_MODE_SCALE_HV3_WIDTH*3 - 1) & 0xFF},
    {Y_ADD_END_HIGH,             (440 + IMX477_MODE_SCALE_HV3_HEIGHT*3 - 1) >> 8},
    {Y_ADD_END_LOW,              (440 + IMX477_MODE_SCALE_HV3_HEIGHT*3 - 1) & 0xFF},
    {BINNING_MODE,               0x00},
    {DIG_CROP_IMAGE_WIDTH_HIGH,  IMX477_MODE_SCALE_HV3_WIDTH*3 >> 8},
    {DIG_CROP_IMAGE_WIDTH_LOW,   IMX477_MODE_SCALE_HV3_WIDTH*3 & 0xFF},
    {DIG_CROP_IMAGE_HEIGHT_HIGH, IMX477_MODE_SCALE_HV3_HEIGHT*3 >> 8},
    {DIG_CROP_IMAGE_HEIGHT_LOW,  IMX477_MODE_SCALE_HV3_HEIGHT*3 & 0xFF},

    {X_OUT_SIZE_HIGH,            IMX477_MODE_SCALE_HV3_WIDTH >> 8},
    {X_OUT_SIZE_LOW,             IMX477_MODE_SCALE_HV3_WIDTH & 0xFF},

    {Y_OUT_SIZE_HIGH,            IMX477_MODE_SCALE_HV3_HEIGHT >> 8},
    {Y_OUT_SIZE_LOW,             IMX477_MODE_SCALE_HV3_HEIGHT & 0xFF},

    {IMX477_TABLE_WAIT_MS,       IMX477_WAIT_MS},
    {IMX477_TABLE_END,           0x0000}
};

static const imx477_reg mode_crop_binning_h2v2[] = {

    {FRM_LENGTH_LINES_HIGH,      (IMX477_MODE_CROP_BINNING_H2V2_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) >> 8},
    {FRM_LENGTH_LINES_LOW,       (IMX477_MODE_CROP_BINNING_H2V2_HEIGHT + IMX477_MIN_FRAME_LENGTH_DELTA) & 0xFF},
    {SCALE_MODE,                 0x00},

    {X_ADD_STA_HIGH,            108 >> 8},
    {X_ADD_STA_LOW,             108 & 0xFF},
    {Y_ADD_STA_HIGH,            440 >> 8},
    {Y_ADD_STA_LOW,             440 & 0xFF},
    {X_ADD_END_HIGH,            (108 + IMX477_MODE_CROP_BINNING_H2V2_WIDTH*2 - 1) >> 8},
    {X_ADD_END_LOW,             (108 + IMX477_MODE_CROP_BINNING_H2V2_WIDTH*2 - 1) & 0xFF},
    {Y_ADD_END_HIGH,            (440 + IMX477_MODE_CROP_BINNING_H2V2_HEIGHT*2 - 1) >> 8},
    {Y_ADD_END_LOW,             (440 + IMX477_MODE_CROP_BINNING_H2V2_HEIGHT*2 - 1) & 0xFF},

    {BINNING_MODE,               0x01},
    {BINNING_TYPE_H_V,           0x22},
    {LINE_LENGTH_PCK_HIGH,       2976 >> 8},
    {LINE_LENGTH_PCK_LOW,        2976 & 0xFF},

    {DIG_CROP_IMAGE_WIDTH_HIGH,  IMX477_MODE_CROP_BINNING_H2V2_WIDTH >> 8},
    {DIG_CROP_IMAGE_WIDTH_LOW,   IMX477_MODE_CROP_BINNING_H2V2_WIDTH & 0xFF},
    {DIG_CROP_IMAGE_HEIGHT_HIGH, IMX477_MODE_CROP_BINNING_H2V2_HEIGHT >> 8},
    {DIG_CROP_IMAGE_HEIGHT_LOW,  IMX477_MODE_CROP_BINNING_H2V2_HEIGHT & 0xFF},

    {X_OUT_SIZE_HIGH,            IMX477_MODE_CROP_BINNING_H2V2_WIDTH >> 8},
    {X_OUT_SIZE_LOW,             IMX477_MODE_CROP_BINNING_H2V2_WIDTH & 0xFF},

    {Y_OUT_SIZE_HIGH,            IMX477_MODE_CROP_BINNING_H2V2_HEIGHT >> 8},
    {Y_OUT_SIZE_LOW,             IMX477_MODE_CROP_BINNING_H2V2_HEIGHT & 0xFF},

    {IMX477_TABLE_WAIT_MS,       IMX477_WAIT_MS},
    {IMX477_TABLE_END,           0x0000}
};

/**
 * Enum of available frame modes
 */

enum {
    /* 4:3 frame modes */
    IMX477_MODE_4056x3040,

    /* 16:9 frame modes */
    IMX477_MODE_CROP_3840x2160,

    /* scaling frame modes */
    IMX477_MODE_SCALE_HV3,

    /* binning frame modes */
    IMX477_MODE_CROP_BINNING_H2V2,

    IMX477_8BIT_MODE,
    IMX477_10BIT_MODE,
    IMX477_12BIT_MODE,

    IMX477_INIT_SETTINGS,
    IMX477_GLOBAL_SETTINGS,
    IMX477_IMAGEQUALITY_SETTINGS,

    IMX477_MODE_START_STREAM,
    IMX477_MODE_STOP_STREAM,
};

/**
 * Connecting frame modes to mode tables
 */

static const imx477_reg *mode_table[] = {

    [IMX477_MODE_4056x3040]      = mode_4056x3040,
    [IMX477_MODE_CROP_3840x2160] = mode_3840x2160,

    [IMX477_MODE_SCALE_HV3]      = mode_scale_hv3,

    [IMX477_MODE_CROP_BINNING_H2V2]   = mode_crop_binning_h2v2,

    [IMX477_8BIT_MODE]           = imx477_8bit_mode,
    [IMX477_10BIT_MODE]          = imx477_10bit_mode,
    [IMX477_12BIT_MODE]          = imx477_12bit_mode,

    [IMX477_INIT_SETTINGS]       = imx477_init_settings,
    [IMX477_GLOBAL_SETTINGS]     = imx477_global_settings,
    [IMX477_IMAGEQUALITY_SETTINGS] = imx477_imageQuality_settings,

    [IMX477_MODE_START_STREAM]   = imx477_start,
    [IMX477_MODE_STOP_STREAM]    = imx477_stop,
};

/**
 * Framerates of available frame modes
 */

static const int imx477_60fps[] = {
    60,
};
static const int imx477_84fps[] = {
    84,
};
static const int imx477_249fps[] = {
    249,
};

/**
 * Connecting resolutions, framerates and mode tables
 */

static const struct camera_common_frmfmt imx477_frmfmt[] = {
    {
        .size = {IMX477_DEFAULT_WIDTH, IMX477_DEFAULT_HEIGHT},
        .framerates = imx477_60fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX477_MODE_4056x3040
    },
    {
        .size = {IMX477_CROP_3840x2160_WIDTH, IMX477_CROP_3840x2160_HEIGHT},
        .framerates = imx477_84fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX477_MODE_CROP_3840x2160
    },
    {
        .size = {IMX477_MODE_SCALE_HV3_WIDTH, IMX477_MODE_SCALE_HV3_HEIGHT},
        .framerates = imx477_60fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX477_MODE_SCALE_HV3
    },
    {
        .size = {IMX477_MODE_CROP_BINNING_H2V2_WIDTH, IMX477_MODE_CROP_BINNING_H2V2_HEIGHT},
        .framerates = imx477_249fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX477_MODE_CROP_BINNING_H2V2
    },
};

#endif /* __IMX477_TABLES__ */
