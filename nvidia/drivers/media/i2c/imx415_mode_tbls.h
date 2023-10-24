/*
 * imx415_mode_tbls.h - imx415 sensor mode tables
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

#ifndef __IMX415_TABLES__
#define __IMX415_TABLES__

/**
 * Image sensor registers as described in the IMX415 register map
 */

#define STANDBY             0x3000
#define REGHOLD             0x3001
#define XMSTA               0x3002
#define XMASTER             0x3003
#define BCWAIT_TIME_LOW     0x3008
#define BCWAIT_TIME_HIGH    0x3009
#define CPWAIT_TIME_LOW     0x300A
#define CPWAIT_TIME_HIGH    0x300B
#define SECOND_SLAVE_ADD    0x300C
#define WINMODE             0x301C
#define HADD                0x3020
#define VADD                0x3021
#define ADDMODE             0x3022
#define VMAX_LOW            0x3024
#define VMAX_MID            0x3025
#define VMAX_HIGH           0x3026
#define HMAX_LOW            0x3028
#define HMAX_HIGH           0x3029
#define VREVERSE_HREVERSE   0x3030
#define ADBIT               0x3031
#define MDBIT               0x3032
#define SYS_MODE            0x3033
#define PIX_HST_LOW         0x3040
#define PIX_HST_HIGH        0x3041
#define PIX_HWIDTH_LOW      0x3042
#define PIX_HWIDTH_HIGH     0x3043
#define PIX_VST_LOW         0x3044
#define PIX_VST_HIGH        0x3045
#define PIX_VWIDTH_LOW      0x3046
#define PIX_VWIDTH_HIGH     0x3047
#define SHR0_LOW            0x3050
#define SHR0_MID            0x3051
#define SHR0_HIGH           0x3052
#define GAIN_PCG_0_LOW      0x3090
#define GAIN_PCG_0_HIGH     0x3091
#define XHSOUTSEL_XVSOUTSEL 0x30C0
#define XVS_XHS_DRV         0x30C1
#define XVSLNG              0x30CC
#define XHSLNG              0x30CD
#define EXTMODE             0x30CE
#define DIG_CLP_VSTART      0x30D9
#define DIG_VLP_VNUM        0x30DA
#define BLKLEVEL_LOW        0x30E2
#define BLKLEVEL_HIGH       0x30E3
#define INCKSEL1            0x3115
#define INCKSEL2            0x3116
#define INCKSEL3_LOW        0x3118
#define INCKSEL3_HIGH       0x3119
#define INCKSEL4_LOW        0x311A
#define INCKSEL4_HIGH       0x311B
#define INCKSEL5            0x311E
#define IMX415_REG_35A0     0x35A0
#define ADBIT1              0x3701
#define LANEMODE            0x4001
#define TXCLKESC_FREQ_LOW   0x4004
#define TXCLKESC_FREQ_HIGH  0x4005
#define INCKSEL6            0x400C

#define TCLKPOST_LOW        0x4018
#define TCLKPOST_HIGH       0x4019
#define TCLKPREPARE_LOW     0x401A
#define TCLKPREPARE_HIGH    0x401B
#define TCLKTRAIL_LOW       0x401C
#define TCLKTRAIL_HIGH      0x401D
#define TCLKZERO_LOW        0x401E
#define TCLKZERO_HIGH       0x401F
#define THSPREPARE_LOW      0x4020
#define THSPREPARE_HIGH     0x4021
#define THSZERO_LOW         0x4022
#define THSZERO_HIGH        0x4023
#define THSTRAIL_LOW        0x4024
#define THSTRAIL_HIGH       0x4025
#define THSEXIT_LOW         0x4026
#define THSEXIT_HIGH        0x4027
#define TLPX_LOW            0x4028
#define TLPX_HIGH           0x4029
#define INCKSEL7            0x4074

#define TPG_EN_DUOUT        0x30E4
#define TPG_PATSEL_DUOUT    0x30E6
#define TPG_COLORWIDTH      0x30E8
#define DIG_CLP_MODE        0x32C8
#define WRJ_OPEN            0x3390
#define TESTCLKEN_MIPI      0x3110

/**
 * Default resolution
 */
#define IMX415_DEFAULT_WIDTH    3864
#define IMX415_DEFAULT_HEIGHT   2192

#define IMX415_CROP_1296x720_WIDTH 1296
#define IMX415_CROP_1296x720_HEIGHT 720
#define IMX415_MODE_BINNING_H2V2_WIDTH 1920
#define IMX415_MODE_BINNING_H2V2_HEIGHT 1080


/**
 * Special values for the write table function
 */
#define IMX415_TABLE_WAIT_MS    0
#define IMX415_TABLE_END        1
#define IMX415_WAIT_MS          10

#define IMX415_MIN_FRAME_LENGTH_DELTA  58

#define IMX415_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX415_TO_MID_BYTE(x) (x>>8)

typedef struct reg_8 imx415_reg;

/**
 * Tables for the write table function
 */

static const imx415_reg imx415_start[] = {

    {STANDBY,              0x00},
    /* After standby canceled - internal regulator stabilization 24 ms or more */
    {IMX415_TABLE_WAIT_MS, 30},
    {XMSTA,                0x00},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};


static const imx415_reg imx415_stop[] = {

    {XMSTA,                0x01},
    {IMX415_TABLE_WAIT_MS, 30},
    {STANDBY,              0x01},

    {HADD,                 0x00},
    {VADD,                 0x00},
    {DIG_CLP_VSTART,       0x06},
    {DIG_VLP_VNUM,         0x02},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

static const imx415_reg imx415_10bit_mode[] = {

    {ADBIT,                0x00},
    {ADBIT1,               0x00},
    {MDBIT,                0x00},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

static const imx415_reg imx415_12bit_mode[] = {

    {ADBIT,                0x01},
    {ADBIT1,               0x03},
    {MDBIT,                0x01},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

/**
 * 1440Mbps and 720Mbps data rates require INCK 24 MHz
 * Other data rates require INCK 37.125 MHz
 */

static const imx415_reg imx415_2079_data_rate[] = {
    {TCLKPOST_LOW,         0xD7},
    {TCLKPREPARE_LOW,      0x7F},
    {TCLKTRAIL_LOW,        0x7F},
    {TCLKZERO_HIGH,        0x02},
    {TCLKZERO_LOW,         0x37},
    {THSPREPARE_LOW,       0x87},
    {THSZERO_LOW,          0xEF},
    {THSTRAIL_LOW,         0x87},
    {THSEXIT_LOW,          0xDF},
    {TLPX_LOW,             0x6F},

    /* INCK = 37.125Mhz */
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x24},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xE0},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xE0},
    {INCKSEL5,             0x24},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x01},
    {INCKSEL7,             0x00},

    {SYS_MODE,             0x02},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_1782_data_rate[] = {
    {TCLKPOST_LOW,         0xB7},
    {TCLKPREPARE_LOW,      0x67},
    {TCLKTRAIL_LOW,        0x6F},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0xDF},
    {THSPREPARE_LOW,       0x6F},
    {THSZERO_LOW,          0xCF},
    {THSTRAIL_LOW,         0x6F},
    {THSEXIT_LOW,          0xB7},
    {TLPX_LOW,             0x5F},

    /* INCK = 37.125Mhz */
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x24},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xC0},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xE0},
    {INCKSEL5,             0x24},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x01},
    {INCKSEL7,             0x00},

    {SYS_MODE,             0x04},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_1485_data_rate[] = {
    {TCLKPOST_LOW,         0x07},
    {TCLKPREPARE_LOW,      0x57},
    {TCLKTRAIL_LOW,        0x5F},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0x97},
    {THSPREPARE_LOW,       0x5F},
    {THSZERO_LOW,          0xAF},
    {THSTRAIL_LOW,         0x5F},
    {THSEXIT_LOW,          0x9F},
    {TLPX_LOW,             0x4F},

    /* INCK = 37.125Mhz */
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x24},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xA0},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xE0},
    {INCKSEL5,             0x24},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x01},
    {INCKSEL7,             0x00},

    {SYS_MODE,             0x08},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_1440_data_rate[] = {
    {TCLKPOST_LOW,         0x9F},
    {TCLKPREPARE_LOW,      0x57},
    {TCLKTRAIL_LOW,        0x57},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0x87},
    {THSPREPARE_LOW,       0x5F},
    {THSZERO_LOW,          0xA7},
    {THSTRAIL_LOW,         0x5F},
    {THSEXIT_LOW,          0x9F},
    {TLPX_LOW,             0x4F},

    /* INCK = 24Mhz */
    {BCWAIT_TIME_LOW,      0x54},
    {CPWAIT_TIME_LOW,      0x3B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x23},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xB4},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xFC},
    {INCKSEL5,             0x23},
    {TXCLKESC_FREQ_HIGH,   0x06},
    {TXCLKESC_FREQ_LOW,    0x00},
    {INCKSEL6,             0x01},
    {INCKSEL7,             0x00},

    {SYS_MODE,             0x08},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_891_data_rate[] = {
    {TCLKPOST_LOW,         0x7F},
    {TCLKPREPARE_LOW,      0x37},
    {TCLKTRAIL_LOW,        0x37},
    {TCLKZERO_HIGH,        0x00},
    {TCLKZERO_LOW,         0xF7},
    {THSPREPARE_LOW,       0x3F},
    {THSZERO_LOW,          0x6F},
    {THSTRAIL_LOW,         0x3F},
    {THSEXIT_LOW,          0x5F},
    {TLPX_LOW,             0x2F},

    /* INCK = 37.125Mhz */
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x24},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xC0},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xE0},
    {INCKSEL5,             0x24},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x00},
    {INCKSEL7,             0x01},

    {SYS_MODE,             0x05},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_720_data_rate[] = {
    {TCLKPOST_LOW,         0x6F},
    {TCLKPREPARE_LOW,      0x2F},
    {TCLKTRAIL_LOW,        0x2F},
    {TCLKZERO_HIGH,        0x00},
    {TCLKZERO_LOW,         0xBF},
    {THSPREPARE_LOW,       0x2F},
    {THSZERO_LOW,          0x57},
    {THSTRAIL_LOW,         0x2F},
    {THSEXIT_LOW,          0x4F},
    {TLPX_LOW,             0x27},

    /* INCK = 24Mhz */
    {BCWAIT_TIME_LOW,      0x54},
    {CPWAIT_TIME_LOW,      0x3B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x23},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xB4},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xFC},
    {INCKSEL5,             0x23},
    {TXCLKESC_FREQ_HIGH,   0x06},
    {TXCLKESC_FREQ_LOW,    0x00},
    {INCKSEL6,             0x00},
    {INCKSEL7,             0x01},

    {SYS_MODE,             0x09},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_594_data_rate[] = {
    {TCLKPOST_LOW,         0x67},
    {TCLKPREPARE_LOW,      0x27},
    {TCLKTRAIL_LOW,        0x27},
    {TCLKZERO_HIGH,        0x00},
    {TCLKZERO_LOW,         0xB7},
    {THSPREPARE_LOW,       0x2F},
    {THSZERO_LOW,          0x4F},
    {THSTRAIL_LOW,         0x2F},
    {THSEXIT_LOW,          0x47},
    {TLPX_LOW,             0x27},

    /* INCK = 37.125Mhz */
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x24},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0x80},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xE0},
    {INCKSEL5,             0x24},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x00},
    {INCKSEL7,             0x01},

    {SYS_MODE,             0x07},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_2376_data_rate[] = {
    {TCLKPOST_LOW,         0xE7},
    {TCLKPREPARE_LOW,      0x8F},
    {TCLKTRAIL_LOW,        0x8F},
    {TCLKZERO_HIGH,        0x02},
    {TCLKZERO_LOW,         0x7F},
    {THSPREPARE_LOW,       0x97},
    {THSZERO_HIGH,         0x01},
    {THSZERO_LOW,          0x0F},
    {THSTRAIL_LOW,         0x97},
    {THSEXIT_LOW,          0xF7},
    {TLPX_LOW,             0x7F},

    /* INCK = 37.125Mhz */
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x00},
    {INCKSEL2,             0x24},
    {INCKSEL3_HIGH,        0x01},
    {INCKSEL3_LOW,         0x00},
    {INCKSEL4_HIGH,        0x00},
    {INCKSEL4_LOW,         0xE0},
    {INCKSEL5,             0x24},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x01},
    {INCKSEL7,             0x00},

    {SYS_MODE,              0x00},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg imx415_init_settings[] = {

    {LANEMODE,             0x03},
    {XMASTER,              0x00},
    {VREVERSE_HREVERSE,    0x00},
    {IMX415_REG_35A0,      0x38},
    {WINMODE,              0x00},
    {ADDMODE,              0x00},
    {HADD,                 0x00},
    {VADD,                 0x00},
    {DIG_CLP_VSTART,       0x06},
    {DIG_VLP_VNUM,         0x02},

    {0x32D4,               0x21},
    {0x32EC,               0xA1},
    {0x3452,               0x7F},
    {0x3453,               0x03},

    {0x358A,               0x04},
    {0x35A1,               0x02},
    {0x36BC,               0x0C},
    {0x36CC,               0x53},
    {0x36CD,               0x00},
    {0x36CE,               0x3C},
    {0x36D0,               0x8C},
    {0x36D1,               0x00},
    {0x36D2,               0x71},
    {0x36D4,               0x3C},
    {0x36D6,               0x53},
    {0x36D7,               0x00},
    {0x36D8,               0x71},
    {0x36DA,               0x8C},
    {0x36DB,               0x00},
    {0x3724,               0x02},
    {0x3726,               0x02},
    {0x3732,               0x02},
    {0x3734,               0x03},
    {0x3736,               0x03},
    {0x3742,               0x03},
    {0x3862,               0xE0},
    {0x38CC,               0x30},
    {0x38CD,               0x2F},
    {0x395C,               0x0C},
    {0x3A42,               0xD1},
    {0x3A4C,               0x77},
    {0x3AE0,               0x02},
    {0x3AEC,               0x0C},
    {0x3B00,               0x2E},
    {0x3B06,               0x29},
    {0x3B98,               0x25},
    {0x3B99,               0x21},
    {0x3B9B,               0x13},
    {0x3B9C,               0x13},
    {0x3B9D,               0x13},
    {0x3B9E,               0x13},
    {0x3BA1,               0x00},
    {0x3BA2,               0x06},
    {0x3BA3,               0x0B},
    {0x3BA4,               0x10},
    {0x3BA5,               0x14},
    {0x3BA6,               0x18},
    {0x3BA7,               0x1A},
    {0x3BA8,               0x1A},
    {0x3BA9,               0x1A},
    {0x3BAC,               0xED},
    {0x3BAD,               0x01},
    {0x3BAE,               0xF6},
    {0x3BAF,               0x02},
    {0x3BB0,               0xA2},
    {0x3BB1,               0x03},
    {0x3BB2,               0xE0},
    {0x3BB3,               0x03},
    {0x3BB4,               0xE0},
    {0x3BB5,               0x03},
    {0x3BB6,               0xE0},
    {0x3BB7,               0x03},
    {0x3BB8,               0xE0},
    {0x3BBA,               0xE0},
    {0x3BBC,               0xDA},
    {0x3BBE,               0x88},
    {0x3BC0,               0x44},
    {0x3BC2,               0x7B},
    {0x3BC4,               0xA2},
    {0x3BC8,               0xBD},
    {0x3BCA,               0xBD},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}

};

static const imx415_reg mode_3864x2192[] = {
    {WINMODE,              0x00},
    {ADDMODE,              0x00},
    {HADD,                 0x00},
    {VADD,                 0x00},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

static const imx415_reg mode_H2V2_binning[] = {
    {WINMODE,              0x04},
    {ADDMODE,              0x01},
    {HADD,                 0x01},
    {VADD,                 0x01},
    {ADBIT,                0x00},
    {MDBIT,                0x01},

    {DIG_CLP_VSTART,       0x02},
    {DIG_VLP_VNUM,         0x01},   

    {PIX_HST_HIGH,         IMX415_TO_MID_BYTE(12)},
    {PIX_HST_LOW,          IMX415_TO_LOW_BYTE(12)},
    {PIX_HWIDTH_HIGH,      IMX415_TO_MID_BYTE(3816)},
    {PIX_HWIDTH_LOW,       IMX415_TO_LOW_BYTE(3816)}, // -1 pix

    {PIX_VST_HIGH,         IMX415_TO_MID_BYTE(32)},
    {PIX_VST_LOW,          IMX415_TO_LOW_BYTE(32)},
    {PIX_VWIDTH_HIGH,      IMX415_TO_MID_BYTE(4316)},
    {PIX_VWIDTH_LOW,       IMX415_TO_LOW_BYTE(4316)}, //-4 (1 line)

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

static const imx415_reg mode_crop_1296x720[] = {
    {WINMODE,              0x04},

    {PIX_HST_HIGH,         IMX415_TO_MID_BYTE(1284)},
    {PIX_HST_LOW,          IMX415_TO_LOW_BYTE(1284)},
    {PIX_HWIDTH_HIGH,      IMX415_TO_MID_BYTE(IMX415_CROP_1296x720_WIDTH)},
    {PIX_HWIDTH_LOW,       IMX415_TO_LOW_BYTE(IMX415_CROP_1296x720_WIDTH)},

    {PIX_VST_HIGH,         IMX415_TO_MID_BYTE(1472)},
    {PIX_VST_LOW,          IMX415_TO_LOW_BYTE(1472)},
    {PIX_VWIDTH_HIGH,      IMX415_TO_MID_BYTE(IMX415_CROP_1296x720_HEIGHT*2)},
    {PIX_VWIDTH_LOW,       IMX415_TO_LOW_BYTE(IMX415_CROP_1296x720_HEIGHT*2)},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

static const imx415_reg mode_enable_pattern_generator[] = {
    {DIG_CLP_MODE,         0x00},
    {TPG_EN_DUOUT,         0x01},
    {TPG_COLORWIDTH,       0x02},
    {WRJ_OPEN,             0x00},
    {TESTCLKEN_MIPI,       (0x01 << 5)},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

static const imx415_reg mode_disable_pattern_generator[] = {
    {DIG_CLP_MODE,         0x01},
    {TPG_EN_DUOUT,         0x00},
    {TPG_COLORWIDTH,       0x00},
    {WRJ_OPEN,             0x01},
    {TESTCLKEN_MIPI,       0x00},

    {IMX415_TABLE_WAIT_MS, IMX415_WAIT_MS},
    {IMX415_TABLE_END,     0x00}
};

/**
 * Enum of available frame modes
 */

enum {
    /* All pixel mode */
    IMX415_MODE_3864x2192,

    /* 16:9 frame modes */
    IMX415_MODE_crop_1296x720,

    /* binning frame modes */
    IMX415_MODE_H2V2_BINNING,

    IMX415_10BIT_MODE,
    IMX415_12BIT_MODE,

    IMX415_EN_PATTERN_GEN,
    IMX415_DIS_PATTERN_GEN,

    IMX415_INIT_SETTINGS,
    IMX415_MODE_START_STREAM,
    IMX415_MODE_STOP_STREAM,

};

enum {
    IMX415_2376_DATA_RATE,
    IMX415_2079_DATA_RATE,
    IMX415_1782_DATA_RATE,
    IMX415_1485_DATA_RATE,
    IMX415_1440_DATA_RATE,
    IMX415_891_DATA_RATE,
    IMX415_720_DATA_RATE,
    IMX415_594_DATA_RATE,
};

static const imx415_reg *data_rate_table[] = {

    [IMX415_2376_DATA_RATE] = imx415_2376_data_rate,
    [IMX415_2079_DATA_RATE] = imx415_2079_data_rate,
    [IMX415_1782_DATA_RATE] = imx415_1782_data_rate,
    [IMX415_1485_DATA_RATE] = imx415_1485_data_rate,
    [IMX415_1440_DATA_RATE] = imx415_1440_data_rate,
    [IMX415_891_DATA_RATE]  = imx415_891_data_rate,
    [IMX415_720_DATA_RATE]  = imx415_720_data_rate,
    [IMX415_594_DATA_RATE]  = imx415_594_data_rate,
};

/**
 * Connecting frame modes to mode tables
 */

static const imx415_reg *mode_table[] = {

    [IMX415_MODE_3864x2192]         = mode_3864x2192,
    [IMX415_MODE_crop_1296x720]     = mode_crop_1296x720,

    [IMX415_MODE_H2V2_BINNING]      = mode_H2V2_binning,

    [IMX415_EN_PATTERN_GEN]         = mode_enable_pattern_generator,
    [IMX415_DIS_PATTERN_GEN]        = mode_disable_pattern_generator,

    [IMX415_10BIT_MODE]             = imx415_10bit_mode,
    [IMX415_12BIT_MODE]             = imx415_12bit_mode,

    [IMX415_INIT_SETTINGS]          = imx415_init_settings,

    [IMX415_MODE_START_STREAM]      = imx415_start,
    [IMX415_MODE_STOP_STREAM]       = imx415_stop,
};

/**
 * Framerates of available frame modes
 */

static const int imx415_90fps[] = {
    90,
};
static const int imx415_260fps[] = {
    260,
};

/**
 * Connecting resolutions, framerates and mode tables
 */

static const struct camera_common_frmfmt imx415_frmfmt[] = {
    {
        .size = {IMX415_DEFAULT_WIDTH, IMX415_DEFAULT_HEIGHT},
        .framerates = imx415_90fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX415_MODE_3864x2192
    },
    {
        .size =  {IMX415_CROP_1296x720_WIDTH, IMX415_CROP_1296x720_HEIGHT},
        .framerates = imx415_260fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX415_MODE_crop_1296x720
    },
    {
        .size = {IMX415_MODE_BINNING_H2V2_WIDTH, IMX415_MODE_BINNING_H2V2_HEIGHT},
        .framerates = imx415_90fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX415_MODE_H2V2_BINNING
    },
};

#endif /* __IMX415_TABLES__ */
