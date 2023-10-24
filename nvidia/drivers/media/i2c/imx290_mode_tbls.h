/*
 * imx290_mode_tbls.h - imx290 sensor mode tables
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

#ifndef __IMX290_TABLES__
#define __IMX290_TABLES__

/**
 * Image sensor registers as described in the IMX290 register map
 */

#define STANDBY                          0x3000
#define REGHOLD                          0x3001
#define XMSTA                            0x3002
#define SW_RESET                         0x3003
#define ADBIT                            0x3005
#define WINMODE_HREVERSE_VREVERSE        0x3007
#define FDGSEL_FRSEL                     0x3009
#define BLKLEVEL_LOW                     0x300A
#define BLKLEVEL_HIGH                    0x300B
#define IMX290_0x300F                    0x300F
#define IMX290_0x3010                    0x3010
#define IMX290_0x3012                    0x3012
#define IMX290_0x3013                    0x3013
#define GAIN                             0x3014
#define IMX290_0x3016                    0x3016
#define VMAX_LOW                         0x3018
#define VMAX_MID                         0x3019
#define VMAX_HIGH                        0x301A
#define HMAX_LOW                         0x301C
#define HMAX_HIGH                        0x301D
#define SHS1_LOW                         0x3020
#define SHS1_MID                         0x3021
#define SHS1_HIGH                        0x3022
#define WINWV_OB                         0x303A
#define WINPV_LOW                        0x303C
#define WINPV_HIGH                       0x303D
#define WINWV_LOW                        0x303E
#define WINWV_HIGH                       0x303F
#define WINPH_LOW                        0x3040
#define WINPH_HIGH                       0x3041
#define WINWH_LOW                        0x3042
#define WINWH_HIGH                       0x3043
#define OPORTSEL_ODBIT                   0x3046
#define XVSLNG                           0x3048
#define XHSLNG                           0x3049
#define XHSOUTSEL_XVSOUTSEL              0x304B
#define INCKSEL1                         0x305C
#define INCKSEL2                         0x305D
#define INCKSEL3                         0x305E
#define INCKSEL4                         0x305F
#define IMX290_3070                      0x3070
#define IMX290_3071                      0x3071
#define IMX290_309B                      0x309B
#define IMX290_309C                      0x309C
#define IMX290_30A2                      0x30A2
#define IMX290_30A6                      0x30A6
#define IMX290_30A8                      0x30A8
#define IMX290_30AA                      0x30AA
#define IMX290_30AC                      0x30AC
#define IMX290_30B0                      0x30B0

#define IMX290_3119                      0x3119
#define IMX290_311C                      0x311C
#define IMX290_311E                      0x311E
#define IMX290_3128                      0x3128
#define ADBIT1                           0x3129
#define IMX290_313D                      0x313D
#define IMX290_3150                      0x3150
#define INCKSEL5                         0x315E
#define INCKSEL6                         0x3164
#define ADBIT2                           0x317C
#define IMX290_317E                      0x317E
#define ADBIT3                           0x31EC

#define IMX290_32B8                      0x32B8
#define IMX290_32B9                      0x32B9
#define IMX290_32BA                      0x32BA
#define IMX290_32BB                      0x32BB
#define IMX290_32C8                      0x32C8
#define IMX290_32C9                      0x32C9
#define IMX290_32CA                      0x32CA
#define IMX290_32CB                      0x32CB

#define IMX290_332C                      0x332C
#define IMX290_332D                      0x332D
#define IMX290_332E                      0x332E
#define IMX290_3358                      0x3358
#define IMX290_3359                      0x3359
#define IMX290_335A                      0x335A
#define IMX290_3360                      0x3360
#define IMX290_3361                      0x3361
#define IMX290_3362                      0x3362
#define IMX290_33B0                      0x33B0
#define IMX290_33B2                      0x33B2
#define IMX290_33B3                      0x33B3

#define REPETITION                       0x3405
#define PHYSICAL_LANE_NUM                0x3407
#define OPB_SIZE_V                       0x3414
#define Y_OUT_SIZE_LOW                   0x3418
#define Y_OUT_SIZE_HIGH                  0x3419
#define CSI_DT_FMT_LOW                   0x3441
#define CSI_DT_FMT_HIGH                  0x3442
#define CSI_LANE_MODE                    0x3443
#define EXTCK_FREQ_LOW                   0x3444
#define EXTCK_FREQ_HIGH                  0x3445
#define TCLKPOST_LOW                     0x3446
#define TCLKPOST_HIGH                    0x3447
#define THSZERO_LOW                      0x3448
#define THSZERO_HIGH                     0x3449
#define THSPREPARE_LOW                   0x344A
#define THSPREPARE_HIGH                  0x344B
#define TCLKTRAIL_LOW                    0x344C
#define TCLKTRAIL_HIGH                   0x344D
#define THSTRAIL_LOW                     0x344E
#define THSTRAIL_HIGH                    0x344F
#define TCLKZERO_LOW                     0x3450
#define TCLKZERO_HIGH                    0x3451
#define TCLKPREPARE_LOW                  0x3452
#define TCLKPREPARE_HIGH                 0x3453
#define TLPX_LOW                         0x3454
#define TLPX_HIGH                        0x3455
#define X_OUT_SIZE_LOW                   0x3472
#define X_OUT_SIZE_HIGH                  0x3473
#define INCKSEL7                         0x3480

#define PGMODE_COLORWIDTH_PGTHRU_PGREGEN 0x308C

/**
 * Resolutions of implemented frame modes
 */

#define IMX290_DEFAULT_WIDTH   1920
#define IMX290_DEFAULT_HEIGHT  1080
#define IMX290_1280x720_WIDTH  1280
#define IMX290_1280x720_HEIGHT 720
#define IMX290_640x480_WIDTH   640
#define IMX290_640x480_HEIGHT  480

/**
 * Special values for the write table function
 */
#define IMX290_TABLE_WAIT_MS 0
#define IMX290_TABLE_END     1
#define IMX290_WAIT_MS       10

/**
 * Minimal value of frame length is resolution height + 45
 * Minimal value for scaling modes is full pixel mode height + 45
 *
 * Determined from the default value of FRM_LENGTH_LINES register
 * and empirically confirmed
 */
#define IMX290_MIN_FRAME_DELTA_1080p    45
#define IMX290_MIN_FRAME_DELTA_720p     30
#define IMX290_MIN_FRAME_DELTA_CROP     25

typedef struct reg_8 imx290_reg;

/**
 * Tables for the write table function
 */

static const imx290_reg imx290_start[] = {

    {STANDBY,              0x00},
    {IMX290_TABLE_WAIT_MS, 30},
    {XMSTA,                0x00},

    {IMX290_TABLE_END,     0x00}
};


static const imx290_reg imx290_stop[] = {

    {STANDBY,              0x01},
    {IMX290_TABLE_WAIT_MS, 30},
    {XMSTA,                0x01},

    {IMX290_TABLE_WAIT_MS, 10},
    {IMX290_TABLE_END,     0x00}
};

static const imx290_reg imx290_10bit_mode[] = {

    {ADBIT,                0x00},
    {OPORTSEL_ODBIT,       0x00},
    {ADBIT1,               0x1D},
    {ADBIT2,               0x12},
    {ADBIT3,               0x37},
    {CSI_DT_FMT_LOW,       0x0A},
    {CSI_DT_FMT_HIGH,      0x0A},
    {BLKLEVEL_LOW,         60 & 0xFF},
    {BLKLEVEL_HIGH,        60 >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x00}
};

static const imx290_reg imx290_12bit_mode[] = {

    {ADBIT,                0x01},
    {OPORTSEL_ODBIT,       0x01},
    {ADBIT1,               0x00},
    {ADBIT2,               0x00},
    {ADBIT3,               0x0E},
    {CSI_DT_FMT_LOW,       0x0C},
    {CSI_DT_FMT_HIGH,      0x0C},
    {BLKLEVEL_LOW,         240 & 0xFF},
    {BLKLEVEL_HIGH,        240 >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x00}
};

static const imx290_reg imx290_init_settings[] = {

    {WINMODE_HREVERSE_VREVERSE, 0x00},
    {FDGSEL_FRSEL,              0x00},

    {VMAX_LOW,                  1125 & 0xFF},
    {VMAX_MID,                  1125 >> 8},
    {VMAX_HIGH,                 1125 >> 16},

    {CSI_LANE_MODE,             0x03},
    {EXTCK_FREQ_LOW,            0x2520 & 0xFF},
    {EXTCK_FREQ_HIGH,           0x2520 >> 8},
    {PHYSICAL_LANE_NUM,         0x03},
    {WINWV_OB,                  0x0C},

    {WINPH_LOW,                 0 & 0xFF},
    {WINPH_HIGH,                0 >> 8},
    {WINPV_LOW,                 0 & 0xFF},
    {WINPV_HIGH,                0 >> 8},
    {WINWH_LOW,                 1948 & 0xFF},
    {WINWH_HIGH,                1948 >> 8},
    {WINWV_LOW,                 1097 & 0xFF},
    {WINWV_HIGH,                1097 >> 8},

    {XHSOUTSEL_XVSOUTSEL,       0x0A},

    {IMX290_0x300F,             0x00},
    {IMX290_0x3010,             0x21},
    {IMX290_0x3012,             0x64},
    {IMX290_0x3016,             0x09},

    {IMX290_3070,               0x02},
    {IMX290_3071,               0x11},
    {IMX290_309B,               0x10},
    {IMX290_309C,               0x22},

    {IMX290_30A2,               0x02},
    {IMX290_30A6,               0x20},
    {IMX290_30A8,               0x20},
    {IMX290_30AA,               0x20},
    {IMX290_30AC,               0x20},

    {IMX290_30B0,               0x43},

    {IMX290_3119,               0x9E},
    {IMX290_311C,               0x1E},
    {IMX290_311E,               0x08},
    {IMX290_3128,               0x05},

    {IMX290_313D,               0x83},
    {IMX290_3150,               0x03},

    {IMX290_317E,               0x00},

    {IMX290_32B8,               0x50},
    {IMX290_32B9,               0x10},
    {IMX290_32BA,               0x00},
    {IMX290_32BB,               0x04},
    {IMX290_32C8,               0x50},
    {IMX290_32C9,               0x10},
    {IMX290_32CA,               0x00},
    {IMX290_32CB,               0x04},

    {IMX290_332C,               0xD3},
    {IMX290_332D,               0x10},
    {IMX290_332E,               0x0D},
    {IMX290_3358,               0x06},
    {IMX290_3359,               0xE1},
    {IMX290_335A,               0x11},
    {IMX290_3360,               0x1E},
    {IMX290_3361,               0x61},
    {IMX290_3362,               0x10},
    {IMX290_33B0,               0x50},
    {IMX290_33B2,               0x1A},
    {IMX290_33B3,               0x04},

    {IMX290_TABLE_WAIT_MS,      IMX290_WAIT_MS},
    {IMX290_TABLE_END,          0x0000}
};

static const imx290_reg mode_1920x1080[] = {

    {WINMODE_HREVERSE_VREVERSE, 0x00},
    {WINWV_OB,                  0x0C},
    {OPB_SIZE_V,                0x0A},
    {X_OUT_SIZE_LOW,            IMX290_DEFAULT_WIDTH & 0xFF},
    {X_OUT_SIZE_HIGH,           IMX290_DEFAULT_WIDTH >> 8},
    {Y_OUT_SIZE_LOW,            IMX290_DEFAULT_HEIGHT & 0xFF},
    {Y_OUT_SIZE_HIGH,           IMX290_DEFAULT_HEIGHT >> 8},

    {IMX290_0x3012,             0x64},
    {IMX290_0x3013,             0x00},

    {INCKSEL1,                  0x18},
    {INCKSEL2,                  0x03},
    {INCKSEL3,                  0x20},
    {INCKSEL4,                  0x01},
    {INCKSEL5,                  0x1A},
    {INCKSEL6,                  0x1A},
    {INCKSEL7,                  0x49},

    {IMX290_TABLE_WAIT_MS,      IMX290_WAIT_MS},
    {IMX290_TABLE_END,          0x0000}
};

static const imx290_reg mode_1280x720[] = {

    {WINMODE_HREVERSE_VREVERSE, 0x10},
    {WINWV_OB,                  0x06},
    {OPB_SIZE_V,                0x04},
    {X_OUT_SIZE_LOW,            IMX290_1280x720_WIDTH & 0xFF},
    {X_OUT_SIZE_HIGH,           IMX290_1280x720_WIDTH >> 8},
    {Y_OUT_SIZE_LOW,            IMX290_1280x720_HEIGHT & 0xFF},
    {Y_OUT_SIZE_HIGH,           IMX290_1280x720_HEIGHT >> 8},

    {IMX290_0x3012,             0x64},
    {IMX290_0x3013,             0x00},

    {INCKSEL1,                  0x20},
    {INCKSEL2,                  0x00},
    {INCKSEL3,                  0x20},
    {INCKSEL4,                  0x01},
    {INCKSEL5,                  0x1A},
    {INCKSEL6,                  0x1A},
    {INCKSEL7,                  0x49},

    {IMX290_TABLE_WAIT_MS,      IMX290_WAIT_MS},
    {IMX290_TABLE_END,          0x0000}
};

static const imx290_reg mode_crop_640x480[] = {

    {WINMODE_HREVERSE_VREVERSE, 0x40},

    {IMX290_0x3012,             0x64},
    {IMX290_0x3013,             0x00},

    {WINPH_LOW,                 640 & 0xFF},
    {WINPH_HIGH,                640 >> 8},
    {WINPV_LOW,                 300 & 0xFF},
    {WINPV_HIGH,                300 >> 8},
    {WINWH_LOW,                 656 & 0xFF},
    {WINWH_HIGH,                656 >> 8},
    {WINWV_LOW,                 496 & 0xFF},
    {WINWV_HIGH,                496 >> 8},
    {WINWV_OB,                  0x0C},
    {OPB_SIZE_V,                0x0A},
    {X_OUT_SIZE_LOW,            IMX290_640x480_WIDTH & 0xFF},
    {X_OUT_SIZE_HIGH,           IMX290_640x480_WIDTH >> 8},
    {Y_OUT_SIZE_LOW,            IMX290_640x480_HEIGHT & 0xFF},
    {Y_OUT_SIZE_HIGH,           IMX290_640x480_HEIGHT >> 8},

    {INCKSEL1,                  0x18},
    {INCKSEL2,                  0x03},
    {INCKSEL3,                  0x20},
    {INCKSEL4,                  0x01},
    {INCKSEL5,                  0x1A},
    {INCKSEL6,                  0x1A},
    {INCKSEL7,                  0x49},

    {IMX290_TABLE_WAIT_MS,      IMX290_WAIT_MS},
    {IMX290_TABLE_END,          0x0000}
};

static const imx290_reg imx290_222_75_mbps[] = {

    {FDGSEL_FRSEL,         0x02},
    {REPETITION,           0x20},

    {TCLKPOST_LOW,         0x47 & 0xFF},
    {TCLKPOST_HIGH,        0x47 >> 8},
    {THSZERO_LOW,          0x1F & 0xFF},
    {THSZERO_HIGH,         0x1F >> 8},
    {THSPREPARE_LOW,       0x17 & 0xFF},
    {THSPREPARE_HIGH,      0x17 >> 8},
    {TCLKTRAIL_LOW,        0x0F & 0xFF},
    {TCLKTRAIL_HIGH,       0x0F >> 8},
    {THSTRAIL_LOW,         0x17 & 0xFF},
    {THSTRAIL_HIGH,        0x17 >> 8},
    {TCLKZERO_LOW,         0x47 & 0xFF},
    {TCLKZERO_HIGH,        0x47 >> 8},
    {TCLKPREPARE_LOW,      0x0F & 0xFF},
    {TCLKPREPARE_HIGH,     0x0F >> 8},
    {TLPX_LOW,             0x0F & 0xFF},
    {TLPX_HIGH,            0x0F >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg imx290_445_5_mbps[] = {

    {FDGSEL_FRSEL,         0x01},
    {REPETITION,           0x10},

    {TCLKPOST_LOW,         0x57 & 0xFF},
    {TCLKPOST_HIGH,        0x57 >> 8},
    {THSZERO_LOW,          0x37 & 0xFF},
    {THSZERO_HIGH,         0x37 >> 8},
    {THSPREPARE_LOW,       0x1F & 0xFF},
    {THSPREPARE_HIGH,      0x1F >> 8},
    {TCLKTRAIL_LOW,        0x1F & 0xFF},
    {TCLKTRAIL_HIGH,       0x1F >> 8},
    {THSTRAIL_LOW,         0x1F & 0xFF},
    {THSTRAIL_HIGH,        0x1F >> 8},
    {TCLKZERO_LOW,         0x77 & 0xFF},
    {TCLKZERO_HIGH,        0x77 >> 8},
    {TCLKPREPARE_LOW,      0x1F & 0xFF},
    {TCLKPREPARE_HIGH,     0x1F >> 8},
    {TLPX_LOW,             0x17 & 0xFF},
    {TLPX_HIGH,            0x17 >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg imx290_891_mbps[] = {

    {FDGSEL_FRSEL,         0x00},
    {REPETITION,           0x00},

    {TCLKPOST_LOW,         0x77 & 0xFF},
    {TCLKPOST_HIGH,        0x77 >> 8},
    {THSZERO_LOW,          0x67 & 0xFF},
    {THSZERO_HIGH,         0x67 >> 8},
    {THSPREPARE_LOW,       0x47 & 0xFF},
    {THSPREPARE_HIGH,      0x47 >> 8},
    {TCLKTRAIL_LOW,        0x37 & 0xFF},
    {TCLKTRAIL_HIGH,       0x37 >> 8},
    {THSTRAIL_LOW,         0x3F & 0xFF},
    {THSTRAIL_HIGH,        0x3F >> 8},
    {TCLKZERO_LOW,         0xFF & 0xFF},
    {TCLKZERO_HIGH,        0xFF >> 8},
    {TCLKPREPARE_LOW,      0x3F & 0xFF},
    {TCLKPREPARE_HIGH,     0x3F >> 8},
    {TLPX_LOW,             0x37 & 0xFF},
    {TLPX_HIGH,            0x37 >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg imx290_148_5_mbps[] = {

    {FDGSEL_FRSEL,         0x02},
    {REPETITION,           0x20},

    {TCLKPOST_LOW,         0x47 & 0xFF},
    {TCLKPOST_HIGH,        0x47 >> 8},
    {THSZERO_LOW,          0x17 & 0xFF},
    {THSZERO_HIGH,         0x17 >> 8},
    {THSPREPARE_LOW,       0x0F & 0xFF},
    {THSPREPARE_HIGH,      0x0F >> 8},
    {TCLKTRAIL_LOW,        0x0F & 0xFF},
    {TCLKTRAIL_HIGH,       0x0F >> 8},
    {THSTRAIL_LOW,         0x0F & 0xFF},
    {THSTRAIL_HIGH,        0x0F >> 8},
    {TCLKZERO_LOW,         0x2B & 0xFF},
    {TCLKZERO_HIGH,        0x2B >> 8},
    {TCLKPREPARE_LOW,      0x0B & 0xFF},
    {TCLKPREPARE_HIGH,     0x0B >> 8},
    {TLPX_LOW,             0x0F & 0xFF},
    {TLPX_HIGH,            0x0F >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg imx290_297_mbps[] = {

    {FDGSEL_FRSEL,         0x01},
    {REPETITION,           0x10},

    {TCLKPOST_LOW,         0x4F & 0xFF},
    {TCLKPOST_HIGH,        0x4F >> 8},
    {THSZERO_LOW,          0x2F & 0xFF},
    {THSZERO_HIGH,         0x2F >> 8},
    {THSPREPARE_LOW,       0x17 & 0xFF},
    {THSPREPARE_HIGH,      0x17 >> 8},
    {TCLKTRAIL_LOW,        0x17 & 0xFF},
    {TCLKTRAIL_HIGH,       0x17 >> 8},
    {THSTRAIL_LOW,         0x17 & 0xFF},
    {THSTRAIL_HIGH,        0x17 >> 8},
    {TCLKZERO_LOW,         0x57 & 0xFF},
    {TCLKZERO_HIGH,        0x57 >> 8},
    {TCLKPREPARE_LOW,      0x17 & 0xFF},
    {TCLKPREPARE_HIGH,     0x17 >> 8},
    {TLPX_LOW,             0x17 & 0xFF},
    {TLPX_HIGH,            0x17 >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg imx290_594_mbps[] = {

    {FDGSEL_FRSEL,         0x00},
    {REPETITION,           0x00},

    {TCLKPOST_LOW,         0x67 & 0xFF},
    {TCLKPOST_HIGH,        0x67 >> 8},
    {THSZERO_LOW,          0x57 & 0xFF},
    {THSZERO_HIGH,         0x57 >> 8},
    {THSPREPARE_LOW,       0x2F & 0xFF},
    {THSPREPARE_HIGH,      0x2F >> 8},
    {TCLKTRAIL_LOW,        0x27 & 0xFF},
    {TCLKTRAIL_HIGH,       0x27 >> 8},
    {THSTRAIL_LOW,         0x2F & 0xFF},
    {THSTRAIL_HIGH,        0x2F >> 8},
    {TCLKZERO_LOW,         0xBF & 0xFF},
    {TCLKZERO_HIGH,        0xBF >> 8},
    {TCLKPREPARE_LOW,      0x2F & 0xFF},
    {TCLKPREPARE_HIGH,     0x2F >> 8},
    {TLPX_LOW,             0x27 & 0xFF},
    {TLPX_HIGH,            0x27 >> 8},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg mode_enable_pattern_generator[] = {

    {BLKLEVEL_LOW,         0x00},
    {BLKLEVEL_HIGH,        0x00},

    {IMX290_TABLE_WAIT_MS, IMX290_WAIT_MS},
    {IMX290_TABLE_END,     0x0000}
};

static const imx290_reg mode_disable_pattern_generator[] = {

    {PGMODE_COLORWIDTH_PGTHRU_PGREGEN, 0x00},

    {IMX290_TABLE_WAIT_MS,             IMX290_WAIT_MS},
    {IMX290_TABLE_END,                 0x0000}
};

/**
 * Enum describing available data rate modes
 */
typedef enum {
    IMX290_HIGH_DATA_RATE,
    IMX290_MID_DATA_RATE,
    IMX290_LOW_DATA_RATE,
} data_rate_mode;

/**
 * Enum of available frame modes
 */

enum {

    IMX290_MODE_1920x1080,
    IMX290_MODE_1280x720,
    IMX290_MODE_crop_640x480,

    IMX290_EN_PATTERN_GEN,
    IMX290_DIS_PATTERN_GEN,

    IMX290_10BIT_MODE,
    IMX290_12BIT_MODE,

    IMX290_INIT_SETTINGS,
    IMX290_MODE_START_STREAM,
    IMX290_MODE_STOP_STREAM,
};


static const imx290_reg *data_rate_table[] = {
    [IMX290_HIGH_DATA_RATE] = imx290_891_mbps,
    [IMX290_MID_DATA_RATE]  = imx290_445_5_mbps,
    [IMX290_LOW_DATA_RATE]  = imx290_222_75_mbps,
};

static const imx290_reg *data_rate_table_720p[] = {
    [IMX290_HIGH_DATA_RATE] = imx290_594_mbps,
    [IMX290_MID_DATA_RATE]  = imx290_297_mbps,
    [IMX290_LOW_DATA_RATE]  = imx290_148_5_mbps,
};

/**
 * Connecting frame modes to mode tables
 */

static const imx290_reg *mode_table[] = {

    [IMX290_MODE_1920x1080]    = mode_1920x1080,
    [IMX290_MODE_1280x720]     = mode_1280x720,
    [IMX290_MODE_crop_640x480] = mode_crop_640x480,

    [IMX290_EN_PATTERN_GEN]    = mode_enable_pattern_generator,
    [IMX290_DIS_PATTERN_GEN]   = mode_disable_pattern_generator,

    [IMX290_10BIT_MODE]        = imx290_10bit_mode,
    [IMX290_12BIT_MODE]        = imx290_12bit_mode,

    [IMX290_INIT_SETTINGS]     = imx290_init_settings,

    [IMX290_MODE_START_STREAM] = imx290_start,
    [IMX290_MODE_STOP_STREAM]  = imx290_stop,
};

/**
 * Framerates of available frame modes
 */

static const int imx290_120fps[] = {
    120,
};
static const int imx290_267fps[] = {
    267,
};

/**
 * Connecting resolutions, framerates and mode tables
 */

static const struct camera_common_frmfmt imx290_frmfmt[] = {
    {
        .size = {IMX290_DEFAULT_WIDTH, IMX290_DEFAULT_HEIGHT},
        .framerates = imx290_120fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX290_MODE_1920x1080
    },
    {
        .size = {IMX290_1280x720_WIDTH, IMX290_1280x720_HEIGHT},
        .framerates = imx290_120fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX290_MODE_1280x720
    },
    {
        .size = {IMX290_640x480_WIDTH, IMX290_640x480_HEIGHT},
        .framerates = imx290_267fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX290_MODE_crop_640x480
    },

};

#endif /* __IMX290_TABLES__ */
