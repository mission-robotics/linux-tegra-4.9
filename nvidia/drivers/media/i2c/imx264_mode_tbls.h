/*
 * imx264_mode_tbls.h - imx264 sensor mode tables
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

#ifndef __IMX264_TABLES__
#define __IMX264_TABLES__

/**
 * Image sensor registers as described in the imx264 register map
 */

#define STANDBY             0x3000
#define REGHOLD             0x3008
#define XMSTA               0x300A

#define HMODE_WINMODE       0x300D
#define HREVERSE_VREVERSE   0x300E


#define VMAX_LOW            0x3010
#define VMAX_MID            0x3011
#define VMAX_HIGH           0x3012

#define HMAX_LOW            0x3014
#define HMAX_HIGH           0x3015
#define CKSEL               0x3019

#define SYNCSEL             0x3036

#define INCKSEL0            0x3089
#define INCKSEL1            0x308A
#define INCKSEL2            0x308B
#define INCKSEL3            0x308C

#define SHS1_LOW            0x308D
#define SHS1_MID            0x308E
#define SHS1_HIGH           0x308F

#define GTWAIT              0x309E
#define GSDLY               0x30A0
#define BLKLEVEL_LOW        0x3254
#define BLKLEVEL_HIGH       0x3255

#define GAIN_LOW            0x3204
#define GAINDLY             0x3212
#define TRIGEN              0x300B

#define FID0_ROI            0x3300
#define FID0_ROIPH1_LOW     0x3310
#define FID0_ROIPH1_HIGH    0x3311 
#define FID0_ROIPV1_LOW     0x3312
#define FID0_ROIPV1_HIGH    0x3313
#define FID0_ROIWH1_LOW     0x3314
#define FID0_ROIWH1_HIGH    0x3315 
#define FID0_ROIWV1_LOW     0x3316
#define FID0_ROIWV1_HIGH    0x3317
     
#define VINT_EN             0x30AA

#define TOUT1SEL            0x3026
#define TRIG_TOUT1_SEL      0x3029
#define PULSE1_SETTINGS     0x306D
#define PULSE1_UP_LOW       0x3070
#define PULSE1_UP_MID       0x3071
#define PULSE1_UP_HIGH      0x3072
#define PULSE1_DN_LOW       0x3074
#define PULSE1_DN_MID       0x3075
#define PULSE1_DN_HIGH      0x3076

#define PULSE2_SETTINGS     0x3079
#define PULSE2_UP_LOW       0x307C
#define PULSE2_UP_MID       0x307D
#define PULSE2_UP_HIGH      0x307E
#define PULSE2_DN_LOW       0x3080
#define PULSE2_DN_MID       0x3081
#define PULSE2_DN_HIGH      0x3082

/**
 * Resolutions of implemented frame modes
 */

#define IMX264_DEFAULT_WIDTH        2464
#define IMX264_DEFAULT_HEIGHT       2056

#define IMX264_ROI_MODE_WIDTH       2048
#define IMX264_ROI_MODE_HEIGHT      1536

#define IMX264_1080p_MODE_WIDTH     1936
#define IMX264_1080p_MODE_HEIGHT    1096

/**
 * Special values for the write table function
 */
#define IMX264_TABLE_WAIT_MS 0
#define IMX264_TABLE_END     1
#define IMX264_WAIT_MS       10

/**
 * Minimal value of frame length is resolution height + 45
 * Minimal value for scaling modes is full pixel mode height + 45
 *
 * Determined from the default value of FRM_LENGTH_LINES register
 * and empirically confirmed
 */
#define IMX264_MIN_FRAME_LENGTH_DELTA  32

#define IMX264_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX264_TO_MID_BYTE(x) (x >> 8)

typedef struct reg_8 imx264_reg;
typedef struct reg_8 crosslink_reg;

/**
 * Tables for the write table function
 */

static const imx264_reg imx264_stop[] = {

    {STANDBY,              0x01},
    {IMX264_TABLE_WAIT_MS, IMX264_WAIT_MS},
    {XMSTA,                0x01},

    {IMX264_TABLE_WAIT_MS,      30},
    {IMX264_TABLE_END,          0x0000}
};

static const imx264_reg imx264_init_settings[] = {

    {SYNCSEL,   0xC0},        // XVS, XHS NOT in High Z

    {0x3001,    0xD0},
    {0x3002,    0xAA},
    {0x3018,    0x01},
    {0x3023,    0x00},
    {0x3080,    0x62},
    {0x30AF,    0x0E},

    {0x3168,    0xD8},
    {0x3169,    0xA0},
    {0x317D,    0xA1},
    {0x3180,    0x62},
    {0x3190,    0x9B},
    {0x3191,    0xA0},
    {0x31A4,    0x3F},
    {0x31A5,    0xB1},
    {0x31E2,    0x00},
    {0x31EA,    0x00},

    {0x3226,    0x03},

    {0x35AA,    0xB3},
    {0x35AC,    0x68},

    {0x371C,    0xB4},
    {0x371D,    0x00},
    {0x371E,    0xDE},
    {0x371F,    0x00},
    {0x3728,    0xB4},
    {0x3729,    0x00},
    {0x372A,    0xDE},
    {0x372B,    0x00},
    {0x373A,    0x36},
    {0x3746,    0x36},

    {0x38E0,    0xEB},
    {0x38E1,    0x00},
    {0x38E2,    0x0D},
    {0x38E3,    0x01},

    {0x39C4,    0xEB},
    {0x39C5,    0x00},
    {0x39C6,    0x0C},
    {0x39C7,    0x01},

    {0x3D02,    0x6E},
    {0x3D04,    0xE3},
    {0x3D05,    0x00},
    {0x3D0C,    0x73},
    {0x3D0E,    0x6E},
    {0x3D10,    0xE8},
    {0x3D11,    0x00},
    {0x3D12,    0xE3},
    {0x3D13,    0x00},
    {0x3D14,    0x6B},
    {0x3D16,    0x1C},
    {0x3D18,    0x1C},
    {0x3D1A,    0x6B},
    {0x3D1C,    0x6E},
    {0x3D1E,    0x9A},
    {0x3D20,    0x12},
    {0x3D22,    0x3E},
    {0x3D28,    0xB4},
    {0x3D29,    0x00},
    {0x3D2A,    0x66},
    {0x3D34,    0x69},
    {0x3D36,    0x17},
    {0x3D38,    0x6A},
    {0x3D3A,    0x18},
    {0x3D3E,    0xFF},
    {0x3D3F,    0x0F},
    {0x3D46,    0xFF},
    {0x3D47,    0x0F},
    {0x3D4E,    0x4C},
    {0x3D50,    0x50},
    {0x3D54,    0x73},
    {0x3D56,    0x6E},
    {0x3D58,    0xE8},
    {0x3D59,    0x00},
    {0x3D5A,    0xCF},
    {0x3D5B,    0x00},
    {0x3D5E,    0x64},
    {0x3D66,    0x61},
    {0x3D6E,    0x0D},
    {0x3D70,    0xFF},
    {0x3D71,    0x0F},
    {0x3D72,    0x00},
    {0x3D73,    0x00},
    {0x3D74,    0x11},
    {0x3D76,    0x6A},
    {0x3D78,    0x7F},
    {0x3D7A,    0xB3},
    {0x3D7C,    0x29},
    {0x3D7E,    0x64},
    {0x3D80,    0xB1},
    {0x3D82,    0xB3},
    {0x3D84,    0x62},
    {0x3D86,    0x64},
    {0x3D88,    0xB1},
    {0x3D8A,    0xB3},
    {0x3D8C,    0x62},
    {0x3D8E,    0x64},
    {0x3D90,    0x6D},
    {0x3D92,    0x65},
    {0x3D94,    0x65},
    {0x3D96,    0x6D},
    {0x3D98,    0x20},
    {0x3D9A,    0x28},
    {0x3D9C,    0x81},
    {0x3D9E,    0x89},
    {0x3D9F,    0x01},
    {0x3DA0,    0x66},
    {0x3DA2,    0x7B},
    {0x3DA4,    0x21},
    {0x3DA6,    0x27},
    {0x3DA8,    0x8B},
    {0x3DA9,    0x01},
    {0x3DAA,    0x95},
    {0x3DAB,    0x01},
    {0x3DAC,    0x12},
    {0x3DAE,    0x1C},
    {0x3DB0,    0x98},
    {0x3DB1,    0x01},
    {0x3DB2,    0xA0},
    {0x3DB3,    0x01},
    {0x3DB4,    0x13},
    {0x3DB6,    0x1D},
    {0x3DB8,    0x99},
    {0x3DB9,    0x01},
    {0x3DBA,    0xA1},
    {0x3DBB,    0x01},
    {0x3DBC,    0x14},
    {0x3DBE,    0x1E},
    {0x3DC0,    0x9A},
    {0x3DC1,    0x01},
    {0x3DC2,    0xA2},
    {0x3DC3,    0x01},
    {0x3DC4,    0x64},
    {0x3DC6,    0x6E},
    {0x3DC8,    0x17},
    {0x3DCA,    0x26},
    {0x3DCC,    0x9D},
    {0x3DCD,    0x01},
    {0x3DCE,    0xAC},
    {0x3DCF,    0x01},
    {0x3DD0,    0x65},
    {0x3DD2,    0x6F},
    {0x3DD4,    0x18},
    {0x3DD6,    0x27},
    {0x3DD8,    0x9E},
    {0x3DD9,    0x01},
    {0x3DDA,    0xAD},
    {0x3DDB,    0x01},
    {0x3DDC,    0x66},
    {0x3DDE,    0x70},
    {0x3DE0,    0x19},
    {0x3DE2,    0x28},
    {0x3DE4,    0x9F},
    {0x3DE5,    0x01},
    {0x3DE6,    0xAE},
    {0x3DE7,    0x01},

    {0x3E04,    0x9D},
    {0x3E06,    0xB0},
    {0x3E07,    0x00},
    {0x3E08,    0x6B},
    {0x3E0A,    0x7E},
    {0x3E24,    0xE3},
    {0x3E25,    0x00},
    {0x3E26,    0x9A},
    {0x3E27,    0x01},

    {0x3F20,    0x00},
    {0x3F21,    0x00},
    {0x3F22,    0xFF},
    {0x3F23,    0x3F},

    {0x4003,    0x55},
    {0x4005,    0xFF},
    {0x400B,    0x00},
    {0x400C,    0x54},
    {0x400E,    0x48},
    {0x400F,    0xA2},
    {0x4012,    0x53},
    {0x4013,    0x0A},
    {0x4014,    0x0C},
    {0x4015,    0x0A},
    {0x402A,    0x7F},
    {0x402C,    0x29},
    {0x4030,    0x73},
    {0x4032,    0x8D},
    {0x4033,    0x01},
    {0x4049,    0x02},
    {0x4056,    0x18},
    {0x408C,    0x9A},
    {0x408E,    0xAA},
    {0x4090,    0x3E},
    {0x4092,    0x5F},
    {0x4094,    0x0A},
    {0x4096,    0x0A},
    {0x4098,    0x7F},
    {0x409A,    0xB3},
    {0x409C,    0x29},
    {0x409E,    0x64},

    {GAINDLY,   0x09},

    {IMX264_TABLE_WAIT_MS,      IMX264_WAIT_MS},
    {IMX264_TABLE_END,          0x0000}
};

static const imx264_reg mode_2464x2056[] = {

    {0x30AF,        0x0E},
    {HMODE_WINMODE, 0x00},
    {VMAX_LOW,      0x28},
    {VMAX_MID,      0x08},
    {VMAX_HIGH,     0x00},
    
    {HMAX_LOW,      0xE4},
    {HMAX_HIGH,     0x03},

    {CKSEL,         0x00},

    {FID0_ROI,      0x00},

    {INCKSEL0,      0x10},
    {INCKSEL1,      0x02},
    {INCKSEL2,      0x10},
    {INCKSEL3,      0x02},

    {GTWAIT,        0x08},
    {GSDLY,         0x04},

    {IMX264_TABLE_WAIT_MS,      IMX264_WAIT_MS},
    {IMX264_TABLE_END,          0x0000}
};

static const imx264_reg mode_ROI_2048x1536[] = {
    
    {0x30AF,        0x0E},
    {HMODE_WINMODE, 0x00},
    {VMAX_LOW,      0x20},
    {VMAX_MID,      0x06},
    {VMAX_HIGH,     0x00},
    
    {HMAX_LOW,      0xE4},
    {HMAX_HIGH,     0x03},

    {CKSEL,         0x00},

    {INCKSEL0,      0x10},
    {INCKSEL1,      0x02},
    {INCKSEL2,      0x10},
    {INCKSEL3,      0x02},

    {GTWAIT,        0x08},
    {GSDLY,         0x04},

    {FID0_ROI,              0x03},

    {FID0_ROIPH1_LOW,       IMX264_TO_LOW_BYTE(208)}, 
    {FID0_ROIPH1_HIGH,      IMX264_TO_MID_BYTE(208)},

    {FID0_ROIPV1_LOW,       IMX264_TO_LOW_BYTE(260)},
    {FID0_ROIPV1_HIGH,      IMX264_TO_MID_BYTE(260)},

    {FID0_ROIWH1_LOW,       IMX264_TO_LOW_BYTE(IMX264_ROI_MODE_WIDTH)},
    {FID0_ROIWH1_HIGH,      IMX264_TO_MID_BYTE(IMX264_ROI_MODE_WIDTH)},

    {FID0_ROIWV1_LOW,       IMX264_TO_LOW_BYTE(IMX264_ROI_MODE_HEIGHT)},
    {FID0_ROIWV1_HIGH,      IMX264_TO_MID_BYTE(IMX264_ROI_MODE_HEIGHT)},

    {IMX264_TABLE_WAIT_MS,  IMX264_WAIT_MS},
    {IMX264_TABLE_END,      0x0000}
};

static const imx264_reg mode_1080p[] = {
    
    {0x30AF,    0x0A},

    {HMODE_WINMODE, 0x0C},
    {VMAX_LOW,      0x65},
    {VMAX_MID,      0x04},
    {VMAX_HIGH,     0x00},
    
    {HMAX_LOW,      0x4C},
    {HMAX_HIGH,     0x04},

    {CKSEL,         0x01},

    {0x3300,        0x00},
    
    {INCKSEL0,      0x18},
    {INCKSEL1,      0x00},
    {INCKSEL2,      0x10},
    {INCKSEL3,      0x02},

    {GTWAIT,        0x06},
    {GSDLY,         0x02},

    {IMX264_TABLE_WAIT_MS,      IMX264_WAIT_MS},
    {IMX264_TABLE_END,          0x0000}
};

/**
 * Enum of available frame modes
 */

enum {

    IMX264_MODE_2464x2056,
    IMX264_MODE_ROI_2048x1536,
    IMX264_MODE_1080p,

    IMX264_INIT_SETTINGS,
    IMX264_MODE_STOP_STREAM,
};

/**
 * Connecting frame modes to mode tables
 */

static const imx264_reg *mode_table[] = {

    [IMX264_MODE_2464x2056]     = mode_2464x2056,
    [IMX264_MODE_ROI_2048x1536] = mode_ROI_2048x1536,
    [IMX264_MODE_1080p]         = mode_1080p,

    [IMX264_INIT_SETTINGS]      = imx264_init_settings,

    [IMX264_MODE_STOP_STREAM]   = imx264_stop,
};

/**
 * Framerates of available frame modes
 */

static const int imx264_35fps[] = {
    35,
};
static const int imx264_46fps[] = {
    46,
};
static const int imx264_60fps[] = {
    60,
};

/**
 * Connecting resolutions, framerates and mode tables
 */

static const struct camera_common_frmfmt imx264_frmfmt[] = {
    {
        .size = {IMX264_DEFAULT_WIDTH, IMX264_DEFAULT_HEIGHT},
        .framerates = imx264_35fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX264_MODE_2464x2056
    },
    {
        .size = {IMX264_ROI_MODE_WIDTH, IMX264_ROI_MODE_HEIGHT},
        .framerates = imx264_46fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX264_MODE_ROI_2048x1536
    },
    {
        .size = {IMX264_1080p_MODE_WIDTH, IMX264_1080p_MODE_HEIGHT},
        .framerates = imx264_60fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX264_MODE_1080p
    },

};

#endif /* __IMX264_TABLES__ */
