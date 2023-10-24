/*
 * Copyright (c) 2019, The Imaging Source Europe GmbH.  All rights reserved.
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TIS_IMX290_H__
#define __TIS_IMX290_H__

#define TIS_IMX290_REG_STANDBY          0x3000
#define TIS_IMX290_REG_REGHOLD          0x3001
#define TIS_IMX290_REG_WINMODE          0x3007
#define TIS_IMX290_REG_FRSEL            0x3009
#define TIS_IMX290_REG_BLKLEVEL         0x300A
#define TIS_IMX290_REG_GAIN             0x3014
#define TIS_IMX290_REG_VMAX             0x3018
#define TIS_IMX290_REG_HMAX             0x301C
#define TIS_IMX290_REG_SHS1             0x3020
#define TIS_IMX290_REG_SHS2             0x3024
#define TIS_IMX290_REG_WINPV            0x303C
#define TIS_IMX290_REG_WINWV            0x303E
#define TIS_IMX290_REG_WINPH            0x3040
#define TIS_IMX290_REG_WINWH            0x3042
#define TIS_IMX290_REG_X_OUT_SIZE       0x3472
#define TIS_IMX290_REG_Y_OUT_SIZE       0x3418

#define TIS_IMX290_BLACK_LEVEL_MIN      0x000
#define TIS_IMX290_BLACK_LEVEL_MAX      0x1FF
#define TIS_IMX290_BLACK_LEVEL_DEFAULT  0x0F0


#endif  /* __TIS_IMX290_H__ */
