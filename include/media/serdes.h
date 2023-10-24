/*
 * Copyright (C) 2018 e-con Systems Pvt Ltd, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SERDES_H
#define __SERDES_H

// #define SER_ADDR1 0x40
// #define DES_ADDR1 0x48
// #define SER_ADDR2 0x41
// #define SER_ADDR3 0x50

int serdes_write_i2c(struct i2c_client *client, u16 sladdr,  u8 * val, u32 count);
int serdes_read_i2c(struct i2c_client *client, u16 sladdr, u8 * val, u32 count);
s32 serdes_read_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 * val);
s32 serdes_write_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 val);
s32 serdes_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 * val);
s32 serdes_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val);

#endif /* __SERDES_H */
