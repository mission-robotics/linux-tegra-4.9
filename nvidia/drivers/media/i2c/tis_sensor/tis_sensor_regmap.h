
#ifndef _TIS_SENSOR_REGMAP_H_
#define _TIS_SENSOR_REGMAP_H_

#include <linux/kernel.h>
#include <linux/regmap.h>

int tis_sensor_regmap_read8(struct regmap *regmap, u16 addr, u8 *val);
int tis_sensor_regmap_write8(struct regmap *regmap, u16 addr, u8 val);
int tis_sensor_regmap_read16_le(struct regmap *regmap, u16 addr, u16 *val);
int tis_sensor_regmap_write16_le(struct regmap *regmap, u16 addr, u16 val);
int tis_sensor_regmap_read16_be(struct regmap *regmap, u16 addr, u16 *val);
int tis_sensor_regmap_write16_be(struct regmap *regmap, u16 addr, u16 val);
int tis_sensor_regmap_read24_le(struct regmap *regmap, u16 addr, u32 *val);
int tis_sensor_regmap_write24_le(struct regmap *regmap, u16 addr, u32 val);

#endif // _TIS_SENSOR_REGMAP_H_
