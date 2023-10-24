
#include "tis_sensor_regmap.h"

int tis_sensor_regmap_read8(struct regmap *regmap, u16 addr, u8 *val)
{
	u32 reg_val = 0;

	int err = regmap_read(regmap, addr, &reg_val);
    if( !err )
    {
	    *val = reg_val & 0xFF;
    }

	return err;
}
int tis_sensor_regmap_write8(struct regmap *regmap, u16 addr, u8 val)
{
    return regmap_write(regmap, addr, val);
}

int tis_sensor_regmap_read16_le(struct regmap *regmap, u16 addr, u16 *val)
{
	u8 raw_val[2];

	int err = regmap_raw_read(regmap, addr, &raw_val, ARRAY_SIZE(raw_val));
    if( !err )
    {
        *val = raw_val[0] | raw_val[1] << 8;
    }

	return err;
}
int tis_sensor_regmap_write16_le(struct regmap *regmap, u16 addr, u16 val)
{
	u8 raw_val[2] = { val, val >> 8 };

    return regmap_raw_write(regmap, addr, raw_val, ARRAY_SIZE(raw_val));
}

int tis_sensor_regmap_read16_be(struct regmap *regmap, u16 addr, u16 *val)
{
	u8 raw_val[2];

	int err = regmap_raw_read(regmap, addr, &raw_val, ARRAY_SIZE(raw_val));
    if( !err )
    {
        *val = raw_val[1] | (raw_val[0] << 8);
    }

	return err;
}
int tis_sensor_regmap_write16_be(struct regmap *regmap, u16 addr, u16 val)
{
	u8 raw_val[2] = { val >> 8, val };

    return regmap_raw_write(regmap, addr, raw_val, ARRAY_SIZE(raw_val));    
}

int tis_sensor_regmap_read24_le(struct regmap *regmap, u16 addr, u32 *val)
{
	u8 raw_val[3];

	int err = regmap_raw_read(regmap, addr, &raw_val, ARRAY_SIZE(raw_val));
    if( !err )
    {
        *val = raw_val[0] | (raw_val[1] << 8) | (raw_val[2] << 16);
    }

	return err;
}
int tis_sensor_regmap_write24_le(struct regmap *regmap, u16 addr, u32 val)
{
	u8 raw_val[3] = { val, val >> 8, val >> 16 };

    return regmap_raw_write(regmap, addr, raw_val, ARRAY_SIZE(raw_val));
}