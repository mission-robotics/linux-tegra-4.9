
#include "tis_sensor.h"
#include "tis_sensor_regmap.h"

int tis_sensor_debug_write_data8(void *data, u64 val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    return tis_sensor_regmap_write8(priv->s_data->regmap, priv->debug_addr, val);
}
EXPORT_SYMBOL(tis_sensor_debug_write_data8);

int tis_sensor_debug_read_data8(void *data, u64 *val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    u8 reg_val;

    int err = tis_sensor_regmap_read8(priv->s_data->regmap, priv->debug_addr, &reg_val);
    if( !err )
    {
        *val = reg_val;
    }

	return err;
}
EXPORT_SYMBOL(tis_sensor_debug_read_data8);

int tis_sensor_debug_write_data16_le(void *data, u64 val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    return tis_sensor_regmap_write16_le(priv->s_data->regmap, priv->debug_addr, val);
}
EXPORT_SYMBOL(tis_sensor_debug_write_data16_le);

int tis_sensor_debug_read_data16_le(void *data, u64 *val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    u16 reg_val;

    int err = tis_sensor_regmap_read16_le(priv->s_data->regmap, priv->debug_addr, &reg_val);
    if( !err )
    {
        *val = reg_val;
    }

	return err;
}
EXPORT_SYMBOL(tis_sensor_debug_read_data16_le);

int tis_sensor_debug_write_data24_le(void *data, u64 val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    return tis_sensor_regmap_write24_le(priv->s_data->regmap, priv->debug_addr, val);
}
EXPORT_SYMBOL(tis_sensor_debug_write_data24_le);

int tis_sensor_debug_read_data24_le(void *data, u64 *val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    u32 reg_val;

    int err = tis_sensor_regmap_read24_le(priv->s_data->regmap, priv->debug_addr, &reg_val);
    if( !err )
    {
        *val = reg_val;
    }

	return err;
}
EXPORT_SYMBOL(tis_sensor_debug_read_data24_le);

int tis_sensor_debug_write_data16_be(void *data, u64 val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    return tis_sensor_regmap_write16_be(priv->s_data->regmap, priv->debug_addr, val);
}
EXPORT_SYMBOL(tis_sensor_debug_write_data16_be);

int tis_sensor_debug_read_data16_be(void *data, u64 *val)
{
	struct tis_sensor *priv = (struct tis_sensor*)data;

    u16 reg_val;

    int err = tis_sensor_regmap_read16_be(priv->s_data->regmap, priv->debug_addr, &reg_val);
    if( !err )
    {
        *val = reg_val;
    }

	return err;
}
EXPORT_SYMBOL(tis_sensor_debug_read_data16_be);


DEFINE_SIMPLE_ATTRIBUTE(tis_sensor_debug_data8_fops, tis_sensor_debug_read_data8,
						tis_sensor_debug_write_data8, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tis_sensor_debug_imx_data16_fops, tis_sensor_debug_read_data16_le,
						tis_sensor_debug_write_data16_le, "0x%04llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tis_sensor_debug_imx_data24_fops, tis_sensor_debug_read_data24_le,
						tis_sensor_debug_write_data24_le, "0x%06llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tis_sensor_debug_ar_data16_fops, tis_sensor_debug_read_data16_be,
						tis_sensor_debug_write_data16_be, "0x%04llx\n");

static int tis_sensor_create_debugdir(struct tis_sensor *priv)
{
    // TODO: Create separate directory, based on name/addr/vc?
	priv->debug_d = priv->tc_dev->s_data->debugdir;

    return 0;
}

void tis_sensor_register_debug_imx(struct tis_sensor *priv)
{
	tis_sensor_create_debugdir(priv);

	debugfs_create_x16("addr", S_IRUGO | S_IWUGO, priv->debug_d, &priv->debug_addr);

	debugfs_create_file("data", S_IRUGO | S_IWUGO, priv->debug_d, priv, &tis_sensor_debug_data8_fops);
    debugfs_create_file("data16", S_IRUGO | S_IWUGO, priv->debug_d, priv, &tis_sensor_debug_imx_data16_fops);
    debugfs_create_file("data24", S_IRUGO | S_IWUGO, priv->debug_d, priv, &tis_sensor_debug_imx_data24_fops);
}
EXPORT_SYMBOL(tis_sensor_register_debug_imx);

void tis_sensor_register_debug_ar(struct tis_sensor *priv)
{
	tis_sensor_create_debugdir(priv);

	debugfs_create_x16("addr", S_IRUGO | S_IWUGO, priv->debug_d, &priv->debug_addr);

	debugfs_create_file("data", S_IRUGO | S_IWUGO, priv->debug_d, priv, &tis_sensor_debug_data8_fops);
    debugfs_create_file("data16", S_IRUGO | S_IWUGO, priv->debug_d, priv, &tis_sensor_debug_ar_data16_fops);
}
EXPORT_SYMBOL(tis_sensor_register_debug_ar);