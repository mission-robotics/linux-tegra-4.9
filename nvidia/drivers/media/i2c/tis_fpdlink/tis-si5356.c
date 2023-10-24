/*
 * The Imaging Source si5356 clock driver
 *
 * Copyright (C) 2021 The Imaging Source Europe GmbH
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>

#include "Si5356-24MHz-Registers.h"
#include "Si5356-27MHz-Registers.h"
#include "Si5356-37MHz-Registers.h"


struct tis_si5356 {
    struct i2c_client *i2c_client;
    u32 clock_khz;
    struct regmap *regmap;
};

static const struct regmap_config tis_si5356_regmap_cfg = {
                .reg_bits = 8,
                .val_bits = 8,
                .reg_stride = 1,
                .name = "tis-si5356",
};


static int tis_si5356_set_clock(struct tis_si5356 *priv, u32 clock)
{
    int i;
    int ret = 0;
    const Reg_Data *regs;

    switch(clock)
    {
        case 24000:
            regs = Reg_Store_24000;
            break;
        case 27000:
            regs = Reg_Store_27000;
            break;
        case 37000:
            regs = Reg_Store_37000;
            break;
        default:
            dev_err(&priv->i2c_client->dev, "invalid clock setting\n");
            return -EINVAL;
    }

    regmap_write(priv->regmap, 230, 0x10); // OEB_ALL = 1
    regmap_write(priv->regmap, 241, 0x65); // DIS_LOL = 1

    for(i=0; i < NUM_REGS_MAX; i++)
    {
        u8 addr, value, mask;
        addr = regs[i].Reg_Addr;
        value = regs[i].Reg_Val;
        mask = regs[i].Reg_Mask;

        if (mask == 0){
            continue;
        }
        if (mask != 0xff)
        {
            unsigned int tmpval;
            u8 tmp8;

            ret = regmap_read(priv->regmap, addr, &tmpval);
            if (ret < 0){
                dev_err(&priv->i2c_client->dev, "failed to read from clockchip\n");
                return -EINVAL;
            }
            tmp8 = tmpval & 0xff;
            tmp8 = tmp8 & (~mask);
            value = value & mask;
            value = tmp8 | value;
        }
        ret = regmap_write(priv->regmap, addr, value);
        if (ret < 0)
        {
            dev_err(&priv->i2c_client->dev, "failed to write to clockchip\n");
            return -EINVAL;
        }
    }

    regmap_write(priv->regmap, 246, 2);
    regmap_write(priv->regmap, 226, 4);
    regmap_write(priv->regmap, 226, 0);
    regmap_write(priv->regmap, 230, 0);

    dev_info(&priv->i2c_client->dev, "clock set to %d khz\n", clock);

    priv->clock_khz = clock;

    return 0;
}

static int tis_si5356_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct device_node *np = client->dev.of_node;
    struct tis_si5356 *priv;
    u32 clock_khz;

    dev_info(&client->dev, "tis si5356 probe\n");

    priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    i2c_set_clientdata(client, priv);
    priv->i2c_client = client;
    priv->regmap = devm_regmap_init_i2c(client, &tis_si5356_regmap_cfg);
    if (IS_ERR(priv->regmap)) {
        dev_err(&client->dev, "Failed to create regmap\n");
        return -EIO;
    }

    of_property_read_u32(np, "clock_khz", &clock_khz);

    tis_si5356_set_clock(priv, clock_khz);

    return 0;
}

static int tis_si5356_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tis_si5356_id[] = {
	{ "tis-si5356", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tis_si5356_id);


static const struct of_device_id tis_si5356_of_match[] = {
	{ .compatible = "tis,tis-si5356", },
	{},
};
MODULE_DEVICE_TABLE(of, tis_si5356_of_match);

static struct i2c_driver tis_si5356_driver = {
	.probe	= tis_si5356_probe,
	.remove	= tis_si5356_remove,
	.id_table = tis_si5356_id,
	.driver	= {
		.name  = "tis-si5356",
        .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tis_si5356_of_match),
	},
};

module_i2c_driver(tis_si5356_driver);

MODULE_DESCRIPTION("The Imaging Source si5356 driver");
MODULE_AUTHOR("The Imaging Source Europe GmbH");
MODULE_LICENSE("GPL");
