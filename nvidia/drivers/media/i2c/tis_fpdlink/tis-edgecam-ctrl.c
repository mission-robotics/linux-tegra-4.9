/*
 * The Imaging Source edgecamera driver
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
#include <linux/gpio/driver.h>


struct tis_edgecam_ctrl {
    struct i2c_client *i2c_client;
    struct gpio_chip gpio_chip;
    struct regmap *regmap;
};


static int tis_edgecam_ctrl_gpio_get(struct gpio_chip *gpio, unsigned int offset);
static void tis_edgecam_ctrl_gpio_set(struct gpio_chip *gpio, unsigned int offset, int val);
static int tis_edgecam_ctrl_gpio_direction_output(struct gpio_chip *gpio, unsigned int offset, int val);

static const struct gpio_chip tis_interposer_gpio_chip = {
    .label          = "gpio-tis-edgecam",
    .owner          = THIS_MODULE,
    .direction_input    = tis_edgecam_ctrl_gpio_get,
    .direction_output   = tis_edgecam_ctrl_gpio_direction_output,
    .set            = tis_edgecam_ctrl_gpio_set,
    .get            = tis_edgecam_ctrl_gpio_get,
    .base           = -1,
    .ngpio          = 8,
    .can_sleep      = true,
};

static const struct regmap_config tis_edgecam_ctrl_regmap_cfg = {
                .reg_bits = 8,
                .val_bits = 8,
                .reg_stride = 1,
                .name = "tis-edgecam-ctrl",
};

static int tis_edgecam_ctrl_gpio_get(struct gpio_chip *gpio, unsigned int offset)
{
    struct tis_edgecam_ctrl *priv = container_of(gpio, struct tis_edgecam_ctrl, gpio_chip);
    int ret = -EINVAL;
	unsigned int val;

    switch (offset)
    {
    	case 0:
    		// CAM_POWER
    		regmap_read(priv->regmap, 0x0, &val);
    		ret =  val & 0x1;
    		break;
    	case 1:
    		// CAM_RESET
    		regmap_read(priv->regmap, 0x0, &val);
    		ret =  (val & 0x4) ? 0 : 1;
    		break;
    	default:
    		break;
    }


    return ret;
}

static void tis_edgecam_ctrl_gpio_set(struct gpio_chip *gpio, unsigned int offset, int val)
{
    struct tis_edgecam_ctrl *priv = container_of(gpio, struct tis_edgecam_ctrl, gpio_chip);
    unsigned int regval;

    switch (offset)
    {
    	case 0:
    		// CAM_POWER
    		regmap_read(priv->regmap, 0x0, &regval);
    		regval &= ~0x3;
    		regval |= val ? 1:0;
    		regmap_write(priv->regmap, 0x0, regval);
    		break;
    	case 1:
    		// CAM_RESET
    		regmap_read(priv->regmap, 0x0, &regval);
    		val = !val;
    		regval &= ~(1<<2);
    		regval |= val ? (1<<2) : 0;
    		regmap_write(priv->regmap, 0x0, regval);
    		break;
    	default:
    		break;
    }
}

static int tis_edgecam_ctrl_gpio_direction_output(struct gpio_chip *gpio, unsigned int offset, int val)
{
	tis_edgecam_ctrl_gpio_set(gpio, offset, val);
	return 0;
}

static int tis_edgecam_ctrl_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct tis_edgecam_ctrl *priv;
    int ret;

    dev_info(&client->dev, "tis edgecam-ctrl probe\n");

    priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    i2c_set_clientdata(client, priv);
    priv->i2c_client = client;
    priv->regmap = devm_regmap_init_i2c(client, &tis_edgecam_ctrl_regmap_cfg);
    priv->gpio_chip = tis_interposer_gpio_chip;
    priv->gpio_chip.parent = &client->dev;
    ret = gpiochip_add_data(&priv->gpio_chip, priv);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to add GPIO chip");
    }

    return 0;
}

static int tis_edgecam_ctrl_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id tis_edgecam_ctrl_id[] = {
	{ "tis-edgecam-ctrl", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tis_edgecam_ctrl_id);


static const struct of_device_id tis_edgecam_ctrl_of_match[] = {
	{ .compatible = "tis,tis-edgecam-ctrl", },
	{},
};
MODULE_DEVICE_TABLE(of, tis_edgecam_ctrl_of_match);

static struct i2c_driver tis_edgecam_ctrl_driver = {
	.probe	= tis_edgecam_ctrl_probe,
	.remove	= tis_edgecam_ctrl_remove,
	.id_table = tis_edgecam_ctrl_id,
	.driver	= {
		.name  = "tis-edgecam-ctrl",
        .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tis_edgecam_ctrl_of_match),
	},
};

module_i2c_driver(tis_edgecam_ctrl_driver);

MODULE_DESCRIPTION("The Imaging Source edgecam_ctrl driver");
MODULE_AUTHOR("The Imaging Source Europe GmbH");
MODULE_LICENSE("GPL");
