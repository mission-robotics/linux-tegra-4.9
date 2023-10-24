/*
 * max929x_imx304.c - max929x_imx304 IO Expander driver
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
/* #define DEBUG */

#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <media/camera_common.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include "max929x_imx304.h"

struct max929x_imx304 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct gpio_desc *pwdn_gpio;
	int sensor_number;
	struct dentry *max929x_imx304_config;
};

static int max929x_imx304_write_reg(struct max929x_imx304 *priv, u8 slave_addr, u16 reg, u8 val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;

	i2c_client->addr = slave_addr;
	err = regmap_write(priv->regmap, reg, val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c write failed, slave_addr 0x%x, 0x%x = 0x%x\n",
			__func__, slave_addr, reg, val);

	return err;
}
/*
static int max929x_imx304_read_reg(struct max929x_imx304 *priv, u8 slave_addr, u16 reg, unsigned int *val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;

	i2c_client->addr = slave_addr;
	err = regmap_read(priv->regmap, reg, val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c read failed, 0x%x = %x\n",
			__func__, reg, *val);

	return err;
}
 */
int max929x_imx304_write_reg_list(struct max929x_imx304 *priv, struct max929x_imx304_reg *table, int size)
{
	struct device dev = priv->i2c_client->dev;
	int err = 0, i;
	u8 slave_addr;
	u16 reg;
	u8 val;

	for(i=0; i<size; i++)
	{
		slave_addr = table[i].slave_addr;
		reg = table[i].reg;
		val = table[i].val;

		dev_dbg(&dev, "%s: size %d, slave_addr 0x%x, reg 0x%x, val 0x%x\n",
				__func__, size, slave_addr, reg, val);

		err = max929x_imx304_write_reg(priv, slave_addr, reg, val);

		if(err!=0)
			break;

		if (reg == 0x0010 || reg == 0x0000)
		    msleep(300);
	}

	return err;
}

void max929x_imx304_parse_dt(struct max929x_imx304 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int sensor_number;
	int err;

	err = of_property_read_u32(dev->of_node, "sensor-number", &sensor_number);
	if(err)
	{
		dev_info(dev, "%s: sensor-number attribute does not exist in dts\n", __func__);
		dev_info(dev, "%s: default: sensor_number is 1\n", __func__);
		sensor_number = 1;
	}
	priv->sensor_number = sensor_number;
}

int max929x_imx304_configuration(struct max929x_imx304 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int sensor_number = priv->sensor_number;
	u8 link_cfg;
	int size, err;

	dev_dbg(dev, "%s: sensor_number %d\n", __func__, sensor_number);

	max929x_imx304_write_reg(priv, 0x48, 0x0010, 0x80); // max9296 reset
	msleep(300);
	max929x_imx304_write_reg(priv, 0x60, 0x0010, 0x80); // linka max9295 reset 
	msleep(300); 
	max929x_imx304_write_reg(priv, 0x62, 0x0010, 0x80); // linkb max9295 reset
	msleep(300);

	size = sizeof(max929x_imx304_common_init)/sizeof(struct max929x_imx304_reg);
	err = max929x_imx304_write_reg_list(priv, max929x_imx304_common_init, size);
	if(err)
		goto EXIT;

	/*
	 * max9296 how many link are connected
	 *
	 * if link A only, link_cfg = 0x21
	 * if link A and link B, link_cfg = 0x23
	 */
	link_cfg = (sensor_number >= 2) ? 0x23 : 0x21;
	err = max929x_imx304_write_reg(priv, 0x48, 0x0010, link_cfg);
	if(err)
		goto EXIT;
	else
		msleep(300);

	size = sizeof(max929x_imx304_Dser_init)/sizeof(struct max929x_imx304_reg);
	err = max929x_imx304_write_reg_list(priv, max929x_imx304_Dser_init, size);
	if(err)
		goto EXIT;

	size = sizeof(max929x_imx304_Double_Ser_A_init)/sizeof(struct max929x_imx304_reg);
	err = max929x_imx304_write_reg_list(priv, max929x_imx304_Double_Ser_A_init, size);
	if(err)
		goto EXIT;

	if(sensor_number == 2) {
		size = sizeof(max929x_imx304_Double_Ser_B_init)/sizeof(struct max929x_imx304_reg);
		err = max929x_imx304_write_reg_list(priv, max929x_imx304_Double_Ser_B_init, size);
		if(err)
			goto EXIT;
	}

EXIT:
	return err;
}

ssize_t max929x_imx304_write_config(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
	struct max929x_imx304 *priv = file->f_inode->i_private;
	struct device *dev = &priv->i2c_client->dev;
    unsigned char *kbuf;
	int ret;

    kbuf = kmalloc(size, GFP_KERNEL);

    ret = copy_from_user(kbuf, buf, size);
    kbuf[size-1] = '\0';

	if(!strcmp("1", kbuf)) {
		ret = max929x_imx304_configuration(priv);
		if(ret)
			dev_err(dev,"max929x_imx304 configuration failed\n");
		else
			dev_info(dev,"max929x_imx304 configuration successful\n");
	}
	else {
		dev_info(dev,
			"%s: echo 1 > /sys/kernel/debug/max929x_imx304/config, to configuration max929x_imx304\n",
			__func__);
	}

    kfree(kbuf);

    return size;
}

const struct file_operations max929x_imx304_fops = {
    .write   = max929x_imx304_write_config,
};

static  struct regmap_config max929x_imx304_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max929x_imx304_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max929x_imx304 *priv;
	struct device *dev = &client->dev;
	char *dir;
	int err;

	dev_dbg(dev, "%s: enter\n", __func__);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	i2c_set_clientdata(client, priv);
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &max929x_imx304_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev,"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	dir = devm_kzalloc(dev, strlen(client->name)+3, GFP_KERNEL);
	sprintf(dir, "%s-%d", client->name, client->adapter->nr);
	priv->max929x_imx304_config = debugfs_create_dir(dir, NULL);
	debugfs_create_file("config", 0644, priv->max929x_imx304_config, (void *)priv, &max929x_imx304_fops);
	debugfs_create_u32("sensor-number", 0644, priv->max929x_imx304_config, &priv->sensor_number);

	max929x_imx304_parse_dt(priv);

	err = max929x_imx304_configuration(priv);
	if(err)
		dev_err(dev,"max929x_imx304 configuration failed\n");
	else
		dev_info(dev,"max929x_imx304 configuration successful\n");

	return 0;
}

static int max929x_imx304_remove(struct i2c_client *client)
{
	struct max929x_imx304 *priv = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	dev_dbg(dev, "%s: \n", __func__);

	debugfs_remove_recursive(priv->max929x_imx304_config);

	return 0;
}

static const struct i2c_device_id max929x_imx304_id[] = {
	{ "max929x_imx304", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max929x_imx304_id);

const struct of_device_id max929x_imx304_of_match[] = {
	{ .compatible = "nvidia,max929x_imx304", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx185_of_match);

static struct i2c_driver max929x_imx304_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "max929x_imx304",
		.of_match_table = of_match_ptr(max929x_imx304_of_match),
	},
	.probe = max929x_imx304_probe,
	.remove = max929x_imx304_remove,
	.id_table = max929x_imx304_id,
};

static int __init max929x_imx304_init(void)
{
	return i2c_add_driver(&max929x_imx304_i2c_driver);
}

static void __exit max929x_imx304_exit(void)
{
	i2c_del_driver(&max929x_imx304_i2c_driver);
}

module_init(max929x_imx304_init);
module_exit(max929x_imx304_exit);

MODULE_DESCRIPTION("IO Expander driver max929x_imx304");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
