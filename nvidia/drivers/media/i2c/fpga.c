/*
 * fpga.c - fpga IO Expander driver
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
#include <linux/debugfs.h>

/*
 * 7bit slaver addr : register addr : regiser value(unit: 10us)
 * 0x3c             : 0x01          :  xxx (hight level time)
 * 0x3c             : 0x02          ： xxx (cycle time)
 */
struct fpga {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
};
static struct fpga *priv;
static struct dentry *fpga;

#define FPGA_EXPOSURE_REG 0x01
#define FPGA_CYCLE_REG    0x02

int fpga_write_reg(u8 reg, u16 val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;

	err = regmap_write(priv->regmap, reg, val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c write failed, 0x%x = 0x%x\n",
			__func__, reg, val);

	return err;
}
EXPORT_SYMBOL(fpga_write_reg);

int fpga_read_reg(u8 reg, unsigned int *val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;

	err = regmap_read(priv->regmap, reg, val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c read failed, 0x%x = %x\n",
			__func__, reg, *val);

	return err;
}
EXPORT_SYMBOL(fpga_read_reg);

ssize_t fps_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
    unsigned char kbuf[6];
	unsigned int fps;
	unsigned int val;

	fpga_read_reg(FPGA_CYCLE_REG, &val);
	fps = 100000 / val;
	sprintf(kbuf, "%dfps\n", fps);

    return simple_read_from_buffer(buf, size, offset, kbuf, sizeof(kbuf));
}

ssize_t fps_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
	struct device *dev = &priv->i2c_client->dev;
    unsigned char *kbuf;
	unsigned int fps;
	unsigned int val;
	int ret;

    kbuf = kmalloc(size, GFP_KERNEL);

    ret = copy_from_user(kbuf, buf, size);
    kbuf[size-1] = '\0';

	ret = kstrtouint(kbuf, 0, &fps);
	val = 1000 / fps * 100;
	fpga_write_reg(FPGA_EXPOSURE_REG, val - 30); // min exposure
	fpga_write_reg(FPGA_CYCLE_REG, val);
	dev_dbg(dev, "%s: write %dfps %d*10us\n", __func__, fps, val);

    kfree(kbuf);

    return size;
}

ssize_t exposure_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
    unsigned char kbuf[12];
	unsigned int cycle, exposure, val;

	memset(kbuf, 0, sizeof(kbuf));
	fpga_read_reg(FPGA_CYCLE_REG, &cycle);
	fpga_read_reg(FPGA_EXPOSURE_REG, &val);

	exposure = (cycle - val)*10;
	sprintf(kbuf, "%dus\n", exposure);

    return simple_read_from_buffer(buf, size, offset, kbuf, sizeof(kbuf));
}

ssize_t exposure_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
	struct device *dev = &priv->i2c_client->dev;
    unsigned char *kbuf;
	unsigned int cycle, exposure, val;
	int ret;

    kbuf = kmalloc(size, GFP_KERNEL);

    ret = copy_from_user(kbuf, buf, size);
    kbuf[size-1] = '\0';

	ret = kstrtouint(kbuf, 0, &exposure);
	exposure = exposure / 10; /* exposure register time units: 10us */
	fpga_read_reg(FPGA_CYCLE_REG, &cycle);

	if ((exposure < 30) || (exposure > (cycle/2))) {
		dev_err(dev, "%s: exposure time: [300, %d]us\n", __func__, (cycle/2)*10);
		return -EFAULT;
	}

	val = cycle - exposure;
	fpga_write_reg(FPGA_EXPOSURE_REG, val);
	dev_dbg(dev, "%s: cycle %dus, exposure %dus\n", __func__, cycle*10, exposure*10);

    kfree(kbuf);

    return size;
}

const struct file_operations fps_fops = {
    .read    = fps_read,
    .write   = fps_write,
};

const struct file_operations exposure_fops = {
    .read    = exposure_read,
    .write   = exposure_write,
};

static  struct regmap_config fpga_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

static int fpga_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device dev = client->dev;
	unsigned int val;

	dev_dbg(&dev, "%s: enter\n", __func__);

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &fpga_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	fpga = debugfs_create_dir("fpga", NULL);
	debugfs_create_file("fps", 0644, fpga, NULL, &fps_fops);
	debugfs_create_file("exposure_us", 0644, fpga, NULL, &exposure_fops);

	val = 1000 / 4 * 100;
	fpga_write_reg(FPGA_EXPOSURE_REG, val - 30); // min exposure
	fpga_write_reg(FPGA_CYCLE_REG, val);

	dev_dbg(&dev, "%s: success\n", __func__);

	return 0;
}

static int fpga_remove(struct i2c_client *client)
{
	struct device dev = client->dev;

	dev_dbg(&dev, "%s: \n", __func__);
	debugfs_remove_recursive(fpga);

	return 0;
}

static const struct i2c_device_id fpga_id[] = {
	{ "fpga", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fpga_id);

const struct of_device_id fpga_of_match[] = {
	{ .compatible = "nvidia,fpga", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx185_of_match);

static struct i2c_driver fpga_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "fpga",
		.of_match_table = of_match_ptr(fpga_of_match),
	},
	.probe = fpga_probe,
	.remove = fpga_remove,
	.id_table = fpga_id,
};

static int __init fpga_init(void)
{
	return i2c_add_driver(&fpga_i2c_driver);
}

static void __exit fpga_exit(void)
{
	i2c_del_driver(&fpga_i2c_driver);
}

module_init(fpga_init);
module_exit(fpga_exit);

MODULE_DESCRIPTION("IO Expander driver fpga");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
