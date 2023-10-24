/*
 * max929x.c - max929x IO Expander driver
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


struct max929x {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	const char *channel;
};
struct max929x *global_priv[4] ;


int max929x_write_reg_Dser(int slaveAddr,int channel,
			u16 addr, u8 val)
{
	struct i2c_client *i2c_client = NULL;
	int bak = 0;
	int err;
	/* unsigned int ival = 0; */

	if(channel > 3 || channel < 0)
		return -1;
	i2c_client = global_priv[channel]->i2c_client;
	bak = i2c_client->addr;

	i2c_client->addr = slaveAddr / 2;
	err = regmap_write(global_priv[channel]->regmap, addr, val);

	i2c_client->addr = bak;
	if(err)
	{
		dev_err(&i2c_client->dev, "%s: addr = 0x%x, val = 0x%x\n",
				__func__, addr, val);
		return -1;
	}
/*
	bak = i2c_client->addr;
	i2c_client->addr = slaveAddr / 2;
	err = regmap_read(global_priv[channel]->regmap, addr, &ival);
	i2c_client->addr = bak;
	if(err)
	{
		dev_err(&i2c_client->dev, "%s: addr = 0x%x, val = 0x%x\n",
				__func__, addr, val);
		return -1;
	}
	dev_dbg(&i2c_client->dev, "%s: addr = 0x%x, val = 0x%x\n", __func__, addr, val);
 */
	return 0;
}

EXPORT_SYMBOL(max929x_write_reg_Dser);

#if 1
static int max929x_write_reg(struct max929x *priv, u8 slave_addr,
				u16 addr, u8 val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;

	i2c_client->addr = slave_addr;
	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}
#endif
static int max929x_read_reg(struct max929x *priv,
			u16 addr, unsigned int *val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;

	err = regmap_read(priv->regmap, addr, val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c read failed, 0x%x = %x\n",
			__func__, addr, *val);

	return err;
}


static int max929x_stats_show(struct seq_file *s, void *data)
{
	return 0;
}

static int max929x_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, max929x_stats_show, inode->i_private);
}

static ssize_t max929x_debugfs_write(struct file *s,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct max929x *priv =
		((struct seq_file *)s->private_data)->private;
	struct i2c_client *i2c_client = priv->i2c_client;

	char buf[255];
	int buf_size;
	int val = 0;

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (buf[0] == 'd') {
		dev_info(&i2c_client->dev, "%s, set daymode\n", __func__);
		max929x_read_reg(priv, 0x0010, &val);
		return count;
	}

	if (buf[0] == 'n') {
		dev_info(&i2c_client->dev, "%s, set nightmode\n", __func__);
		return count;
	}


	return count;
}


static const struct file_operations max929x_debugfs_fops = {
	.open = max929x_debugfs_open,
	.read = seq_read,
	.write = max929x_debugfs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int max929x_debugfs_init(const char *dir_name,
				struct dentry **d_entry,
				struct dentry **f_entry,
				struct max929x *priv)
{
	struct dentry  *dp, *fp;
	char dev_name[20];
	struct i2c_client *i2c_client = priv->i2c_client;
	struct device_node *np = i2c_client->dev.of_node;
	int err = 0;
	int index = 0;

	if (np) {
		err = of_property_read_string(np, "channel", &priv->channel);
		if (err)
			dev_err(&i2c_client->dev, "channel not found\n");

		snprintf(dev_name, sizeof(dev_name), "max929x_%s", priv->channel);
	}
	index = priv->channel[0] - 'a';
	global_priv[index] = priv;

	dev_dbg(&i2c_client->dev, "%s: index %d\n", __func__, index);

	dp = debugfs_create_dir(dev_name, NULL);
	if (dp == NULL) {
		dev_err(&i2c_client->dev, "%s: debugfs create dir failed\n",
			__func__);
		return -ENOMEM;
	}

	fp = debugfs_create_file("max929x", S_IRUGO|S_IWUSR,
		dp, priv, &max929x_debugfs_fops);
	if (!fp) {
		dev_err(&i2c_client->dev, "%s: debugfs create file failed\n",
			__func__);
		debugfs_remove_recursive(dp);
		return -ENOMEM;
	}

	if (d_entry)
		*d_entry = dp;
	if (f_entry)
		*f_entry = fp;
	return 0;
}

static  struct regmap_config max929x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max929x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max929x *priv;
	int err = 0;
//	unsigned int val = 0;

	dev_info(&client->dev, "%s: enter\n", __func__);

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&max929x_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	err = max929x_debugfs_init(NULL, NULL, NULL, priv);
	if (err)
		return err;

	max929x_write_reg(priv, 0x48, 0x0010, 0xff);
	msleep(300);
	max929x_write_reg(priv, 0x62, 0x0010, 0xff);
	msleep(300);
	max929x_write_reg(priv, 0x60, 0x0010, 0xff);
	msleep(300);

	/*set daymode by fault*/
	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}


static int
max929x_remove(struct i2c_client *client)
{

	if (client != NULL) {
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max929x_id[] = {
	{ "max929x", 0 },
	{ },
};

const struct of_device_id max929x_of_match[] = {
	{ .compatible = "nvidia,max929x", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx185_of_match);
MODULE_DEVICE_TABLE(i2c, max929x_id);

static struct i2c_driver max929x_i2c_driver = {
	.driver = {
		.name = "max929x",
		.owner = THIS_MODULE,
	},
	.probe = max929x_probe,
	.remove = max929x_remove,
	.id_table = max929x_id,
};

static int __init max929x_init(void)
{
	return i2c_add_driver(&max929x_i2c_driver);
}

static void __exit max929x_exit(void)
{
	i2c_del_driver(&max929x_i2c_driver);
}

module_init(max929x_init);
module_exit(max929x_exit);

MODULE_DESCRIPTION("IO Expander driver max929x");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
