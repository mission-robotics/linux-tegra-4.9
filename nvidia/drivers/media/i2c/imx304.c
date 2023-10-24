/*
 * imx304.c - imx304 sensor driver
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include "imx304_mode_tbls.h"
#define CREATE_TRACE_POINTS
#include <trace/events/imx304.h>

#define IMX304_FRAME_LENGTH_ADDR_MSB		0x301A
#define IMX304_FRAME_LENGTH_ADDR_MID		0x3019
#define IMX304_FRAME_LENGTH_ADDR_LSB		0x3018

#define IMX304_GAIN_ADDR_MSB				0x3205
#define IMX304_GAIN_ADDR_LSB				0x3204

#define IMX304_GROUP_HOLD_ADDR				0x3008

#define IMX304_ANALOG_GAIN_LIMIT_ADDR			0x3012
#define IMX304_ANALOG_GAIN_LIMIT_VALUE			0x0f

#define FPGA_EXPOSURE_REG 0x01
#define FPGA_CYCLE_REG    0x02
extern int fpga_write_reg(u8 reg, u16 val);
extern int fpga_read_reg(u8 reg, unsigned int *val);

static const struct of_device_id imx304_of_match[] = {
	{ .compatible = "nvidia,imx304",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx304_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	/* TEGRA_CAMERA_CID_FUSE_ID, */
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx304 {
	struct i2c_client	*i2c_client;
	struct v4l2_subdev	*subdev;
	u32				frame_length;
	struct dentry			*debugfs_dir;
	s64 last_wdr_et_val;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline void imx304_get_frame_length_regs(imx304_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX304_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 16) & 0x01;

	(regs + 1)->addr = IMX304_FRAME_LENGTH_ADDR_MID;
	(regs + 1)->val = (frame_length >> 8) & 0xff;

	(regs + 2)->addr = IMX304_FRAME_LENGTH_ADDR_LSB;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void imx304_get_gain_reg(imx304_reg *regs,
				u16 gain)
{
	regs->addr = IMX304_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x01;

	(regs + 1)->addr = IMX304_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx304_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx304_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx304_write_table(struct imx304 *priv,
				const imx304_reg table[])
{
	struct tegracam_device *tc_dev = priv->tc_dev;
	struct device *dev = tc_dev->dev;
	int i = 0;
	int ret = 0;
	int retry;

	while (table[i].addr != IMX304_TABLE_END)
	{
		retry = 5;

		if(table[i].addr == IMX304_TABLE_WAIT_MS)
		{
			dev_dbg(dev, "%s: sleep %d\n", __func__, table[i].val);
			msleep(table[i].val);
			i++;
			continue;
		}

retry_sensor:
		ret = imx304_write_reg(priv->s_data, table[i].addr, table[i].val);
		if (ret)
		{
			retry--;
			if (retry > 0) {
				dev_warn(dev, "imx304_write_reg: try %d\n", retry);
				msleep(4);
				goto retry_sensor;
			}
			return -1;
		}
		/* ret = imx304_read_reg(priv->s_data, table[i].addr, &val); */
		/* dev_dbg(dev, "table[i].addr = 0x%x, val = 0x%x\n", table[i].addr, val); */
		i++;
	}

	return 0;
}

static int imx304_read_max929x(struct imx304 *priv,
				u8 slave, u16 addr, u8 *val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	u8 back;

	back = i2c_client->addr;
	i2c_client->addr = slave;
	imx304_read_reg(priv->s_data, addr, val);
	i2c_client->addr = back;

	return 0;
}

static int imx304_write_max929x(struct imx304 *priv,
				u8 slave, u16 addr, u8 val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	u8 back;

	back = i2c_client->addr;
	i2c_client->addr = slave;
	imx304_write_reg(priv->s_data, addr, val);
	i2c_client->addr = back;

	return 0;
}

static int imx304_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;
return 0;
	err = imx304_write_reg(s_data,
				IMX304_GROUP_HOLD_ADDR, val);
	if (err) {
		dev_dbg(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int imx304_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct imx304 *priv = (struct imx304 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx304_reg reg_list[2];
	u16 gain;
	int err, i;
return 0;
	gain = (u16)val;
	dev_dbg(dev, "%s:  gain reg: %d\n",  __func__, gain);

	imx304_get_gain_reg(reg_list, gain);

	for (i = 0; i < 2; i++) {
		err = imx304_write_reg(priv->s_data, reg_list[i].addr,
			reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int imx304_set_coarse_time(struct imx304 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	unsigned int cycle, high_time;
return 0;
	fpga_read_reg(FPGA_CYCLE_REG, &cycle);
	priv->frame_length = cycle;
	val = val > (cycle/2) ? (cycle/2) : val;

	high_time = priv->frame_length - val;
	dev_dbg(dev, "%s: FL %d, cycle %d, val %lld, high_time %d\n", __func__,
		priv->frame_length, cycle, val, high_time);

	fpga_write_reg(FPGA_EXPOSURE_REG, high_time);

	return 0;
}

static int imx304_set_coarse_time_hdr(struct imx304 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s: \n", __func__);

	return 0;
}

static int imx304_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx304 *priv = (struct imx304 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx304_reg reg_list[3];
	int err;
	u32 frame_length;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	struct v4l2_control control;
	int hdr_en;
	int i = 0;
return 0;
	frame_length = mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	priv->frame_length = frame_length;
	/* if (priv->frame_length > IMX304_MAX_FRAME_LENGTH) */
	/*	priv->frame_length = IMX304_MAX_FRAME_LENGTH; */

	dev_dbg(dev, "%s: val: %lld, , frame_length: %d\n", __func__,
		val, priv->frame_length);

	imx304_get_frame_length_regs(reg_list, priv->frame_length);

	for (i = 0; i < 3; i++) {
		err = imx304_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	/* check hdr enable ctrl */
	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if ((hdr_en == SWITCH_ON) && (priv->last_wdr_et_val != 0)) {
		err = imx304_set_coarse_time_hdr(priv, priv->last_wdr_et_val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time SHS1 SHS2 override\n", __func__);
	}

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx304_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx304 *priv = (struct imx304 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;
	struct v4l2_control control;
	int hdr_en;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);
return 0;
	/* check hdr enable ctrl */
	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if (hdr_en == SWITCH_ON) {
		err = imx304_set_coarse_time_hdr(priv, val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time SHS1 SHS2 override\n", __func__);
	} else {
		err = imx304_set_coarse_time(priv, val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time SHS1 override\n", __func__);
	}

	return err;
}

static struct tegracam_ctrl_ops imx304_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx304_set_gain,
	.set_exposure = imx304_set_exposure,
	.set_frame_rate = imx304_set_frame_rate,
	.set_group_hold = imx304_set_group_hold,
};

static int imx304_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	pw->state = SWITCH_ON;
	return 0;

}

static int imx304_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int imx304_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "extperiph1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parent = devm_clk_get(dev, "pllp_grtba");
	if (IS_ERR(parent))
		dev_err(dev, "devm_clk_get failed for pllp_grtba");
	else
		clk_set_parent(pw->mclk, parent);

	pw->state = SWITCH_OFF;
	return err;
}

static int imx304_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static struct camera_common_pdata *imx304_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!np)
		return NULL;

	match = of_match_device(imx304_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	return board_priv_pdata;
}

static int imx304_set_mode(struct tegracam_device *tc_dev)
{
	struct imx304 *priv = (struct imx304 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	bool limit_analog_gain = false;
	const struct of_device_id *match;
	struct i2c_client *i2c_client = priv->i2c_client;
	u8 slave = 0;
	int err;

	match = of_match_device(imx304_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	limit_analog_gain = of_property_read_bool(np, "limit_analog_gain");

	/* Reset sensor/sensor-fpga
	 *
	 * max9295 slave addr : slave
	 * register addr : 0x02be/0x02d3
	 * regiser value : 0x80(low level) / 0x90(high level)
	 *
	 * imx304 slave addr(MAX9296 link A) : 0x28
	 * max9295 slave addr(MAX9296 link A) : 0x60
	 *
	 * imx304 slave addr(MAX9296 link B) : 0x2a
	 * max9295 slave addr(MAX9296 link B) : 0x62
	 */
	if(i2c_client->addr == 0x28)
		slave = 0x60;
	else if(i2c_client->addr == 0x2a)
		slave = 0x62;

	imx304_write_max929x(priv, slave, 0x02d3, 0x80);
	msleep(50);

	imx304_write_max929x(priv, slave, 0x02be, 0x80);
	msleep(50);
	imx304_write_max929x(priv, slave, 0x02be, 0x90);
	msleep(100);

	err = imx304_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	if (limit_analog_gain) {
		err = imx304_write_reg(priv->s_data,
			IMX304_ANALOG_GAIN_LIMIT_ADDR,
			IMX304_ANALOG_GAIN_LIMIT_VALUE);
		if (err)
			return err;
	}

	imx304_write_max929x(priv, slave, 0x02d3, 0x90);

	return 0;
}

static int imx304_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx304 *priv = (struct imx304 *)tegracam_get_privdata(tc_dev);
	u8 value;
	int err;

	if (test_mode) {
		err = imx304_write_table(priv,
			mode_table[IMX304_MODE_TEST_PATTERN]);
		if (err)
			return err;
	}

	/*
	 * Enable MAX9296 PHYS
	 *
	 * max9296 slave addr : 0x48
	 * register addr : 0x0332
	 * regiser value : value
	 *
	 * imx304 slave addr(MAX9296 link A) : 0x28
	 * imx304 slave addr(MAX9296 link B) : 0x2a
	 */
#if 1
	imx304_read_max929x(priv, 0x48, 0x0332, &value);

	value = 0x20;
	imx304_write_max929x(priv, 0x48, 0x0332, value);
	
	msleep(1);
	value |= 0x40;	/* Enable MAX9296 link B PHY. */
	imx304_write_max929x(priv, 0x48, 0x0332, value);
	
	imx304_read_max929x(priv, 0x48, 0x0332, &value);
#else
	imx304_read_max929x(priv, 0x48, 0x0332, &value);
#endif

	err = imx304_write_table(priv,
		mode_table[IMX304_MODE_START_STREAM]);
	if (err)
		return err;

	return 0;
}

static int imx304_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx304 *priv = (struct imx304 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx304_write_table(priv, mode_table[IMX304_MODE_STOP_STREAM]);
	if (err)
		return err;

	/*
	 * Disable MAX9296 PHYS
	 *
	 * max9296 slave addr : 0x48
	 * register addr : 0x0332
	 * regiser value : value
	 *
	 * imx304 slave addr(MAX9296 link A) : 0x28
	 * imx304 slave addr(MAX9296 link B) : 0x2a
	 */
#if 0
	imx304_read_max929x(priv, 0x48, 0x0332, &value);
	if(i2c_client->addr == 0x28)
		value &= (~0x20);	/* Disable MAX9296 link A PHY. */
	else if(i2c_client->addr == 0x2a)
		value &= (~0x40);	/* Disable MAX9296 link B PHY. */
	imx304_write_max929x(priv, 0x48, 0x0332, value);
#endif

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline (10 us)
	 */
	usleep_range(priv->frame_length * 10, priv->frame_length * 10 + 1000);

	return 0;
}


static struct camera_common_sensor_ops imx304_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx304_frmfmt),
	.frmfmt_table = imx304_frmfmt,
	.power_on = imx304_power_on,
	.power_off = imx304_power_off,
	.write_reg = imx304_write_reg,
	.read_reg = imx304_read_reg,
	.parse_dt = imx304_parse_dt,
	.power_get = imx304_power_get,
	.power_put = imx304_power_put,
	.set_mode = imx304_set_mode,
	.start_streaming = imx304_start_streaming,
	.stop_streaming = imx304_stop_streaming,
};

static int imx304_board_setup(struct imx304 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx304_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

	imx304_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static int imx304_debugfs_gain_read(void *data, u64 *val)
{
	int err = 0;
	u8 gain = 0;
	struct imx304 *priv = data;
	struct i2c_client *client = priv->i2c_client;

	err = imx304_read_reg(priv->s_data,
				IMX304_GAIN_ADDR_LSB, &gain);
	if (err) {
		dev_err(&client->dev, "%s: error get sensor gain LSB val\n",
			__func__);
		return err;
	}

	*val = gain & 0xff;

	err = imx304_read_reg(priv->s_data,
				IMX304_GAIN_ADDR_MSB, &gain);
	if (err) {
		dev_err(&client->dev, "%s: error get sensor gain MSB val\n",
			__func__);
		return err;
	}

	*val |= (gain & 0x1) << 8;

	return 0;
}

static int imx304_debugfs_gain_write(void *data, u64 val)
{
	int err = 0;
	struct imx304 *priv = data;
	struct i2c_client *client = priv->i2c_client;

	if ((val < 0) || (val > 480)) {
		dev_err(&client->dev, "%s: gain val is not in range\n", __func__);
		return -EFAULT;
	}

	err = imx304_write_reg(priv->s_data,
				IMX304_GAIN_ADDR_MSB, ((val >> 8) & 0x1));
	if (err) {
		dev_err(&client->dev, "%s: error setting sensor MSB gain\n",
			__func__);
		return err;
	}

	err = imx304_write_reg(priv->s_data,
				IMX304_GAIN_ADDR_LSB, (val & 0xff));
	if (err) {
		dev_err(&client->dev, "%s: error setting sensor LSB gain\n",
			__func__);
		return err;
	}

	return err;

}

DEFINE_SIMPLE_ATTRIBUTE(imx304_debugfs_gain_fops,
	imx304_debugfs_gain_read,
	imx304_debugfs_gain_write,
	"%lld\n");

static void imx304_debugfs_remove(struct imx304 *priv);

static int imx304_debugfs_create(struct imx304 *priv)
{
	int err = 0;
	struct i2c_client *client = priv->i2c_client;
	const char *devnode;
	char debugfs_dir[16];

	err = of_property_read_string(client->dev.of_node, "devnode", &devnode);
	if (err) {
		dev_err(&client->dev, "devnode not in DT\n");
		return err;
	}
	snprintf(debugfs_dir, sizeof(debugfs_dir), "camera-%s", devnode);

	priv->debugfs_dir = debugfs_create_dir(debugfs_dir, NULL);
	if (priv->debugfs_dir == NULL)
		return -ENOMEM;

	if (!debugfs_create_file("gain", 0644, priv->debugfs_dir, priv,
			&imx304_debugfs_gain_fops))
		goto error;

	return 0;

error:
	imx304_debugfs_remove(priv);

	return -ENOMEM;
}

static void imx304_debugfs_remove(struct imx304 *priv)
{
	debugfs_remove_recursive(priv->debugfs_dir);
	priv->debugfs_dir = NULL;
}


static int imx304_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx304_subdev_internal_ops = {
	.open = imx304_open,
};

static int imx304_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx304 *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx304), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx304", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx304_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx304_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx304_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx304_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	err = imx304_debugfs_create(priv);
	if (err) {
		dev_err(&client->dev, "error creating debugfs interface");
		imx304_debugfs_remove(priv);
		return err;
	}

	dev_info(dev, "Detected IMX304 sensor\n");

	return 0;
}

static int
imx304_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx304 *priv = (struct imx304 *)s_data->priv;

	imx304_debugfs_remove(priv);
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx304_id[] = {
	{ "imx304", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx304_id);

static struct i2c_driver imx304_i2c_driver = {
	.driver = {
		.name = "imx304",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx304_of_match),
	},
	.probe = imx304_probe,
	.remove = imx304_remove,
	.id_table = imx304_id,
};

module_i2c_driver(imx304_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX304");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
