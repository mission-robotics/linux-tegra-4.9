/*
 * tis_imx390.c - imx390 sensor driver
 *
 * Copyright (c) 2019, The Imaging Source Europe GmbH.  All rights reserved.
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

#include "tis_imx415.h"
#include "tis_imx415_mode_tbls.h"

#include "tis_sensor.h"
#include "tis_sensor_ctrl.h"

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

struct tis_imx415_priv
{
	int binning_factor;
};

#define TIS_IMX415_PRIV(tis_sensor) ((struct tis_imx415_priv*)tis_sensor->priv)

static struct of_device_id tis_imx415_of_match[] = {
	{ .compatible = "tis,tis_imx415", },
	{ },
};

MODULE_DEVICE_TABLE(of, tis_imx415_of_match);

static int tis_imx415_set_mode(struct tegracam_device *tc_dev);
static int tis_imx415_start_streaming(struct tegracam_device *tc_dev);
static int tis_imx415_stop_streaming(struct tegracam_device *tc_dev);
static int tis_imx415_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int tis_imx415_set_gain(struct tegracam_device *tc_dev, s64 val);
static int tis_imx415_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int tis_imx415_set_group_hold(struct tegracam_device *tc_dev, bool val);
static struct camera_common_pdata *tis_imx415_parse_dt(struct tegracam_device *tc_dev);

static const struct regmap_config tis_imx415_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = false,
};

static struct camera_common_sensor_ops tis_imx415_common_ops = {
	//.numfrmfmts = ARRAY_SIZE(tis_imx390_frmfmt),
	//.frmfmt_table = tis_imx390_frmfmt,
	.power_on = tis_imx_fake_power_on,
	.power_off = tis_imx_fake_power_off,
	.write_reg = tis_imx_write_8,
	.read_reg = tis_imx_read_8,
	.parse_dt = tis_imx415_parse_dt,
	.power_get = tis_imx_power_get,
	.power_put = tis_imx_power_put,
	.set_mode = tis_imx415_set_mode,
	.start_streaming = tis_imx415_start_streaming,
	.stop_streaming = tis_imx415_stop_streaming,
};

static const u32 tis_imx415_ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

static struct tegracam_ctrl_ops tis_imx415_ctrl_ops = {
	.numctrls = ARRAY_SIZE(tis_imx415_ctrl_cid_list),
	.ctrl_cid_list = tis_imx415_ctrl_cid_list,
	.set_gain = tis_imx415_set_gain,
	.set_exposure = tis_imx415_set_exposure,
	.set_frame_rate = tis_imx415_set_frame_rate,
	.set_group_hold = tis_imx415_set_group_hold,
};

static int tis_imx415_write_table(struct tis_sensor *priv, const struct reg_8 table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap,
					 table,
					 NULL, 0,
					 TIS_IMX415_TABLE_WAIT_MS,
					 TIS_IMX415_TABLE_END);
}

static int tis_imx415_apply_exposure_fps(struct tis_sensor *priv)
{
	struct device *dev = priv->dev;
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	
	const u32 HMAX = mode->image_properties.line_length;
	const u64 PIXEL_CLOCK = priv->pixel_clock;

	const u32 VMAX_MIN = max(priv->mode_active_h + 46, 1222u);
	const u32 VMAX_MAX = 0xFFFFF;

	const u64 exposure_factor = mode->control_properties.exposure_factor;
	const u64 framerate_factor = mode->control_properties.framerate_factor;

	u32 fps_defined_vmax_unbounded = (PIXEL_CLOCK * framerate_factor) / (priv->fps * HMAX);
	u32 fps_defined_vmax = clamp(fps_defined_vmax_unbounded, VMAX_MIN, VMAX_MAX);

	const u64 S_TO_NS = 1000000000;
	
    const u64 h_period_ns = HMAX * S_TO_NS / PIXEL_CLOCK;
    const u64 exposure_offset_ns = 2680; // 1790 in 10-bit mode

	u64 exposure_time_ns = priv->exposure_time * S_TO_NS / exposure_factor;
	s64 adjusted_exposure_ns = (exposure_time_ns > exposure_offset_ns) ? (exposure_time_ns - exposure_offset_ns) : 0;

	u32 exposure_lines = max( (u32)(adjusted_exposure_ns / h_period_ns), 4u );

    u32 vmax = max(exposure_lines + 8, fps_defined_vmax);
    u32 shr0 = vmax - exposure_lines;

	int err = tis_imx_write_24(priv->s_data, TIS_IMX415_REG_SHR0, shr0);
	err = tis_imx_write_24(priv->s_data, TIS_IMX415_REG_VMAX, vmax);
	err = tis_imx_write_24(priv->s_data, TIS_IMX415_REG_HMAX, HMAX);

	dev_info(dev, "%s: exposure_time = %lld, fps = %lld\n", __func__, priv->exposure_time, priv->fps);
	dev_dbg(dev, "%s: h_period = %lld, exposure_offset = %lld, adjusted_exposure = %lld, exposure_lines = %d", __func__, h_period_ns, exposure_offset_ns, adjusted_exposure_ns, exposure_lines);
	dev_dbg(dev, "%s: shr0 = %d vmax = %d hmax = %d", __func__, shr0, vmax, HMAX);

	return err;
}
static int tis_imx415_apply_roi_position(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;

	const int SENSOR_WIDTH = priv->SENSOR_WIDTH;
	const int SENSOR_HEIGHT = priv->SENSOR_HEIGHT;

	int active_w = priv->mode_active_w * TIS_IMX415_PRIV(priv)->binning_factor;
	int active_h = priv->mode_active_h * TIS_IMX415_PRIV(priv)->binning_factor;
	int offset_x = priv->ctrl.offset_x;
	int offset_y = priv->ctrl.offset_y;	

	// Make sure we don't overflow to the right/bottom
	offset_x = min(offset_x, SENSOR_WIDTH - active_w);
	offset_y = min(offset_y, SENSOR_HEIGHT - active_h);

	tis_imx_write_16(s_data, TIS_IMX415_REG_PIX_HST, offset_x);
	return tis_imx_write_16(s_data, TIS_IMX415_REG_PIX_VST, offset_y * 2);
}
static int tis_imx415_apply_roi(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;

	int active_w = priv->mode_active_w * TIS_IMX415_PRIV(priv)->binning_factor;
	int active_h = priv->mode_active_h * TIS_IMX415_PRIV(priv)->binning_factor;

	tis_imx_write_16(s_data, TIS_IMX415_REG_PIX_HWIDTH, active_w);
	tis_imx_write_16(s_data, TIS_IMX415_REG_PIX_VWIDTH, active_h * 2);

	tis_imx415_apply_roi_position(priv);

	return tis_imx415_apply_exposure_fps(priv);	
}
static int tis_imx415_apply_gain(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;

	u16 reg_val = priv->gain / 3;

	return tis_imx_write_16(s_data, TIS_IMX415_REG_GAIN_PGC_0, reg_val);
}
static int tis_imx415_apply_offsets(struct tis_sensor *priv)
{
	tis_sensor_ctrl_do_offset_auto_center(priv);

	if( !tis_sensor_is_power_on(priv) )
		return 0;

	return tis_imx415_apply_roi_position(priv);
}
static int tis_imx415_apply_black_level(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;

	return tis_imx_write_16(s_data, TIS_IMX415_REG_BLKLEVEL, priv->ctrl.black_level);
}
static int tis_imx415_apply_ext_sync(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;

	switch( priv->ctrl.ext_sync )
	{		
		case TIS_SENSOR_CTRL_EXT_SYNC_MASTER:
			tis_imx_write_8(s_data, TIS_IMX415_REG_XVS_XHS_DRV, 0x00);
			return tis_imx_write_8(s_data, TIS_IMX415_REG_EXTMODE, 0x00);
			break;
		case TIS_SENSOR_CTRL_EXT_SYNC_SLAVE:
			tis_imx_write_8(s_data, TIS_IMX415_REG_XVS_XHS_DRV, 0x03);
			return tis_imx_write_8(s_data, TIS_IMX415_REG_EXTMODE, 0x01);		
		case TIS_SENSOR_CTRL_EXT_SYNC_OFF:
		default:
			tis_imx_write_8(s_data, TIS_IMX415_REG_XVS_XHS_DRV, 0x03);
			return tis_imx_write_8(s_data, TIS_IMX415_REG_EXTMODE, 0x00);
	}
}

static int tis_imx415_set_mode(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int err;
	u32 mode_lanes, mode_lane_mbps, mode_bpp, mclk_khz;

	dev_info(dev, "set mode %d", s_data->mode_prop_idx);

	err = tis_sensor_ctrl_do_lazy_init(priv);
	if( err )
	{
		dev_err(dev, "%s: do_lazy_init failed: %d", __func__, err);
		return err;
	}

	tis_sensor_init_defaults(priv, mode);

	tis_imx_get_mode_from_dt(tc_dev, s_data->mode_prop_idx, &priv->mode_active_w, &priv->mode_active_h, &mode_lanes, &mode_lane_mbps, &mode_bpp, &mclk_khz);
	dev_info(dev, "active_w: %d, active_h: %d", priv->mode_active_w, priv->mode_active_h);
	dev_info(dev, "mode_lanes: %d, mode_bpp: %d", mode_lanes, mode_bpp);
	dev_info(dev, "mclk_khz: %d", mclk_khz);

	priv->pixel_clock = 74250000ull * mclk_khz * 1000 / 37250000;

	TIS_IMX415_PRIV(priv)->binning_factor = 1; // (s_data->mode_prop_idx == 2) ? 2 : 1; // (priv->mode_active_w < 2000) ? 2 : 1;
	if( TIS_IMX415_PRIV(priv)->binning_factor == 2 )
	{
		// Subtract dummy pixels
		priv->mode_active_w -= 12;
		priv->mode_active_h -= 1;
	}

	err = tis_imx415_write_table(priv, tis_imx415_all_pixel_12bit_4lane);
	if (err)
	{
		dev_err(dev, "%s: error writing mode settings", __func__);
		return err;
	}

	switch( mode_lane_mbps )
	{
		case 2500:
			err = tis_imx415_write_table(priv, tis_imx415_37MHz_2376Mbps);
			break;
		case 1500:
			err = tis_imx415_write_table(priv, tis_imx415_37MHz_1485Mbps);
			break;
		default:
			dev_warn(dev, "%s: Unexpected mode_lane_mbps (%d), falling back to 800; high frame rates might not work)", __func__, mode_lane_mbps);
		case 800:		
			err = tis_imx415_write_table(priv, tis_imx415_37MHz_891Mbps);
			break;
	}

	if (err)
	{
		dev_err(dev, "%s: error writing clock settings", __func__);
		return err;
	}

	if( mode_lanes == 4 )
	{
		tis_imx_write_8(s_data, TIS_IMX415_REG_LANEMODE, 0x03);
	}
	else if( mode_lanes == 2 )
	{
		tis_imx_write_8(s_data, TIS_IMX415_REG_LANEMODE, 0x01);
	}
	else
	{
		dev_err(dev, "%s: Unexpected mode_lanes %d", __func__, mode_lanes);
		return -EINVAL;
	}

	if( mode_bpp == 10 )
	{
		tis_imx_write_8(s_data, TIS_IMX415_REG_ADBIT, 0x00);
		tis_imx_write_8(s_data, TIS_IMX415_REG_MDBIT, 0x00);
	}
	else if( mode_bpp == 12 )
	{
		// Init script already set 12 bpp
	}
	else
	{
		dev_err(dev, "%s: Unexpected mode_bpp %d", __func__, mode_bpp);
		return -EINVAL;
	}

	if( TIS_IMX415_PRIV(priv)->binning_factor == 2 )
	{
		tis_imx_write_8(s_data, TIS_IMX415_REG_HADD, 0x01);
		tis_imx_write_8(s_data, TIS_IMX415_REG_VADD, 0x01);
		tis_imx_write_8(s_data, TIS_IMX415_REG_ADDMODE, 0x01);

		tis_imx_write_8(s_data, 0x30D9, 0x02);
		tis_imx_write_8(s_data, 0x30DA, 0x01);
	}
	else
	{
		tis_imx_write_8(s_data, TIS_IMX415_REG_HADD, 0x00);
		tis_imx_write_8(s_data, TIS_IMX415_REG_VADD, 0x00);
		tis_imx_write_8(s_data, TIS_IMX415_REG_ADDMODE, 0x00);

		tis_imx_write_8(s_data, 0x30D9, 0x06);
		tis_imx_write_8(s_data, 0x30DA, 0x02);
	}
	
	tis_sensor_init_defaults(priv, mode);
	tis_sensor_ctrl_do_offset_auto_center(priv);
	tis_imx415_apply_roi(priv);
	tis_imx415_apply_gain(priv);
	tis_imx415_apply_black_level(priv);
	tis_imx415_apply_ext_sync(priv);

    return 0;
}
static int tis_imx415_start_streaming(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	int err;

	dev_info(dev, "%s", __func__);

	err = tis_imx415_write_table(priv, tis_imx415_start);
	if (err)
	{
		dev_err(dev, "%s: error setting stream", __func__);
		return err;
	}

	return 0;
}
static int tis_imx415_stop_streaming(struct tegracam_device *tc_dev)
{
    struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	int err;

	dev_info(dev, "%s", __func__);

	err = tis_imx415_write_table(priv, tis_imx415_stop);
	if (err)
	{
		dev_err(dev, "%s: error setting stream", __func__);
		// Don't fail stop_streaming; if this does not work we were likely disconnected
	}

	return 0;
}

static int tis_imx415_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);

	priv->exposure_time = val;

    return tis_imx415_apply_exposure_fps(priv);
}
static int tis_imx415_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);

	priv->gain = clamp(val, 0ll, 300ll);

    return tis_imx415_apply_gain(priv);
}
static int tis_imx415_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);

	priv->fps = val;
	
    return tis_imx415_apply_exposure_fps(priv);
}
static int tis_imx415_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    return 0;
}
static struct camera_common_pdata *tis_imx415_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!np)
		return NULL;

	match = of_match_device(tis_imx415_of_match, dev);
	if (!match) {
		dev_err(dev, " Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(dev, "Failed to allocate pdata\n");
		return NULL;
	}

	err = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev, "mclk name not present, "
			"assume sensor driven externally\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(np,
			"reset-gpios", 0);

	dev_info(dev, "reset-gpio: %d\n", board_priv_pdata->reset_gpio);

	board_priv_pdata->pwdn_gpio = of_get_named_gpio(np,
			"pwdn-gpios", 0);

/*	of_property_read_string(node, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	of_property_read_string(node, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);*/
	of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);

	return board_priv_pdata;
}

struct tis_sensor_ctrl_ops tis_imx415_sensor_ctrl_ops_template = {	
	.sensor_apply_gpout_mode = tis_sensor_apply_gpout_mode,
	.sensor_apply_gpout_function = tis_sensor_apply_gpout_function,	
	.sensor_apply_offsets = tis_imx415_apply_offsets,
	.sensor_apply_black_level = tis_imx415_apply_black_level,
	.sensor_apply_ext_sync = tis_imx415_apply_ext_sync,
};


static int tis_imx415_update_common_ctrl_config(struct tis_sensor *priv, struct tis_sensor_ctrl_config *config)
{
	struct device *dev = priv->dev;
	const char *outs;
	int err;

	struct tis_sensor_ctrl_ops *ctrl_ops = devm_kmemdup(dev, &tis_imx415_sensor_ctrl_ops_template, sizeof(tis_imx415_sensor_ctrl_ops_template), GFP_KERNEL);
	if( !ctrl_ops )
	{
		return -ENOMEM;			
	}	

	config->offset_x_step = 12;
	config->offset_y_step = 4;

	config->black_level_min = 0;
	config->black_level_max = 0x3FF;
	config->black_level_default = 0x032;

	err = of_property_read_string(dev->of_node, "fpdlink_signals", &outs);
	if( !err )
	{
		if( !strcmp(outs, "master") )
		{
			// Camera configured as master cannot be slave
			config->ext_sync_skip_mask = (1 << TIS_SENSOR_CTRL_EXT_SYNC_SLAVE);
		}
		else // if( !strcmp(outs, "default") )
		{
			// Camera configured as slave cannot be master
			config->ext_sync_skip_mask = (1 << TIS_SENSOR_CTRL_EXT_SYNC_MASTER);

			// Slave can select trigger source
			ctrl_ops->sensor_apply_trigger_source = tis_sensor_apply_trigger_source;
		}
	}
	else
	{
		// Camera without FPD-Link cannot be master (would need to set pin 21)
		// Might be possible with future adapter boards, or custom hardware
		config->ext_sync_skip_mask = (1 << TIS_SENSOR_CTRL_EXT_SYNC_MASTER);

		// Slave can select trigger source
		ctrl_ops->sensor_apply_trigger_source = tis_sensor_apply_trigger_source;
	}

	config->ops = ctrl_ops;

	return 0;
}

static struct tis_sensor_ops tis_imx415_sensor_ops =
{
	.sensor_power_on = tis_imx_do_power_on,
	.sensor_update_common_ctrl_config = tis_imx415_update_common_ctrl_config,
};


static int tis_imx415_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tis_sensor *priv;
	struct camera_common_sensor_ops *sensor_ops;
	int err;

	pr_info("[tis_imx415]: probing sensor, driver ver %s\n", __stringify(TIS_DRIVERS_VERSION));

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	if (!tis_sensor_is_sensor_model(dev, node, "imx415") )
	{
		dev_err(dev, "%s: Unexpected sensor_model", __func__);
		return -EINVAL;
	}

	if (of_get_named_gpio(node, "reset-gpios", 0) < 0) {
		dev_info(dev, "GPIO not available! Deferring probe.\n");
		return -EPROBE_DEFER;
	}

	priv = tis_sensor_create(client, 3864, 2192, TIS_SENSOR_MODEL_IMX415, sizeof(struct tis_imx415_priv), &tis_imx415_sensor_ops);
	if( IS_ERR(priv) )
	{
		return PTR_ERR(priv);
	}

	sensor_ops = tis_sensor_tegracam_build_sensor_ops(priv, &tis_imx415_common_ops);
	if( IS_ERR(sensor_ops) )
	{
		return PTR_ERR(sensor_ops);
	}

	err = tis_sensor_tegracam_device_register(priv, &tis_imx415_regmap_config, &tis_imx415_ctrl_ops, sensor_ops, "tis_imx415" );
	if (err != 0)
	{
		return err;
	}

	err = tis_sensor_ctrl_init(priv);

	tis_sensor_register_debug_imx(priv);

	return 0;
}


static int
tis_imx415_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct tis_sensor*priv = (struct tis_sensor*)s_data->priv;

	tis_sensor_remove(priv);

	return 0;
}

static const struct i2c_device_id tis_imx415_id[] = {
	{ "tis_imx415", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tis_imx415_id);

static struct i2c_driver tis_imx415_i2c_driver = {
	.driver = {
		.name = "tis_imx415",
		.owner = THIS_MODULE,
	},
	.probe = tis_imx415_probe,
	.remove = tis_imx415_remove,
	.id_table = tis_imx415_id,
};

TIS_SENSOR_DRIVER_MODULE(tis_imx415_i2c_driver)

MODULE_DESCRIPTION("Media Controller driver for Sony TIS_IMX415");
MODULE_AUTHOR("Arne Caspari <arne.caspari@theimagingsource.com>");
MODULE_LICENSE("GPL v2");
