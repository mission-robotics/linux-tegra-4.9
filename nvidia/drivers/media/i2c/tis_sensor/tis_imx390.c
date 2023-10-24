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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>

#include "tis_sensor.h"
#include "tis_imx390.h"

#include "tis_imx390_mode_tbls.h"

#include "tis_sensor_ctrl.h"

static const struct of_device_id tis_imx390_of_match[] = {
	{ .compatible = "tis,tis_imx390", },
	{ },
};

MODULE_DEVICE_TABLE(of, tis_imx390_of_match);

static int tis_imx390_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int tis_imx390_set_gain(struct tegracam_device *tc_dev, s64 val);
static int tis_imx390_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int tis_imx390_set_group_hold(struct tegracam_device *tc_dev, bool val);
static struct camera_common_pdata *tis_imx390_parse_dt(struct tegracam_device *tc_dev);
static int tis_imx390_set_output_sel(struct tis_sensor *priv, int val);

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = false,
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

static const struct tegracam_ctrl_ops tis_imx390_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = tis_imx390_set_gain,
	.set_exposure = tis_imx390_set_exposure,
	.set_frame_rate = tis_imx390_set_frame_rate,
	.set_group_hold = tis_imx390_set_group_hold,
};

static const char* tis_imx390_output_sel_items[] =
{
	"HDR",
	"SP1_HCG",
	"SP1_LCG",
	"SP2"
};

static int tis_imx390_write_table(struct tis_sensor *priv,
				const tis_imx390_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 TIS_IMX390_TABLE_WAIT_MS,
					 TIS_IMX390_TABLE_END);
}

static int tis_imx390_set_window_cropping(struct tis_sensor *priv,
                                          int active_w, int active_h,
										  int offset_x, int offset_y)
{
	struct camera_common_data *s_data = priv->s_data;

	int subtract_smpg_height = TIS_IMX390_PRIV(priv)->ctrl.subtract_smpg_height->cur.val ? 4 : 0;

	if( offset_x < 0 )
		offset_x = (priv->SENSOR_WIDTH - active_w) / 2;
	if( offset_y < 0 )
		offset_y = (priv->SENSOR_HEIGHT - active_h) / 2;

	tis_imx_write_8( s_data, TIS_IMX390_REG_APPLICATION_LOCK, 0x01); // SM_CROP_ON_APL

	tis_imx_write_8( s_data, TIS_IMX390_REG_CROP_ON, 0x01 );
	tis_imx_write_16( s_data, TIS_IMX390_REG_CROP_H_SIZE, active_w );
	tis_imx_write_16( s_data, TIS_IMX390_REG_CROP_V_SIZE, active_h - subtract_smpg_height );
	tis_imx_write_16( s_data, TIS_IMX390_REG_CROP_H_OFFSET, offset_x );
	tis_imx_write_16( s_data, TIS_IMX390_REG_CROP_V_OFFSET, offset_y );

	return 0;
}

static bool is_full_resolution_mode(struct tis_sensor *priv)
{
	return priv->mode_active_h >= 1100;	
}

static int tis_imx390_set_mode(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int err;

	dev_info(dev, "set mode %d", s_data->mode_prop_idx);

	err = tis_sensor_ctrl_do_lazy_init(priv);
	if( err )
	{
		dev_err(dev, "%s: do_lazy_init failed: %d", __func__, err);
		return err;
	}

	tis_imx_get_mode_from_dt(tc_dev, s_data->mode_prop_idx, &priv->mode_active_w, &priv->mode_active_h, &priv->mode_lanes, NULL, NULL, NULL);
	dev_info(dev, "active_w: %d, active_h: %d, lanes: %d", priv->mode_active_w, priv->mode_active_h, priv->mode_lanes);

	err = tis_imx390_write_table(priv, tis_imx390_mode_init);
	if (err)
		return err;
	
	if( priv->mode_lanes == 2 )
	{
		if (is_full_resolution_mode(priv))
		{
			err = tis_imx390_write_table(priv, tis_imx390_mode_1920x1200_27mhz_25fps_2lane);
		}
		else
		{
			err = tis_imx390_write_table(priv, tis_imx390_mode_1920x1080_27mhz_30fps_2lane);
		}
	}
	else // priv->mode_lanes == 4
	{
		if (is_full_resolution_mode(priv))
		{
			err = tis_imx390_write_table(priv, tis_imx390_mode_1920x1200_27mhz_50fps_4lane);			
		}
		else
		{
			err = tis_imx390_write_table(priv, tis_imx390_mode_1920x1080_27mhz_60fps_4lane);
		}		
	}
	if (err)
		return err;

#define FOURCC_PWL2 0x324c5750 // 'PWL2'
#define FOURCC_PWL3 0x334c5750 // 'PWL3'
	if( priv->raw_fourcc[s_data->mode_prop_idx] == FOURCC_PWL2 || priv->raw_fourcc[s_data->mode_prop_idx] == FOURCC_PWL3 )
	{		
		// Enable PWL compression
		tis_imx_write_8( s_data, TIS_IMX390_PWL_THRU, 0x00 );		
	}
	else
	{
		// Disable PWL compression
		tis_imx_write_8( s_data, TIS_IMX390_PWL_THRU, 0x01 );
	}	

	tis_imx390_set_window_cropping(priv, priv->mode_active_w, priv->mode_active_h, -1, -1);

	tis_imx390_set_output_sel( priv, TIS_IMX390_PRIV(priv)->ctrl.output_sel->cur.val );

	tis_sensor_init_defaults(priv, mode);

	tis_imx390_set_gain( tc_dev, priv->gain );
	tis_imx390_set_exposure( tc_dev, priv->exposure_time );

	return 0;
}

static int tis_imx390_start_streaming(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	err = tis_imx390_write_table(priv, tis_imx390_start);
	if (err)
	{
		dev_dbg(dev, "%s: error setting stream\n", __func__);
		return err;
	}

	return 0;
}

static int tis_imx390_stop_streaming(struct tegracam_device *tc_dev)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	dev_info(dev, "%s\n", __func__);

	err = tis_imx390_write_table(priv, tis_imx390_stop);
	if (err)
	{
		dev_dbg(dev, "%s: error setting stream\n", __func__);
		// Don't fail stop_streaming; if this does not work we were likely disconnected
	}

	return 0;
}

static struct camera_common_sensor_ops tis_imx390_common_ops = {
	.numfrmfmts = ARRAY_SIZE(tis_imx390_frmfmt),
	.frmfmt_table = tis_imx390_frmfmt,
	.power_on = tis_imx_fake_power_on,
	.power_off = tis_imx_fake_power_off,
	.write_reg = tis_imx_write_8,
	.read_reg = tis_imx_read_8,
	.parse_dt = tis_imx390_parse_dt,
	.power_get = tis_imx_power_get,
	.power_put = tis_imx_power_put,
	.set_mode = tis_imx390_set_mode,
	.start_streaming = tis_imx390_start_streaming,
	.stop_streaming = tis_imx390_stop_streaming,
};

static int tis_imx390_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;	

	err = tis_imx_write_8(s_data,
					TIS_IMX390_REGHOLD_ADDR, val);
	if (err)
	{
		dev_err(dev,
		 "%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int tis_imx390_apply_gain(struct tis_sensor *priv, int gain, int gain_offset)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = priv->dev;
	int lcg, hcg;
	int lcg_reg, hcg_reg;	

	hcg = gain + 30;
	lcg = clamp(gain + 30 + gain_offset, 30, 300);

	hcg_reg = hcg / 3;
	lcg_reg = lcg / 3;

	dev_info(dev, "%s: val = %d, hcg_reg = %d, lcg_reg = %d", __func__, gain, hcg_reg, lcg_reg);

	tis_imx_write_8(s_data, TIS_IMX390_AGAIN_SP1L, lcg_reg);
	tis_imx_write_8(s_data, TIS_IMX390_AGAIN_SP1H, hcg_reg);

	return 0;
}

static int tis_imx390_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);

	int gain_offset = TIS_IMX390_PRIV(priv)->ctrl.gain_offset->cur.val;

	priv->gain = val;

	return tis_imx390_apply_gain(priv, val, gain_offset);
}

static int tis_imx390_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{	
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;

	dev_info(dev, "set frame rate: %lld\n", val);

	priv->fps = val;

	return tis_imx390_set_exposure(tc_dev, priv->exposure_time);
}

struct IMX390_SENSOR_TIMING
{
	const u64 PIXEL_CLOCK;
	const u32 HMAX;
	const u32 VMAX;		
};

static struct IMX390_SENSOR_TIMING tis_imx390_get_sensor_timing(struct tis_sensor *priv)
{
	struct IMX390_SENSOR_TIMING timing_1200_2lane = { 123750000, 3300, 1500 };
	struct IMX390_SENSOR_TIMING timing_1200_4lane = { 148500000, 2376, 1250 };
	struct IMX390_SENSOR_TIMING timing_1080_2lane = { 148500000, 4400, 1125 };
	struct IMX390_SENSOR_TIMING timing_1080_4lane = { 148500000, 2200, 1125 };

	if (is_full_resolution_mode(priv))
	{
		if (priv->mode_lanes == 2)
		{			
			return timing_1200_2lane;
		}
		else // (priv->mode_lanes == 4)
		{
			return timing_1200_4lane;
		}
	}
	else
	{
		if (priv->mode_lanes == 2)
		{
			return timing_1080_2lane;
		}
		else // (priv->mode_lanes == 4)
		{
			return timing_1080_4lane;
		}
	}
}

static int tis_imx390_apply_exposure_fps(struct tis_sensor *priv, s64 exposure_time, u32 exposure_factor, s64 frame_rate, u32 framerate_factor, u32 exposure_ratio)
{
	struct camera_common_data *s_data = priv->s_data;

	const struct IMX390_SENSOR_TIMING timing = tis_imx390_get_sensor_timing(priv);
	const u64 S_TO_NS = 1000000000ll;	

	const u64 h_period_ns = (timing.HMAX * S_TO_NS) / timing.PIXEL_CLOCK;
	u32 shs1_exposure_lines = max((u32)((exposure_time * S_TO_NS) / (h_period_ns * exposure_factor)), 1u);
	u32 shs2_exposure_lines = max((u32)((exposure_time * S_TO_NS * exposure_ratio) / (h_period_ns * exposure_factor * 100)), 1u);
	
	u32 fps_defined_vmax = (u32)((timing.PIXEL_CLOCK * framerate_factor) / (priv->fps * timing.HMAX));
	int fmax = (max(shs1_exposure_lines, fps_defined_vmax) - 1) / timing.VMAX;

	u32 shs1 = timing.VMAX * (fmax + 1) - shs1_exposure_lines;
	u32 shs2 = timing.VMAX * (fmax + 1) - shs2_exposure_lines;

	tis_imx_write_16(s_data, TIS_IMX390_REG_MODE_HMAX, timing.HMAX);
	tis_imx_write_24(s_data, TIS_IMX390_REG_MODE_VMAX, timing.VMAX);
	tis_imx_write_8(s_data, TIS_IMX390_REG_FMAX, fmax);
	tis_imx_write_24(s_data, TIS_IMX390_REG_SHS1, shs1);
	tis_imx_write_24(s_data, TIS_IMX390_REG_SHS2, shs2);

	dev_info(priv->dev, "%s: HMAX = %d VMAX = %d FMAX = %d SHS1 = %d SHS2 = %d", __func__, timing.HMAX, timing.VMAX, fmax, shs1, shs2 );

	return 0;
}

static int tis_imx390_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	const struct sensor_mode_properties *mode =
		&priv->s_data->sensor_props.sensor_modes[priv->s_data->mode_prop_idx];

	int exposure_ratio = TIS_IMX390_PRIV(priv)->ctrl.exposure_ratio->cur.val;

	dev_info(priv->dev, "set exposure time: %lld (ratio = %d/100)\n", val, exposure_ratio);

	priv->exposure_time = val;

	return tis_imx390_apply_exposure_fps(priv, val, mode->control_properties.exposure_factor, priv->fps, mode->control_properties.framerate_factor, exposure_ratio);
}


static struct camera_common_pdata *tis_imx390_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!np)
		return NULL;

	match = of_match_device(tis_imx390_of_match, dev);
	if (!match) {
		dev_err(dev, " Failed to find matching dt id '%s'\n", tis_imx390_of_match[0].compatible);
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

static const struct media_entity_operations tis_imx390_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static int tis_imx390_set_output_sel(struct tis_sensor *priv, int val)
{
	struct camera_common_data *s_data = priv->s_data;

	if( val == 0 )
	{
		// Enable HDR processing
		return tis_imx_write_8(s_data, TIS_IMX390_WDC_OUTSEL, 0x01);
	}
	else
	{
		// Disable HDR processing
		tis_imx_write_8(s_data, TIS_IMX390_WDC_OUTSEL, 0x00);
		// Select output mode
		return tis_imx_write_8(s_data, TIS_IMX390_WDC_THR_FRM_SEL, val - 1);
	}	
}

static int tis_imx390_v4l2_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tis_sensor* priv = (struct tis_sensor*)ctrl->priv;
	const struct sensor_mode_properties *mode =
		&priv->s_data->sensor_props.sensor_modes[priv->s_data->mode_prop_idx];

	dev_dbg(priv->tc_dev->dev, "%s: id = %x val = %d", __func__, ctrl->id, ctrl->val);

	if( !tis_sensor_is_power_on(priv) )
		return 0;

	switch( ctrl->id )
	{
		case TIS_SENSOR_CID_IMX390_SENSOR_OUTPUT_SELECT:
			return tis_imx390_set_output_sel(priv, ctrl->val);
		case TIS_SENSOR_CID_IMX390_EXPOSURE_RATIO:
			return tis_imx390_apply_exposure_fps(priv, priv->exposure_time, mode->control_properties.exposure_factor, priv->fps, mode->control_properties.framerate_factor, ctrl->val);
		case TIS_SENSOR_CID_IMX390_GAIN_OFFSET:
			return tis_imx390_apply_gain(priv, priv->gain, ctrl->val);
		case TIS_SENSOR_CID_IMX390_SUBTRACT_SMPG_HEIGHT:
			return tis_imx390_set_window_cropping(priv, priv->mode_active_w, priv->mode_active_h, -1, -1);
	}
	return -EINVAL;
}

const struct v4l2_ctrl_ops tis_imx390_v4l2_ctrl_ops = {
	.s_ctrl = tis_imx390_v4l2_s_ctrl
};

const struct tis_sensor_ctrl_ops tis_imx390_sensor_ctrl_ops = {
	.sensor_apply_gpout_mode = tis_sensor_apply_gpout_mode,
	.sensor_apply_gpout_function = tis_sensor_apply_gpout_function,
};

static int tis_imx390_update_common_ctrl_config(struct tis_sensor *priv, struct tis_sensor_ctrl_config *config)
{
	config->ops = &tis_imx390_sensor_ctrl_ops;

	return 0;
}

static int tis_imx390_register_private_controls(struct tis_sensor *priv, struct v4l2_ctrl_handler *handler)
{
	TIS_IMX390_PRIV(priv)->ctrl.output_sel = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_IMX390_SENSOR_OUTPUT_SELECT, "Sensor Output Select", tis_imx390_output_sel_items, ARRAY_SIZE(tis_imx390_output_sel_items), 0, 0, &tis_imx390_v4l2_ctrl_ops);
	TIS_IMX390_PRIV(priv)->ctrl.exposure_ratio = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_IMX390_EXPOSURE_RATIO, "Exposure Ratio", 1, 100, 1, 100, &tis_imx390_v4l2_ctrl_ops);
	TIS_IMX390_PRIV(priv)->ctrl.gain_offset = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_IMX390_GAIN_OFFSET, "Gain Offset", -270, 0, 1, -270, &tis_imx390_v4l2_ctrl_ops);
	TIS_IMX390_PRIV(priv)->ctrl.subtract_smpg_height = tis_sensor_ctrl_add_bool(priv, handler, TIS_SENSOR_CID_IMX390_SUBTRACT_SMPG_HEIGHT, "subtract_smpg_height", false, &tis_imx390_v4l2_ctrl_ops);

	return 0;
}

static int tis_imx390_update_private_controls(struct tis_sensor *priv)
{
	tis_sensor_ctrl_enable(TIS_IMX390_PRIV(priv)->ctrl.output_sel);
	tis_sensor_ctrl_enable(TIS_IMX390_PRIV(priv)->ctrl.exposure_ratio);
	tis_sensor_ctrl_enable(TIS_IMX390_PRIV(priv)->ctrl.gain_offset);
	tis_sensor_ctrl_enable(TIS_IMX390_PRIV(priv)->ctrl.subtract_smpg_height);

	return 0;
}

static const struct tis_sensor_ops tis_imx390_sensor_ops =
{
	.sensor_power_on = tis_imx_do_power_on,
	.sensor_update_common_ctrl_config = tis_imx390_update_common_ctrl_config,
	.sensor_register_private_controls = tis_imx390_register_private_controls,	
	.sensor_update_private_controls = tis_imx390_update_private_controls,
};

static int tis_imx390_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tis_sensor *priv;
	struct camera_common_sensor_ops *sensor_ops;
	int err;

	pr_info("[tis_imx390]: probing sensor, driver ver %s\n", __stringify(TIS_DRIVERS_VERSION));

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	if (!tis_sensor_is_sensor_model(dev, node, "imx390") )
	{
		dev_err(dev, "%s: Unexpected sensor_model", __func__);
		return -EINVAL;
	}

	if (of_get_named_gpio(node, "reset-gpios", 0) < 0) {
		dev_info(dev, "GPIO not available! Deferring probe.\n");
		return -EPROBE_DEFER;
	}

	priv = tis_sensor_create(client, 1936, 1200, TIS_SENSOR_MODEL_IMX390, sizeof(struct tis_imx390_priv), &tis_imx390_sensor_ops);
	if( IS_ERR(priv) )
	{
		return PTR_ERR(priv);
	}

	sensor_ops = tis_sensor_tegracam_build_sensor_ops(priv, &tis_imx390_common_ops);
	if( IS_ERR(sensor_ops) )
	{
		return PTR_ERR(sensor_ops);
	}

	err = tis_sensor_tegracam_device_register(priv, &sensor_regmap_config, &tis_imx390_ctrl_ops, sensor_ops, "tis_imx390" );
	if (err != 0)
	{
		return err;
	}

	err = tis_sensor_ctrl_init(priv);

	tis_sensor_register_debug_imx(priv);

	return 0;
}


static int
tis_imx390_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct tis_sensor*priv = (struct tis_sensor*)s_data->priv;

	tis_sensor_remove(priv);

	return 0;
}

static const struct i2c_device_id tis_imx390_id[] = {
	{ "tis_imx390", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tis_imx390_id);

static struct i2c_driver tis_imx390_i2c_driver = {
	.driver = {
		.name = "tis_imx390",
		.owner = THIS_MODULE,
	},
	.probe = tis_imx390_probe,
	.remove = tis_imx390_remove,
	.id_table = tis_imx390_id,
};

TIS_SENSOR_DRIVER_MODULE(tis_imx390_i2c_driver)

MODULE_DESCRIPTION("Media Controller driver for Sony TIS_IMX390");
MODULE_AUTHOR("Arne Caspari <arne.caspari@theimagingsource.com>");
MODULE_LICENSE("GPL v2");
