/*
 * tis_imx290.c - imx290 sensor driver
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2018-2019, The Imaging Source Europe GmbH.  All rights reserved.
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
#include "tis_imx290.h"
#include "tis_imx290_mode_tbls.h"

#include "tis_sensor_ctrl.h"

/* differ to max coarse */
#define TIS_IMX290_MAX_COARSE_DIFF		(1)
/* max gain reg value */
#define TIS_IMX290_GAIN_REG_MAX		(0x00F0)
/* min gain reg value */
#define TIS_IMX290_GAIN_REG_MIN		(0x01)
/* minimum gain value */
#define TIS_IMX290_MIN_GAIN			(0)
/* maximum gain value */
#define TIS_IMX290_MAX_GAIN			(100)
/* gain shift bits */
#define TIS_IMX290_GAIN_SHIFT		8
/* minimum frame length */
#define TIS_IMX290_MIN_FRAME_LENGTH		(1125)
/* maximum frame length */
#define TIS_IMX290_MAX_FRAME_LENGTH		(0x2BFD)
/* minimum exposure coarse */
#define TIS_IMX290_MIN_EXPOSURE_COARSE	(1)
/* maximum exposure coarse */
#define TIS_IMX290_MAX_EXPOSURE_COARSE	\
	(TIS_IMX290_MAX_FRAME_LENGTH-TIS_IMX290_MAX_COARSE_DIFF)

/* default gain value */
#define TIS_IMX290_DEFAULT_GAIN		(TIS_IMX290_MIN_GAIN)
/* default frame length value */
#define TIS_IMX290_DEFAULT_FRAME_LENGTH	(1125)
/* default exposure coarse value */
#define TIS_IMX290_DEFAULT_EXPOSURE_COARSE	\
	(TIS_IMX290_DEFAULT_FRAME_LENGTH-TIS_IMX290_MAX_COARSE_DIFF)

/* default mode */
#define TIS_IMX290_DEFAULT_MODE		(TIS_IMX290_MODE_1920X1080_60FPS)

/* default image output width */
#define TIS_IMX290_DEFAULT_WIDTH		(1948)
/* default image output height */
#define TIS_IMX290_DEFAULT_HEIGHT		(1096)
/* default image data format */
#define TIS_IMX290_DEFAULT_DATAFMT		(MEDIA_BUS_FMT_SRGGB12_1X12)
/* default output clk frequency for camera */
#define TIS_IMX290_DEFAULT_CLK_FREQ		(37125000)


static const struct of_device_id tis_imx290_of_match[] = {
	{ .compatible = "tis,tis_imx290", },
	{ },
};

MODULE_DEVICE_TABLE(of, tis_imx290_of_match);


static int tis_imx290_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int tis_imx290_set_gain(struct tegracam_device *tc_dev, s64 val);
static int tis_imx290_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int tis_imx290_set_group_hold(struct tegracam_device *tc_dev, bool val);
static struct camera_common_pdata *tis_imx290_parse_dt(struct tegracam_device *tc_dev);


static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

static const struct tegracam_ctrl_ops tis_imx290_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = tis_imx290_set_gain,
	.set_exposure = tis_imx290_set_exposure,
	.set_frame_rate = tis_imx290_set_frame_rate,
	.set_group_hold = tis_imx290_set_group_hold,
};


static int tis_imx290_write_table(struct tis_sensor *priv,
				const tis_imx290_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 TIS_IMX290_TABLE_WAIT_MS,
					 TIS_IMX290_TABLE_END);
}

// Sets exposure time and frame rate
static int tis_imx290_set_exposure(struct tegracam_device *tc_dev, s64 exposure_time)
{
	// struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&priv->s_data->sensor_props.sensor_modes[priv->s_data->mode_prop_idx];

	int err = 0;	

	const u64 pixel_clock = priv->pixel_clock * 2; // IMX290 calculations are done with double the pixel clock
	const u32 exposure_factor = mode->control_properties.exposure_factor;
	const u32 framerate_factor = mode->control_properties.framerate_factor;
	const u32 HMAX = mode->image_properties.line_length;
    
	const u32 vmax_min = mode->image_properties.height + 12 + 13; // height + IMX290_WINWV_OB_VALUE + 13
    const u32 vmax_max = 0x3FFFF;

	u32 fps_defined_vmax_unbounded = (pixel_clock * framerate_factor) / (priv->fps * HMAX);
	u32 fps_defined_vmax = clamp(fps_defined_vmax_unbounded, vmax_min, vmax_max);

    const u64 h_period_ns = HMAX * exposure_factor * 1000ull / pixel_clock;
	u32 exposure_lines = max( (u32)(exposure_time * 1000ull / h_period_ns), 1u );
    u32 vmax = max(exposure_lines + 2, fps_defined_vmax);
    u32 shs = vmax - exposure_lines - 1;

	err = tis_imx_write_24(priv->s_data, TIS_IMX290_REG_SHS1, shs);
	err = tis_imx_write_24(priv->s_data, TIS_IMX290_REG_VMAX, vmax);
	err = tis_imx_write_16(priv->s_data, TIS_IMX290_REG_HMAX, HMAX);

	dev_info(dev, "%s: exposure_time = %lld, fps = %lld\n", __func__, exposure_time, priv->fps);
	dev_info(dev, "%s: shs = %d vmax = %d", __func__, shs, vmax);
	dev_info(dev, "%s: HMAX = %d, pixel_clock = %lld", __func__, HMAX, pixel_clock);

	priv->exposure_time = exposure_time;

	return err;
}


static int tis_imx290_configure_roi_position(struct tis_sensor *priv,
	int offset_x, int offset_y,
	int flip_x, int flip_y)
{
	struct camera_common_data *s_data = priv->s_data;
	tis_imx_write_16(s_data, TIS_IMX290_REG_WINPH, offset_x + (flip_x ? 1 : 0));
	tis_imx_write_16(s_data, TIS_IMX290_REG_WINPV, offset_y + (flip_y ? 1 : 0));
	return 0;
}

static int tis_imx290_configure_roi(struct tis_sensor *priv,
	int active_w, int active_h,
	int offset_x, int offset_y,
	int flip_x, int flip_y)
{
	struct camera_common_data *s_data = priv->s_data;

	const int SENSOR_WIDTH = priv->SENSOR_WIDTH;
	const int SENSOR_HEIGHT = priv->SENSOR_HEIGHT;

	// Make sure we don't overflow to the right/bottom
	offset_x = min(offset_x, SENSOR_WIDTH - active_w);
	offset_y = min(offset_y, SENSOR_HEIGHT - active_h);

	tis_imx_write_8(s_data, TIS_IMX290_REG_WINMODE, 0x40 | (flip_x ? 0x02 : 0x00) | (flip_y ? 0x01 : 0x00));
	tis_imx_write_16(s_data, TIS_IMX290_REG_WINWH, active_w);
	tis_imx_write_16(s_data, TIS_IMX290_REG_WINWV, active_h);
	tis_imx_write_16(s_data, TIS_IMX290_REG_X_OUT_SIZE, active_w);
	tis_imx_write_16(s_data, TIS_IMX290_REG_Y_OUT_SIZE, active_h);

	return tis_imx290_configure_roi_position(priv, offset_x, offset_y, flip_x, flip_y);
}

static int tis_imx290_apply_black_level(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;

	int reg_val = clamp( priv->ctrl.black_level, TIS_IMX290_BLACK_LEVEL_MIN, TIS_IMX290_BLACK_LEVEL_MAX );

	return tis_imx_write_16( s_data, TIS_IMX290_REG_BLKLEVEL, reg_val );
}

static int tis_imx290_apply_gain_mode(struct tis_sensor *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	u8 reg_val = 0;

	int err = tis_imx_read_8( s_data, TIS_IMX290_REG_FRSEL, &reg_val );
	if( err )
		return err;

	reg_val = (reg_val & 0x0F) | (priv->ctrl.gain_mode ? 0x10 : 0x00);

	return tis_imx_write_8( s_data, TIS_IMX290_REG_FRSEL, reg_val );
}

static int tis_imx290_set_mode(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int err;
	u32 mode_lane_mbps, mode_bpp, mclk_khz;
	u32 actual_clk_hz;

	dev_dbg(dev, "set mode %d", s_data->mode_prop_idx);

	err = tis_sensor_ctrl_do_lazy_init(priv);
	if( err )
	{
		dev_err(dev, "%s: do_lazy_init failed: %d", __func__, err);
		return err;
	}

	tis_imx_get_mode_from_dt(tc_dev, s_data->mode_prop_idx, &priv->mode_active_w, &priv->mode_active_h, &priv->mode_lanes, &mode_lane_mbps, &mode_bpp, &mclk_khz);
	dev_info(dev, "active_w: %d, active_h: %d, mode_lanes: %d, bpp: %d", priv->mode_active_w, priv->mode_active_h, priv->mode_lanes, mode_bpp);	

	err = tis_sensor_configure_connection(priv, priv->mode_lanes, mclk_khz * 1000, &actual_clk_hz);
	if( err )
	{
		dev_err(dev, "%s: tis_sensor_configure_connection failed: %d", __func__, err);
		return err;
	}

	// Sensor PLL settings multiply input by 2
	priv->pixel_clock = 2 * actual_clk_hz;

	err = tis_imx290_write_table(priv, tis_imx290_mode_1920x1080_60fps);
	if (err)
		return err;

	if( priv->mode_lanes == 2 )
	{
		// Already configured by table above
	}
	else
	{
		tis_imx_write_8( s_data, 0x3009, 0x00 ); // FRSEL
		tis_imx_write_8( s_data, 0x3407, 0x03 ); // PHYSICAL_LANE_NUM
		tis_imx_write_8( s_data, 0x3443, 0x03 ); // CSI_LANE_MODE
	}

	if( mode_bpp == 12 )
	{
		// Already configured by table above
	}
	else
	{
		tis_imx_write_8( s_data, 0x3005, 0x00 ); // ADBIT
		tis_imx_write_8( s_data, 0x3046, 0x00 ); // ODBIT
		tis_imx_write_8( s_data, 0x3129, 0x1D ); // ADBIT1
		tis_imx_write_8( s_data, 0x317C, 0x12 ); // ADBIT2
		tis_imx_write_8( s_data, 0x31EC, 0x37 ); // ADBIT3
		tis_imx_write_8( s_data, 0x3441, 0x0A ); // CSI_DT_FMT
		tis_imx_write_8( s_data, 0x3442, 0x0A ); // CSI_DT_FMT
	}

	tis_sensor_ctrl_do_offset_auto_center(priv);

	err = tis_imx290_configure_roi(priv, priv->mode_active_w, priv->mode_active_h, priv->ctrl.offset_x, priv->ctrl.offset_y, 0, 0);
	if (err)
		return err;

	tis_sensor_init_defaults(priv, mode);

	tis_imx290_apply_black_level(priv);

	tis_imx290_set_gain(tc_dev, priv->gain);
	return tis_imx290_set_exposure(tc_dev, priv->exposure_time);
}


static int tis_imx290_start_streaming(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	int err;

	dev_dbg(dev, "%s\n", __func__);

	err = tis_imx290_write_table(priv, tis_imx290_start);
	if (err)
	{
		dev_dbg(dev, "%s: error setting stream\n", __func__);
		return err;
	}

	return 0;
}


static int tis_imx290_stop_streaming(struct tegracam_device *tc_dev)
{
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	dev_dbg(dev, "%s\n", __func__);

	err = tis_imx290_write_table(priv, tis_imx290_stop);
	if (err)
	{
		dev_dbg(dev, "%s: error setting stream\n", __func__);
		// Don't fail stop_streaming; if this does not work we were likely disconnected
	}

	return 0;
}


static struct camera_common_sensor_ops tis_imx290_common_ops = {
	.numfrmfmts = ARRAY_SIZE(tis_imx290_frmfmt),
	.frmfmt_table = tis_imx290_frmfmt,
	.power_on = tis_imx_fake_power_on,
	.power_off = tis_imx_fake_power_off,
	.write_reg = tis_imx_write_8,
	.read_reg = tis_imx_read_8,
	.parse_dt = tis_imx290_parse_dt,
	.power_get = tis_imx_power_get,
	.power_put = tis_imx_power_put,
	.set_mode = tis_imx290_set_mode,
	.start_streaming = tis_imx290_start_streaming,
	.stop_streaming = tis_imx290_stop_streaming,
};


static int tis_imx290_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	dev_dbg(dev, "%s\n", __func__);

	err = tis_imx_write_8(s_data,
					TIS_IMX290_REG_REGHOLD, val);
	if (err){
		dev_err(dev,
		 "%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}


static int tis_imx290_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct tis_sensor *priv = (struct tis_sensor *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;	
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	int err;

	dev_dbg(dev, "%s - val = %lld\n", __func__, val);

	priv->gain = val;

	// Supplied val is scaled by gain_factor to dB, but register value is is .3 dB steps
	val *= 10;
	val /= (mode->control_properties.gain_factor * 3);

	if (val < TIS_IMX290_MIN_GAIN)
		val = TIS_IMX290_MIN_GAIN;
	else if (val > TIS_IMX290_MAX_GAIN)
		val = TIS_IMX290_MAX_GAIN;

	err = tis_imx_write_8(s_data, TIS_IMX290_REG_GAIN, val & 0xff);	

	return err;
}


static int tis_imx290_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	struct tis_sensor *priv = (struct tis_sensor *)tegracam_get_privdata(tc_dev);
	int err = 0;

	dev_info(dev, "set frame rate: %lld\n", val);

	priv->fps = val;

	tis_imx290_set_exposure(tc_dev, priv->exposure_time);

	return err;
}



static struct camera_common_pdata *tis_imx290_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!np)
		return NULL;

	match = of_match_device(tis_imx290_of_match, dev);
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
	err = of_property_read_string(np, "parent_clk",
				      &board_priv_pdata->parentclk_name);

	board_priv_pdata->reset_gpio = of_get_named_gpio(np,
			"reset-gpios", 0);

	board_priv_pdata->pwdn_gpio = of_get_named_gpio(np,
			"pwdn-gpios", 0);

	of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);

	return board_priv_pdata;
}

static const struct media_entity_operations tis_imx290_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

int tis_imx290_apply_offsets(struct tis_sensor *priv)
{
	tis_sensor_ctrl_do_offset_auto_center(priv);

	if( !tis_sensor_is_power_on(priv) )
		return 0;

	return tis_imx290_configure_roi(priv, priv->mode_active_w, priv->mode_active_h, priv->ctrl.offset_x, priv->ctrl.offset_y, 0, 0);
}

const struct tis_sensor_ctrl_ops tis_imx290_sensor_ctrl_ops = {
	.sensor_apply_gpout_mode = tis_sensor_apply_gpout_mode,
	.sensor_apply_gpout_function = tis_sensor_apply_gpout_function,
	.sensor_apply_offsets = tis_imx290_apply_offsets,
	.sensor_apply_black_level = tis_imx290_apply_black_level,
	.sensor_apply_gain_mode = tis_imx290_apply_gain_mode,
};

static int tis_imx290_update_common_ctrl_config(struct tis_sensor *priv, struct tis_sensor_ctrl_config *config)
{
	config->ops = &tis_imx290_sensor_ctrl_ops;

	config->offset_x_step = 4;
	config->offset_y_step = 4;

	config->black_level_min = TIS_IMX290_BLACK_LEVEL_MIN;
	config->black_level_max = TIS_IMX290_BLACK_LEVEL_MAX;
	config->black_level_default = TIS_IMX290_BLACK_LEVEL_DEFAULT;

	return 0;
}

static const struct tis_sensor_ops tis_imx290_sensor_ops =
{
	.sensor_power_on = tis_imx_do_power_on,
	.sensor_update_common_ctrl_config = tis_imx290_update_common_ctrl_config,
};

static int tis_imx290_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tis_sensor *priv;
	struct camera_common_sensor_ops *sensor_ops;
	int err;

	pr_info("[tis_imx290]: probing sensor, driver ver %s\n", __stringify(TIS_DRIVERS_VERSION));

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	if (of_get_named_gpio(node, "reset-gpios", 0) < 0) {
		dev_info(dev, "GPIO not available! Deferring probe.\n");
		return -EPROBE_DEFER;
	}

	priv = tis_sensor_create(client, 1948, 1096, TIS_SENSOR_MODEL_IMX290, 0, &tis_imx290_sensor_ops);
	if( IS_ERR(priv) )
	{
		return PTR_ERR(priv);
	}

	sensor_ops = tis_sensor_tegracam_build_sensor_ops(priv, &tis_imx290_common_ops);
	if( IS_ERR(sensor_ops) )
	{
		return PTR_ERR(sensor_ops);
	}

	err = tis_sensor_tegracam_device_register(priv, &sensor_regmap_config, &tis_imx290_ctrl_ops, sensor_ops, "tis_imx290" );
	if (err != 0)
	{
		return err;
	}

	dev_dbg(dev, "Detected TIS_IMX290 sensor\n");

	err = tis_sensor_ctrl_init(priv);
	if( !err )
	{

	}

	tis_sensor_register_debug_imx(priv);

	return 0;
}


static int
tis_imx290_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct tis_sensor *priv = (struct tis_sensor *)s_data->priv;

	tis_sensor_remove(priv);

	return 0;
}


static const struct i2c_device_id tis_imx290_id[] = {
	{ "tis_imx290", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tis_imx290_id);

static struct i2c_driver tis_imx290_i2c_driver = {
	.driver = {
		.name = "tis_imx290",
		.owner = THIS_MODULE,
	},
	.probe = tis_imx290_probe,
	.remove = tis_imx290_remove,
	.id_table = tis_imx290_id,
};

TIS_SENSOR_DRIVER_MODULE(tis_imx290_i2c_driver)

MODULE_DESCRIPTION("Media Controller driver for Sony IMX290");
MODULE_AUTHOR("Arne Caspari <arne.caspari@theimagingsource.com>");
MODULE_LICENSE("GPL");
