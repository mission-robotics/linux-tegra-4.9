/*
 * imx530.c - imx530 sensor driver
 *
 * Copyright (c) 2019. FRAMOS.  All rights reserved.
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
//#define DEBUG 1
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "imx530_mode_tbls.h"
#include "framos_sensor_common.h"

#define IMX530_K_FACTOR 1000LL
#define IMX530_M_FACTOR 1000000LL
#define IMX530_G_FACTOR 1000000000LL
#define IMX530_T_FACTOR 1000000000000LL

#define IMX530_MAX_GAIN_DEC 480
#define IMX530_MAX_GAIN_DB  48

#define IMX530_MAX_BLACK_LEVEL 0xFFF
#define IMX530_DEFAULT_BLACK_LEVEL 0x3C

#define IMX530_DEFAULT_LINE_TIME 14380 // [ns]

#define IMX530_MIN_SHS_LENGTH 16
#define IMX530_INTEGRATION_OFFSET 0
#define IMX530_MIN_INTEGRATION_LINES 1

#define IMX530_INCK 74250000LL

#define IMX530_LVDS_NUM_CHANNELS 8
#define IMX530_READOUT_LINE_SKIP 68
#define IMX530_READOUT_LINE_SKIP_2 36

/**
 * Declaration
 */
static int imx530_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int imx530_calculate_line_time(struct tegracam_device *tc_dev);

static const struct of_device_id imx530_of_match[] = {
	{ .compatible = "framos,imx530",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx530_of_match);


static int imx530_set_custom_ctrls(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops imx530_custom_ctrl_ops = {
	.s_ctrl = imx530_set_custom_ctrls,
};

static const char * const imx530_test_pattern_menu[] = {
    [0]   = "No pattern",
    [1]   = "Sequence Pattern 1",
    [2]   = "Sequence Pattern 2",
    [3]   = "Gradation Pattern",
};

static struct v4l2_ctrl_config imx530_custom_ctrl_list[] = {
    {
        .ops = &imx530_custom_ctrl_ops,
        .id = TEGRA_CAMERA_CID_TEST_PATTERN,
        .name = "Test Pattern",
        .type = V4L2_CTRL_TYPE_MENU,
        .min = 0,
        .max = ARRAY_SIZE(imx530_test_pattern_menu) - 1,
        .def = 0,
        .qmenu = imx530_test_pattern_menu,
    },
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
    TEGRA_CAMERA_CID_OPERATION_MODE,
    TEGRA_CAMERA_CID_BLACK_LEVEL,
    TEGRA_CAMERA_CID_FPGA_FIRMWARE_PROPERTIES,
    TEGRA_CAMERA_CID_SHUTTER_MODE,
    TEGRA_CAMERA_CID_TG_MODE,
    TEGRA_CAMERA_CID_EXPAND_EXPOSURE,
    TEGRA_CAMERA_CID_FRAME_DELAY,
};

struct imx530 {
	struct i2c_client	            *i2c_client;
	struct v4l2_subdev	            *subdev;
	u64				                frame_length;
    u64                             min_frame_length;
	u32				                line_time;
    operation_mode                  current_operation_mode;
    shutter_mode                    current_shutter_mode;
	struct camera_common_data	    *s_data;
	struct tegracam_device		    *tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline int imx530_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx530_write_reg(struct camera_common_data *s_data,
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

/**
 * Reads multiple sequential registers at the same time using grouphold
 */
static int imx530_read_buffered_reg(struct camera_common_data *s_data,
                                 u16 addr_low, u8 number_of_registers, u64 *val)
{
	struct device *dev = s_data->dev;
    int err, i;
    u8 reg;

    *val = 0;

    if (!s_data->group_hold_active){
        err = imx530_write_reg(s_data, REGHOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: error setting register hold\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx530_read_reg(s_data, addr_low + i, &reg);
        *val += reg << (i * 8);
        if (err) {
            dev_err(dev, "%s: error reading buffered registers\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx530_write_reg(s_data, REGHOLD, 0x00);
        if (err) {
            dev_err(dev, "%s: error unsetting register hold\n", __func__);
            return err;
        }
    }

    return err;
}

/**
 * Writes multiple sequential registers at the same time using reghold
 */
static int imx530_write_buffered_reg(struct camera_common_data *s_data, 
                                u16 addr_low, u8 number_of_registers, u64 val)
{
    int err, i;
	struct device *dev = s_data->dev;

    if (!s_data->group_hold_active){
        err = imx530_write_reg(s_data, REGHOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx530_write_reg(s_data, addr_low + i, (u8)(val >> (i * 8)));
        if (err) {
            dev_err(dev, "%s: BUFFERED register write error\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx530_write_reg(s_data, REGHOLD, 0x00);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
            return err;
        }
    }

    return err;
}

static int imx530_write_table(struct imx530 *priv,
				const imx530_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 IMX530_TABLE_WAIT_MS,
					 IMX530_TABLE_END);
}

static int imx530_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

    s_data->group_hold_active = val;

    err = imx530_write_reg(s_data, REGHOLD, val);
    if (err) {
        dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
        return err;
    }

	return err;
}

static int imx530_update_ctrl(struct tegracam_device *tc_dev)
{
    struct v4l2_ctrl *ctrl;

	/* Update Black level control*/
    ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL);
    if (ctrl) {
        *ctrl->p_new.p_s64 = IMX530_DEFAULT_BLACK_LEVEL;
        *ctrl->p_cur.p_s64 = IMX530_DEFAULT_BLACK_LEVEL;
		ctrl->default_value = IMX530_DEFAULT_BLACK_LEVEL;
    }

    return 0;
}

static int imx530_set_black_level(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

    err = imx530_write_buffered_reg(s_data, BLKLEVEL_LOW, 2, val);
	if (err){
	    dev_dbg(dev, "%s: BLACK LEVEL control error\n", __func__);
	    return err;
    }

	dev_dbg(dev, "%s: black level: %lld\n",  __func__, val);

	return 0;
}

static int imx530_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	int err;
	u32 gain;

	/* translate value */
	gain = val * IMX530_MAX_GAIN_DEC /
                 (IMX530_MAX_GAIN_DB *
                     mode->control_properties.gain_factor);

    err = imx530_write_buffered_reg(s_data, GAIN_LOW, 2, gain);
	if (err){
	    dev_dbg(dev, "%s: GAIN control error\n", __func__);
	    return err;
    }

	dev_dbg(dev, "%s:  gain val [%lld] reg [%d]\n",  __func__, val, gain);

	return 0;
}


static int imx530_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx530 *priv = (struct imx530 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
    struct v4l2_ctrl *ctrl;
	int err;
	u32 integration_time_line;
    u32 integration_offset = IMX530_INTEGRATION_OFFSET;
	u32 reg_shs, min_reg_shs;

	dev_dbg(dev, "%s: integration time: %lld [us]\n", __func__, val);

    /* Check value with internal range */
    if (val > s_data->exposure_max_range) {
        val = s_data->exposure_max_range;
    }
    else if (val < s_data->exposure_min_range) {
        val = s_data->exposure_min_range;
    }

	integration_time_line = ((val - integration_offset) 
                                    * IMX530_K_FACTOR) / priv->line_time ;

	reg_shs = priv->frame_length - integration_time_line;
    
    switch (s_data->mode) {
    case IMX530_MODE_5328x4608:
    case IMX530_MODE_ROI_4512x4512:
    case IMX530_MODE_ROI_5328x3040:
    case IMX530_MODE_ROI_4064x3008:
        min_reg_shs = 16;
        break;
    case IMX530_MODE_BINNING_2656x2304:
        min_reg_shs = 32;
        break;
    default:
        min_reg_shs = 16;
    }

    if (reg_shs < min_reg_shs)
        reg_shs = min_reg_shs;
	else if (reg_shs > (priv->frame_length - IMX530_MIN_INTEGRATION_LINES))
		reg_shs = priv->frame_length - IMX530_MIN_INTEGRATION_LINES;

    err = tg_set_trigger_exposure(tc_dev, (u32)val);
    err = imx530_write_buffered_reg (s_data, SHS_LOW, 3, reg_shs);
    if (err) {
        dev_err(dev, "%s: failed to set frame length\n", __func__);
        return err;
    }

    /* Update new ctrl value */
    ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_EXPOSURE);
    if (ctrl) {
        /* Value could be adjusted, set the right value */
        *ctrl->p_new.p_s64 = val;
        /* This ctrl is affected on FRAME RATE control also */
        *ctrl->p_cur.p_s64 = val;
    }

	dev_dbg(dev,
     "%s: set integration time: %lld [us], coarse1:%d [line], shs: %d [line], frame length: %llu [line]\n",
     __func__, val, integration_time_line, reg_shs, priv->frame_length);

	return err;
}

static int imx530_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct imx530 *priv = (struct imx530 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;
    u32 min_reg_shs;
    u64 frame_length;
    u64 exposure_max_range, exposure_min_range;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];

    frame_length = (((u64)mode->control_properties.framerate_factor * 
                              IMX530_G_FACTOR) / (val * priv->line_time));

    if (frame_length < priv->min_frame_length)
        frame_length = priv->min_frame_length;
    
    priv->frame_length = frame_length;
    tg_params->frame_length = frame_length;

    switch (s_data->mode) {
    case IMX530_MODE_5328x4608:
    case IMX530_MODE_ROI_4512x4512:
    case IMX530_MODE_ROI_5328x3040:
    case IMX530_MODE_ROI_4064x3008:
        min_reg_shs = 16;
        break;
    case IMX530_MODE_BINNING_2656x2304:
        min_reg_shs = 32;
        break;
    default:
        min_reg_shs = 16;
    }

    /* Update exposure range, before writing the new frame length */
    exposure_min_range = IMX530_MIN_INTEGRATION_LINES 
                                            * priv->line_time / IMX530_K_FACTOR;
    exposure_min_range += IMX530_INTEGRATION_OFFSET;    
    exposure_max_range = (priv->frame_length - min_reg_shs) 
                                            * priv->line_time / IMX530_K_FACTOR;
    fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_EXPOSURE,
                             exposure_min_range, exposure_max_range);

    err = tg_set_frame_width(tc_dev, IMX530_G_FACTOR / (val / 1000));
    err = imx530_write_buffered_reg(s_data, VMAX_LOW, 3, priv->frame_length);
    if (err) {
        dev_err(dev, "%s: failed to set frame length\n", __func__);
        return err;
    }

	dev_dbg(dev,
        "%s: val: %lld, frame_length set: %llu\n",
             __func__, val, priv->frame_length);

	return 0;
}

/**
 * Test pattern is described in the "Pattern Generator (PG)" chapter
 * in the IMX530 support package
 */
static int imx530_set_test_pattern(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev; 
    int err;

    if (val) {
        err = imx530_write_reg(s_data, 0x3550, 0x07);
        if (err) 
            goto fail;

        err = imx530_write_reg(s_data, 0x3551, (u8)(val));
        if (err) 
            goto fail;
    }
    else {
        err = imx530_write_reg(s_data, 0x3550, 0x06);
        if (err) 
            goto fail;
    }

    return 0;

fail:
    dev_err(dev, "%s: error setting test pattern\n", __func__);
    return err;
}

/**
 * Update max framerate range
 */
static int imx530_update_framerate_range(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx530 *priv = (struct imx530 *)tegracam_get_privdata(tc_dev);
	struct sensor_control_properties *ctrlprops = NULL;
    u64 max_framerate;

    ctrlprops = 
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

    switch (s_data->mode) {
    case IMX530_MODE_5328x4608:
        priv->min_frame_length = IMX530_DEFAULT_HEIGHT + IMX530_MIN_FRAME_DELTA;
        break;
    case IMX530_MODE_ROI_4512x4512:
        if (s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB10_1X10) {
            priv->min_frame_length = IMX530_ROI_MODE_HEIGHT + IMX530_MIN_FRAME_DELTA_10BIT;
        } else {
            priv->min_frame_length = IMX530_ROI_MODE_HEIGHT + IMX530_MIN_FRAME_DELTA;
        }
        break;
    case IMX530_MODE_ROI_5328x3040:
        priv->min_frame_length = IMX530_ROI_MODE_2_HEIGHT + IMX530_MIN_FRAME_DELTA;
        break;
    case IMX530_MODE_ROI_4064x3008:
        if (s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB10_1X10) {
            priv->min_frame_length = IMX530_ROI_MODE_3_HEIGHT + IMX530_MIN_FRAME_DELTA_10BIT;
        } else {
            priv->min_frame_length = IMX530_ROI_MODE_3_HEIGHT + IMX530_MIN_FRAME_DELTA;
        }
        break;
    case IMX530_MODE_BINNING_2656x2304:
        priv->min_frame_length = 2432;
        break;
    }

    max_framerate = (IMX530_G_FACTOR * IMX530_M_FACTOR) /
                                     (priv->min_frame_length * priv->line_time);
    
    fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_FRAME_RATE, 
                            ctrlprops->min_framerate, max_framerate);

    return 0;
}

/**
 * Set operation mode of sensor 
 */
static int imx530_set_operation_mode(struct tegracam_device *tc_dev, u32 val)
{
	struct imx530 *priv = (struct imx530 *)tc_dev->priv;
    struct v4l2_ctrl *ctrl;
    int err;

    priv->current_operation_mode = val;

    ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_TG_MODE);
    err = tg_set_operation_mode(tc_dev, ctrl);
    if (err)
        return err;   
    
    return 0;
}

/**
 * Set supported shutter mode
 */
static int imx530_set_shutter_mode(struct tegracam_device *tc_dev, 
                                   struct v4l2_ctrl *ctrl)
{
	struct imx530 *priv = (struct imx530 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;

    if (priv->current_operation_mode == MASTER_MODE
        && *ctrl->p_new.p_u8 == SEQ_TRIGGER){
        dev_warn(dev, 
        "%s: Sequential trigger isn't supported in master mode\n", __func__);     
        goto default_state;    
    }

    priv->current_shutter_mode = *ctrl->p_new.p_u8;
    return 0;

default_state:
    priv->current_shutter_mode = NORMAL_EXPO;
    ctrl->val = priv->current_shutter_mode;
    return 0;
}

static int imx530_set_custom_ctrls(struct v4l2_ctrl *ctrl)
{
    struct tegracam_ctrl_handler *handler = 
                                container_of(ctrl->handler,
                                    struct tegracam_ctrl_handler, ctrl_handler);
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	struct tegracam_device *tc_dev = handler->tc_dev;
	int err = 0;

	switch (ctrl->id) {
    case TEGRA_CAMERA_CID_TEST_PATTERN:
		err = ops->set_test_pattern(tc_dev, *ctrl->p_new.p_u32);
        break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static struct tegracam_ctrl_ops imx530_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx530_set_gain,
	.set_exposure = imx530_set_exposure,
	.set_frame_rate = imx530_set_frame_rate,
	.set_group_hold = imx530_set_group_hold,
    .set_test_pattern = imx530_set_test_pattern,
    .set_operation_mode = imx530_set_operation_mode,
    .set_shutter_mode = imx530_set_shutter_mode,
    .set_black_level = imx530_set_black_level,
    .verify_fw_compatibility = common_verify_crosslink_fw_compatibility,
    .set_timing_generator_mode = tg_set_operation_mode,
    .expand_trigger_exposure = tg_expand_trigger_exposure,
    .delay_frame = tg_delay_frame,
};

static int imx530_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);

	if (gpio_is_valid(pw->fw_reset_gpio))
		fm_gpio_set(s_data, pw->fw_reset_gpio, 1);
	if (gpio_is_valid(pw->pwdn_gpio))
		fm_gpio_set(s_data, pw->pwdn_gpio, 1);

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

    if (!pw->mclk) {
        dev_err(dev, "%s: mclk not available\n",  __func__);
        return -ENODEV;
    }

    /* Power ON sequence according to IMX530 datasheet */

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto imx530_dvdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto imx530_iovdd_fail;
	}

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto imx530_avdd_fail;
	}

    err = clk_prepare_enable(pw->mclk);
    if(err) {
        dev_err(dev, "%s: failed to enable mclk\n",  __func__);
        return err;    
    }

    usleep_range(1, 2);

	if (gpio_is_valid(pw->reset_gpio))
		fm_gpio_set(s_data, pw->reset_gpio, 1);

    /* MUST be before sleep */
    pw->state = SWITCH_ON;

    /* Additional sleep required in the case of hardware power-on sequence */
    usleep_range(30000, 31000);

	return 0;

imx530_avdd_fail:
	regulator_disable(pw->iovdd);

imx530_iovdd_fail:
	regulator_disable(pw->dvdd);

imx530_dvdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int imx530_power_off(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}

    /* Power OFF sequence according to IMX530 datasheet */

    clk_disable_unprepare(pw->mclk);

	if (gpio_is_valid(pw->reset_gpio))
		fm_gpio_set(s_data, pw->reset_gpio, 0);

	if (pw->avdd)
		regulator_disable(pw->avdd);
	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

/**
 * Acquires regulators, clock and GPIO defined in platform_data
 */
static int imx530_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "extperiph1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* Analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* Dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	if (err) {
		dev_info(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	pw->reset_gpio = pdata->reset_gpio;
    pw->pwdn_gpio = pdata->pwdn_gpio;
    pw->fw_reset_gpio = pdata->creset_b;

	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(dev, pw->reset_gpio);
		if (err) {
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->reset_gpio);
            goto done;
        }
		err = cam_gpio_register(dev, pw->pwdn_gpio);
		if (err) {
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->pwdn_gpio);
            goto done;
        }
		err = cam_gpio_register(dev, pw->fw_reset_gpio);
		if (err)
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->fw_reset_gpio);
	}

done:
	pw->state = SWITCH_OFF;
	return err;
}

/**
 * Frees regulators acquired in power_get
 */
static int imx530_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;

	if (unlikely(!pw))
		return -EFAULT;

    if (likely(pw->avdd)) {
        regulator_put(pw->avdd);
    }

    if (likely(pw->iovdd)) {
        regulator_put(pw->iovdd);
    }

    if (likely(pw->dvdd)) {
        regulator_put(pw->dvdd);
    }

    pw->avdd = NULL;
    pw->iovdd = NULL;
    pw->dvdd = NULL;

	if (pdata && pdata->use_cam_gpio) {
		cam_gpio_deregister(dev, pw->reset_gpio);
		cam_gpio_deregister(dev, pw->pwdn_gpio);
		cam_gpio_deregister(dev, pw->fw_reset_gpio);
    }
	else {
		if (gpio_is_valid(pw->reset_gpio))
			gpio_free(pw->reset_gpio);
		if (gpio_is_valid(pw->pwdn_gpio))
			gpio_free(pw->pwdn_gpio);
		if (gpio_is_valid(pw->fw_reset_gpio))
			gpio_free(pw->fw_reset_gpio);
	}

	return 0;
}

static struct camera_common_pdata *imx530_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx530_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(dev,
					 board_priv_pdata);
	if (err) {
		dev_err(dev, "Failed to find clocks\n");
		goto error;
	}

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found %d\n", err);
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

    gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "pwdn gpio not found %d\n", err);
		goto error;
	}
    board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

    gpio = of_get_named_gpio(np, "cresetb", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "creset_b gpio not found %d\n", err);
		goto error;
	}
    board_priv_pdata->creset_b = (unsigned int)gpio;

    fm_get_gpio_ctrl(board_priv_pdata);

	err = of_property_read_string(np, "avdd-reg",
		&board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
		&board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
		&board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;
}

static int imx530_set_pixel_format(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct imx530 *priv = (struct imx530 *)tegracam_get_privdata(tc_dev);
    struct camera_common_data *s_data = tc_dev->s_data;
    int err;

	switch (s_data->colorfmt->code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
        err = imx530_write_table(priv, mode_table[IMX530_10BIT_MODE]);
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
        err = imx530_write_table(priv, mode_table[IMX530_12BIT_MODE]);
		break;
	default:
        dev_err(dev, "%s: unknown pixel format\n", __func__);
		return -EINVAL;
	}

    return err;
}

static int imx530_set_dep_registers(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
    struct camera_common_data *s_data = tc_dev->s_data;
    int err;
    /* Default for all-pixel 12bit */
    u8 gmrwt = 0x04;
    u8 gmtwt = 0x04;
    u8 gaindly = 0x04;
    u8 gsdly = 0x08;

	switch (s_data->colorfmt->code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
        gmtwt = 0x08;
        if (s_data->mode == IMX530_MODE_BINNING_2656x2304) {
            gmrwt = 0x08;
            gmtwt = 0x10;
            gaindly = 0x08;
            gsdly = 0x10;
        } else if (s_data->mode != IMX530_MODE_5328x4608) {
            gmrwt = 0x0C;
            gmtwt = 0x1C;
            gaindly = 0x04;
            gsdly = 0x2C;
        }
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
        if (s_data->mode == IMX530_MODE_BINNING_2656x2304) {
            gmrwt = 0x08;
            gmtwt = 0x08;
            gaindly = 0x08;
            gsdly = 0x10;
        } else if (s_data->mode != IMX530_MODE_5328x4608) {
            gmrwt = 0x08;
            gmtwt = 0x14;
            gaindly = 0x04;
            gsdly = 0x20;
        }
		break;
	default:
        dev_err(dev, "%s: unknown pixel format\n", __func__);
		return -EINVAL;
	}

    err  = imx530_write_reg(s_data, GMRWT, gmrwt);
    err |= imx530_write_reg(s_data, GMTWT, gmtwt);
    err |= imx530_write_reg(s_data, GAINDLY, gaindly);
    err |= imx530_write_reg(s_data, GSDLY, gsdly);
    if (err) {
        dev_err(dev, "%s: error setting exposure mode\n", __func__);  
        return err;        
    }

    return err;
}

/**
 * Calculate 1H time
 */
static int imx530_calculate_line_time(struct tegracam_device *tc_dev)
{
	struct imx530 *priv = (struct imx530 *)tc_dev->priv;
    struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
    u64 hmax;
    int err;

	dev_dbg(dev, "%s:++\n", __func__);

    err = imx530_read_buffered_reg(s_data, HMAX_LOW, 2, &hmax);
    if (err) {
        dev_err(dev, "%s: unable to read hmax\n", __func__);
        return err;
    }

    priv->line_time = (hmax*IMX530_G_FACTOR) / (IMX530_INCK);


	dev_dbg(dev, "%s: hmax: %llu [inck], INCK: %u [Hz], line_time: %u [ns]\n",
            __func__, hmax, s_data->def_clk_freq, priv->line_time);

    return 0;
}

/**
 * Configure Global Shutter Operation
 * V interrupt is disabled in init mode table
 */
static int imx530_configure_shutter(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx530 *priv = (struct imx530 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
    int err = 0;
    /* Default is Normal exposure */
    u8 trigen = 0; 
    u8 vint_en = 0;

    switch (priv->current_shutter_mode) {
    case NORMAL_EXPO:
        trigen = 0; 
        vint_en = 2;         
        dev_dbg(dev, "%s: Sensor is in Normal Exposure Mode\n", __func__);

        break;

    case SEQ_TRIGGER:
        if (priv->current_operation_mode == MASTER_MODE) {
            dev_warn(dev, 
"%s: Sequential Trigger Mode not supported in Master mode, switchig to default\n",
 __func__);
            break;
        }
        trigen = 1;  
        vint_en = 1;  
        dev_dbg(dev, "%s: Sensor is in Sequential Trigger Mode\n", __func__);
   
        break;
    default:
        pr_err("%s: unknown exposure mode.\n", __func__);
        return -EINVAL;
    }   

    err  = imx530_write_reg(s_data, TRIGMODE, trigen);
    err |= imx530_write_reg(s_data, VINT_EN, vint_en);
    if (err) {
        dev_err(dev, "%s: error setting exposure mode\n", __func__);  
        return err;        
    }

    return 0; 
}

/**
 * According to the V4L2 documentation, driver should not return error when 
 * invalid settings are detected.
 * It should apply the settings closest to the ones that the user has requested.
 */
static int imx530_check_unsupported_mode(struct camera_common_data *s_data,
                                     struct v4l2_mbus_framefmt *mf)
{
    struct crosslink_readout_mode *cl_readout_mode = &s_data->cl_priv->cl_readout_mode;
	struct device *dev = s_data->dev;
    bool unsupported_mode = false;

	dev_dbg(dev, "%s++\n", __func__);

    /**
     * Binning is supported only in 12 bit mode according to the chapter "Operation mode"
     * in the imx530 datasheet
     */

    if (mf->code == MEDIA_BUS_FMT_SRGGB10_1X10 
        && (s_data->mode == IMX530_MODE_5328x4608 || 
            s_data->mode == IMX530_MODE_ROI_5328x3040 ||
            s_data->mode == IMX530_MODE_BINNING_2656x2304)) {
        unsupported_mode = true;
        dev_warn(dev, 
        "%s: selected mode is not supported with RAW10, switching to default\n",
                  __func__);
    }

    if (unsupported_mode){
        mf->width	= s_data->frmfmt[s_data->def_mode].size.width;
        mf->height	= s_data->frmfmt[s_data->def_mode].size.height;
    }

    if (s_data->mode == IMX530_MODE_BINNING_2656x2304)
        cl_readout_mode->line_skip = IMX530_READOUT_LINE_SKIP_2;
    else
        cl_readout_mode->line_skip = IMX530_READOUT_LINE_SKIP;

    return 0;
}

/**
 * Set trigger t_tgpd ---> TBD in IMX530 Datasheet
 */
static int imx530_set_trigger_tgpd(struct tegracam_device *tc_dev)
{
	struct imx530 *priv = (struct imx530 *)tc_dev->priv;
    struct camera_common_data *s_data = tc_dev->s_data;

    switch (s_data->mode) {
    case IMX530_MODE_5328x4608:
    case IMX530_MODE_ROI_4512x4512:
    case IMX530_MODE_ROI_5328x3040:
    case IMX530_MODE_ROI_4064x3008:
    case IMX530_MODE_BINNING_2656x2304:
        priv->s_data->cl_priv->tg_params.t_tgpd = s_data->fmt_height 
                                + IMX530_MIN_FRAME_DELTA;
    }

    return 0;
}

static int imx530_set_mode(struct tegracam_device *tc_dev)
{
	struct imx530 *priv = (struct imx530 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

    err = crosslink_set_readout_mode(tc_dev);
	if (err)
		goto fail;

    err = crosslink_set_is_operation_mode(tc_dev);
	if (err)
		goto fail;

    err = imx530_write_table(priv, mode_table[IMX530_INIT_SETTINGS]);
    if (err) {
        dev_err(dev, "%s: unable to initialize sensor settings\n", __func__);
        return err;
    }
    
    err = imx530_set_pixel_format(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to write format to image sensor\n", __func__);
        return err;
    }

    err = imx530_set_dep_registers(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to write format to image sensor\n", __func__);
        return err;
    }

	err = imx530_write_table(priv, mode_table[s_data->mode]);
    if (err) {
        dev_err(dev, "%s: unable to set sensor mode settings\n", __func__);
        return err;
    }

    err = imx530_configure_shutter(tc_dev);
	if (err)
		goto fail;
    
    err = imx530_set_trigger_tgpd(tc_dev);
	if (err)
		goto fail;

    /* Override V4L GAIN, EXPOSURE and FRAME RATE controls */
    s_data->override_enable = true;

    err = imx530_calculate_line_time(tc_dev);
	if (err)
		goto fail;

    err = tg_set_line_width(tc_dev);
	if (err)
		goto fail;

    err = imx530_update_framerate_range(tc_dev);
	if (err)
		goto fail;

	dev_dbg(dev, "%s: set mode %u\n", __func__, s_data->mode);

	return 0;

fail:
    dev_err(dev, "%s: unable to set mode\n", __func__);
    return err;   
}

static int imx530_start_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	int err;

    err = imx530_write_reg(s_data, STANDBY, 0x00);

    /* "Internal regulator stabilization" time */
    usleep_range(1138000, 1140000);
    
    err |= crosslink_start(tc_dev);
   
    if (*tg_params->is_operation_mode == MASTER_MODE) {
        err |= imx530_write_reg(s_data, XMSTA, 0x00);
    } else{
        err |= tg_start(tc_dev);
    }

	if (err)
		return err;

	return 0;
}

static int imx530_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx530 *priv = (struct imx530 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx530_write_table(priv, mode_table[IMX530_MODE_STOP_STREAM]);

    err |= tg_stop(tc_dev);

    err |= crosslink_stop(tc_dev);

	if (err)
		return err;

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline 
	 */
	usleep_range(priv->frame_length * priv->line_time / IMX530_K_FACTOR, 
                priv->frame_length * priv->line_time / IMX530_K_FACTOR + 1000);

	return 0;
}


static struct camera_common_sensor_ops imx530_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx530_frmfmt),
	.frmfmt_table = imx530_frmfmt,
	.power_on = imx530_power_on,
	.power_off = imx530_power_off,
	.write_reg = imx530_write_reg,
	.read_reg = imx530_read_reg,
	.parse_dt = imx530_parse_dt,
	.power_get = imx530_power_get,
	.power_put = imx530_power_put,
	.set_mode = imx530_set_mode,
	.start_streaming = imx530_start_streaming,
	.stop_streaming = imx530_stop_streaming,
    .check_unsupported_mode = imx530_check_unsupported_mode,
};


static int imx530_board_setup(struct imx530 *priv)
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

	err = imx530_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

	imx530_power_off(s_data);
	camera_common_mclk_disable(s_data);

	return err;
}

static int imx530_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx530_subdev_internal_ops = {
	.open = imx530_open,
};

static int imx530_ctrls_init(struct tegracam_device *tc_dev)
{
	struct imx530 *priv = (struct imx530 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = priv->s_data;
	struct v4l2_ctrl_config *ctrl_cfg;
    struct v4l2_ctrl *ctrl;
    struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
    int numctrls;
    int err, i;

    numctrls = ARRAY_SIZE(imx530_custom_ctrl_list);

    for (i = 0; i < numctrls; i++) {
		ctrl_cfg = &imx530_custom_ctrl_list[i];

        ctrl = v4l2_ctrl_new_custom(&handler->ctrl_handler, ctrl_cfg, NULL);
        if (ctrl == NULL) {
            dev_err(dev, "%s: Failed to create control %s\n", __func__, 
                                                              ctrl_cfg->name);
            continue;
        }

        if (ctrl_cfg->type == V4L2_CTRL_TYPE_STRING && 
                ctrl_cfg->flags & V4L2_CTRL_FLAG_READ_ONLY) {
            ctrl->p_new.p_char = devm_kzalloc(tc_dev->dev, 
                                    ctrl_cfg->max + 1, GFP_KERNEL);
            if (!ctrl->p_new.p_char) {
                dev_err(dev, "%s: failed to allocate memory\n", __func__);
                return -ENOMEM;
            }
        }
        handler->ctrls[handler->numctrls + i] = ctrl;
        dev_dbg(dev, "%s: Added custom control %s to handler index: %d\n", 
            __func__, ctrl_cfg->name,  handler->numctrls + i);
    }

    handler->numctrls = handler->numctrls + numctrls;

	err = handler->ctrl_handler.error;
	if (err) {
		dev_err(dev, "Error %d adding controls\n", err);
		goto error;
	}

    return 0;

error:
	v4l2_ctrl_handler_free(&handler->ctrl_handler);
	return err;
}

static int imx530_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx530 *priv;
	struct sensor_control_properties *ctrlprops = NULL;
	int err;

	dev_info(dev, "probing v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx530), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx530", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx530_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx530_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx530_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

    priv->frame_length = IMX530_DEFAULT_HEIGHT 
                                + IMX530_MIN_FRAME_DELTA;
    priv->current_operation_mode = MASTER_MODE;
    priv->s_data->blklvl_max_range = IMX530_MAX_BLACK_LEVEL;
    priv->line_time = IMX530_DEFAULT_LINE_TIME;
    priv->min_frame_length = IMX530_DEFAULT_HEIGHT 
                                + IMX530_MIN_FRAME_DELTA;
    priv->s_data->def_mode = IMX530_MODE_ROI_4512x4512;

    priv->s_data->cl_priv = devm_kzalloc(dev,
			sizeof(struct crosslink_private), GFP_KERNEL);
	if (!priv->s_data->cl_priv)
		return -ENOMEM;

    priv->s_data->cl_priv->tg_params.tg_mode = TG_DISABLED;
    priv->s_data->cl_priv->tg_params.sync_logic = 0;
    priv->s_data->cl_priv->tg_params.out_logic = 0;
    priv->s_data->cl_priv->tg_params.xhs_min_active_width = 8;
    priv->s_data->cl_priv->tg_params.xhs_clk_offset = 1;
    priv->s_data->cl_priv->tg_params.frame_active_width = 2;
    priv->s_data->cl_priv->tg_params.line_time = &priv->line_time;
    priv->s_data->cl_priv->tg_params.is_operation_mode = &priv->current_operation_mode;
    priv->s_data->cl_priv->tg_params.is_shutter_mode = &priv->current_shutter_mode;

    priv->s_data->cl_priv->sensor_numch     = IMX530_LVDS_NUM_CHANNELS;
    priv->s_data->cl_priv->cl_readout_mode      = cl_def_readout_mode; 
    priv->s_data->cl_priv->cl_readout_mode.line_skip = IMX530_READOUT_LINE_SKIP;

    /* Get default device tree properties of first sensor mode */
    ctrlprops = 
		&priv->s_data->sensor_props.sensor_modes[0].control_properties;

    priv->s_data->exposure_min_range = ctrlprops->min_exp_time.val;
    priv->s_data->exposure_max_range = ctrlprops->max_exp_time.val;

	err = imx530_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

    err = imx530_ctrls_init(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera custom ctrl init failed\n");
		return err;
	}

	err = imx530_update_ctrl(tc_dev);
    if (err)
		return err;

    err = v4l2_async_register_subdev(priv->subdev);
    if (err) {
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }

	dev_info(dev, "Detected imx530 sensor\n");

	return 0;
}

static int
imx530_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx530 *priv = (struct imx530 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx530_id[] = {
	{ "imx530", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx530_id);

static struct i2c_driver imx530_i2c_driver = {
	.driver = {
		.name = "imx530",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx530_of_match),
	},
	.probe = imx530_probe,
	.remove = imx530_remove,
	.id_table = imx530_id,
};

module_i2c_driver(imx530_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX530");
MODULE_AUTHOR("FRAMOS GmbH");
MODULE_LICENSE("GPL v2");
