/*
 * imx296.c - imx296 sensor driver
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

#include "imx296_framos_mode_tbls.h"
#include "framos_sensor_common.h"

#define IMX296_K_FACTOR 1000LL
#define IMX296_M_FACTOR 1000000LL
#define IMX296_G_FACTOR 1000000000LL
#define IMX296_T_FACTOR 1000000000000LL

#define IMX296_MAX_GAIN_DEC 480
#define IMX296_MAX_GAIN_DB  48

#define IMX296_MAX_BLACK_LEVEL 0xFFF

#define IMX296_MIN_SHR0_LENGTH 4
#define IMX296_MIN_INTEGRATION_LINES 1
#define IMX296_INTEGRATION_OFFSET 14

#define IMX296_MAX_CSI_LANES 4
#define IMX296_TWO_LANE_MODE 2

#define IMX296_INCK 74250000LL

/**
 * list HEAD of private data of all probed sensors on the platform 
 */
LIST_HEAD(imx296_sensor_list);

/**
 * Declaration
 */
static int imx296_set_exposure(struct tegracam_device *tc_dev, s64 val);


static const struct of_device_id imx296_of_match[] = {
	{ .compatible = "framos,fsm-imx296",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx296_of_match);


static int imx296_set_custom_ctrls(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops imx296_custom_ctrl_ops = {
	.s_ctrl = imx296_set_custom_ctrls,
};

static const char * const imx296_test_pattern_menu[] = {
    [0] = "No Pattern",
    [1] = "Sequence 1",
    [2] = "Sequence 2",
    [3] = "Gradiation",
};

static struct v4l2_ctrl_config imx296_custom_ctrl_list[] = {
    {
        .ops = &imx296_custom_ctrl_ops,
        .id = TEGRA_CAMERA_CID_TEST_PATTERN,
        .name = "Test Pattern",
        .type = V4L2_CTRL_TYPE_MENU,
        .min = 0,
        .max = ARRAY_SIZE(imx296_test_pattern_menu) - 1,
        .def = 0,
        .qmenu = imx296_test_pattern_menu,
    },
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
    TEGRA_CAMERA_CID_STREAMING_MODE,
    TEGRA_CAMERA_CID_OPERATION_MODE,
    TEGRA_CAMERA_CID_BLACK_LEVEL,
    TEGRA_CAMERA_CID_SHUTTER_MODE,
};

struct imx296 {
	struct i2c_client	        *i2c_client;
	struct v4l2_subdev	        *subdev;
	u64				            frame_length;
    u64                         min_frame_length;
    u64				            black_level;
	u32				            line_time;
    streaming_mode              current_streaming_mode;
    operation_mode              current_operation_mode;
    shutter_mode                current_shutter_mode;
    struct list_head            entry; 
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline int imx296_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx296_write_reg(struct camera_common_data *s_data,
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
static int imx296_read_buffered_reg(struct camera_common_data *s_data,
                                 u16 addr_low, u8 number_of_registers, u64 *val)
{
	struct device *dev = s_data->dev;
    int err, i;
    u8 reg;

    *val = 0;

    if (!s_data->group_hold_active){
        err = imx296_write_reg(s_data, REGHOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: error setting register hold\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx296_read_reg(s_data, addr_low + i, &reg);
        *val += reg << (i * 8);
        if (err) {
            dev_err(dev, "%s: error reading buffered registers\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx296_write_reg(s_data, REGHOLD, 0x00);
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
static int imx296_write_buffered_reg(struct camera_common_data *s_data, 
                                u16 addr_low, u8 number_of_registers, u64 val)
{
    int err, i;
	struct device *dev = s_data->dev;

    if (!s_data->group_hold_active){
        err = imx296_write_reg(s_data, REGHOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx296_write_reg(s_data, addr_low + i, (u8)(val >> (i * 8)));
        if (err) {
            dev_err(dev, "%s: BUFFERED register write error\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx296_write_reg(s_data, REGHOLD, 0x00);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
            return err;
        }
    }

    return err;
}

static int imx296_write_table(struct imx296 *priv,
				const imx296_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 IMX296_TABLE_WAIT_MS,
					 IMX296_TABLE_END);
}

static int imx296_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

    s_data->group_hold_active = val;

    err = imx296_write_reg(s_data, REGHOLD, val);
    if (err) {
        dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
        return err;
    }

	return err;
}

static int imx296_update_ctrl(struct tegracam_device *tc_dev, u64 val)
{
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
    struct v4l2_ctrl *ctrl;

	/* Update Black level control*/
    ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL);
    if (ctrl) {
        *ctrl->p_new.p_s64 = val;
        *ctrl->p_cur.p_s64 = val;
		ctrl->default_value = priv->black_level;
    }

    return 0;
}

static int imx296_set_black_level(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

    err = imx296_write_buffered_reg(s_data, BLKLEVEL_LOW, 2, val);
	if (err){
	    dev_dbg(dev, "%s: BLACK LEVEL control error\n", __func__);
	    return err;
    }

	dev_dbg(dev, "%s: black level: %lld\n",  __func__, val);

	return 0;
}

static int imx296_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	int err;
	u32 gain;

	/* translate value */
	gain = val * IMX296_MAX_GAIN_DEC /
                 (IMX296_MAX_GAIN_DB *
                     mode->control_properties.gain_factor);

    err = imx296_write_buffered_reg(s_data, GAIN_LOW, 2, gain);
	if (err){
	    dev_dbg(dev, "%s: GAIN control error\n", __func__);
	    return err;
    }

	dev_dbg(dev, "%s:  gain val [%lld] reg [%d]\n",  __func__, val, gain);

	return 0;
}


static int imx296_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
    struct v4l2_ctrl *ctrl;
	int err;
	u32 integration_time_line;
    u32 integration_offset = IMX296_INTEGRATION_OFFSET;
	u32 reg_shs;

	dev_dbg(dev, "%s: integration time: %lld [us]\n", __func__, val);    

    /* Check value with internal range */
    if (val > s_data->exposure_max_range) {
        val = s_data->exposure_max_range;
    }
    else if (val < s_data->exposure_min_range) {
        val = s_data->exposure_min_range;
    }

	integration_time_line = ((val - integration_offset)
                                     * IMX296_K_FACTOR) / priv->line_time ;

	reg_shs = priv->frame_length - integration_time_line;

    if (reg_shs < IMX296_MIN_SHR0_LENGTH)
        reg_shs = IMX296_MIN_SHR0_LENGTH;
	else if (reg_shs > (priv->frame_length - IMX296_MIN_INTEGRATION_LINES))
		reg_shs = priv->frame_length - IMX296_MIN_INTEGRATION_LINES;

    err = imx296_write_buffered_reg (s_data, SHS1_LOW, 3, reg_shs);
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

static int imx296_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;
    u64 frame_length;
    u64 exposure_max_range, exposure_min_range;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];

    frame_length = (((u64)mode->control_properties.framerate_factor * 
                              IMX296_G_FACTOR) / (val * priv->line_time));

    if (frame_length < priv->min_frame_length)
        frame_length = priv->min_frame_length;
    
    priv->frame_length = frame_length;

    /* Update exposure range, before writing the new frame length */
    exposure_min_range = IMX296_MIN_INTEGRATION_LINES 
                                            * priv->line_time / IMX296_K_FACTOR;
    exposure_min_range += IMX296_INTEGRATION_OFFSET;   

    exposure_max_range = (priv->frame_length - IMX296_MIN_SHR0_LENGTH) 
                                            * priv->line_time / IMX296_K_FACTOR;
    exposure_max_range += IMX296_INTEGRATION_OFFSET; 

    fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_EXPOSURE,
                             exposure_min_range, exposure_max_range);

    /* Due to unexpected sensor behavior only all pixel mode can change 
       its frame rate*/
    if (s_data->mode == IMX296_MODE_1456x1088) {
        err = imx296_write_buffered_reg(s_data, VMAX_LOW, 3, priv->frame_length);
        if (err) {
            dev_err(dev, "%s: failed to set frame length\n", __func__);
            return err;
        }

	    dev_dbg(dev,
            "%s: val: %lld, frame_length set: %llu\n",
                 __func__, val, priv->frame_length);
    }   

	return 0;
}

/**
 * Test pattern is described in the "Pattern Generator (PG)" chapter
 * in the IMX296 support package
 */
static int imx296_set_test_pattern(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev; 
    int err;

    if (val) {
        err = imx296_write_reg(s_data, PATERN_GEN, (val << 3) + 7);
        if (err) 
            goto fail;
  
    } else {
        err = imx296_write_reg(s_data, PATERN_GEN, 6);
        if (err) 
            goto fail;
    }

	dev_dbg(dev, "%s++ Test mode pattern: %u\n", __func__, val-1);

    return 0;
fail:
    dev_err(dev, "%s: error setting test pattern\n", __func__);
    return err;
}

/**
 * Update max framerate range
 */
static int imx296_update_framerate_range(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	struct sensor_control_properties *ctrlprops = NULL;
    u64 max_framerate, min_framerate;

    ctrlprops = 
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

    priv->min_frame_length = s_data->fmt_height 
                            + IMX296_MIN_FRAME_LENGTH_DELTA;

    max_framerate = (IMX296_G_FACTOR * IMX296_M_FACTOR) /
                                     (priv->min_frame_length * priv->line_time);
    
    /* Frame rate could be controled only in all pixel mode */
    if (s_data->mode == IMX296_MODE_1456x1088) {
        min_framerate = ctrlprops->min_framerate;
    } 
    else {
        min_framerate = max_framerate;
    }

    fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_FRAME_RATE, 
                                        min_framerate, max_framerate);

    return 0;
}

/**
 * Set streaming mode 
 */
static int imx296_set_streaming_mode(struct tegracam_device *tc_dev, u32 val)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;

    priv->current_streaming_mode = val;

    return 0;
}

/**
 * Set operation mode of sensor 
 */
static int imx296_set_operation_mode(struct tegracam_device *tc_dev, u32 val)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;

    priv->current_operation_mode = val;

    return 0;
}

/**
 * Set supported shutter mode
 */
static int imx296_set_shutter_mode(struct tegracam_device *tc_dev, struct v4l2_ctrl *ctrl)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
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

static int imx296_set_custom_ctrls(struct v4l2_ctrl *ctrl)
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

static struct tegracam_ctrl_ops imx296_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx296_set_gain,
	.set_exposure = imx296_set_exposure,
	.set_frame_rate = imx296_set_frame_rate,
	.set_group_hold = imx296_set_group_hold,
    .set_test_pattern = imx296_set_test_pattern,
    .set_streaming_mode = imx296_set_streaming_mode,
    .set_operation_mode = imx296_set_operation_mode,
    .set_black_level = imx296_set_black_level,
    .set_shutter_mode = imx296_set_shutter_mode,
};

static int imx296_power_on(struct camera_common_data *s_data)
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

    if (!pw->mclk) {
        dev_err(dev, "%s: mclk not available\n",  __func__);
        return -ENODEV;
    }

    /* Power ON sequence according to IMX296 datasheet */

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto imx296_dvdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto imx296_iovdd_fail;
	}

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto imx296_avdd_fail;
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

imx296_avdd_fail:
	regulator_disable(pw->iovdd);

imx296_iovdd_fail:
	regulator_disable(pw->dvdd);

imx296_dvdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int imx296_power_off(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
        pr_info("power off callback\n");
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}

    /* Power OFF sequence according to IMX296 datasheet */

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
static int imx296_power_get(struct tegracam_device *tc_dev)
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

	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(dev, pw->reset_gpio);
		if (err)
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->reset_gpio);
	}

done:
	pw->state = SWITCH_OFF;
	return err;
}

/**
 * Frees regulators acquired in power_get
 */
static int imx296_power_put(struct tegracam_device *tc_dev)
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

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_deregister(dev, pw->reset_gpio);
	else {
		if (gpio_is_valid(pw->reset_gpio))
			gpio_free(pw->reset_gpio);
	}

	return 0;
}

/**
 * Read frame length register to confirm communication
 */
static int imx296_communication_verify(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
    int err;
    u64 vmax, black_level;

    err = imx296_read_buffered_reg(s_data, VMAX_LOW, 2, &vmax);
    if (err) {
        dev_err(dev, "%s: failed to read VMAX\n", __func__);
        return err;
    }
	/* Initialize frame length */
    priv->frame_length = vmax;

	err = imx296_read_buffered_reg(s_data, BLKLEVEL_LOW, 2, &black_level);
    if (err) {
        dev_err(dev, "%s: failed to read BLKLEVEL\n", __func__);
        return err;
    }
	/* Initialize black level */
	priv->black_level = black_level;

    return err;
}

static struct camera_common_pdata *imx296_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err;
	int gpio;
    const char *filter_info[10];

	if (!np)
		return NULL;

	match = of_match_device(imx296_of_match, dev);
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
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

    gpio = of_get_named_gpio(np, "vsync-gpios", 0);
    if (gpio > 0) {
        gpio_direction_input(gpio);
    }

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

    err = of_property_read_string(np, "sensor_filter_info", &filter_info[0]);
	if (err) {
		dev_info(dev, "%s: sensor_filter_info not available on platform\n",  __func__);
        err = 0;
	}

    if (!strcmp(filter_info[0], "color")) {
        board_priv_pdata->has_color_filter = true;
    }
    else {
        board_priv_pdata->has_color_filter = false;
    }
    
	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;
}

/**
 * Calculate 1H time
 */
static int imx296_calculate_line_time(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
    struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
    u64 hmax;
    int err;

	dev_dbg(dev, "%s:++\n", __func__);

    err = imx296_read_buffered_reg(s_data, HMAX_LOW, 2, &hmax);
    if (err) {
        dev_err(dev, "%s: unable to read hmax\n", __func__);
        return err;
    }

    priv->line_time = (hmax*IMX296_G_FACTOR) / (IMX296_INCK);


	dev_dbg(dev, "%s: hmax: %llu [inck], INCK: %u [Hz], line_time: %u [ns]\n",
            __func__, hmax, s_data->def_clk_freq, priv->line_time);

    return 0;
}

/**
 * Configure Global Shutter Operation
 * V interrupt is disabled in init mode table
 */
static int imx296_configure_shutter(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
    int err = 0;
    u8 trigen = 0;
    u8 trigger_sel = 0;

    switch (priv->current_shutter_mode) {
    case NORMAL_EXPO:
        trigen = 0; 
        trigger_sel = 0;         
        dev_dbg(dev, "%s: Sensor is in Normal Exposure Mode\n", __func__);

        break;

    case SEQ_TRIGGER:
        trigen = 1;  
        trigger_sel = 0;  
        dev_dbg(dev, "%s: Sensor is in Sequential Trigger Mode\n", __func__);
   
        break;
    default:
        pr_err("%s: unknown exposure mode.\n", __func__);
        return -EINVAL;
    }   

    err  = imx296_write_reg(s_data, TRIGEN, trigen);
    err |= imx296_write_reg(s_data, LOWLAGTRG, trigger_sel);
    if (err) {
        dev_err(dev, "%s: error setting exposure mode\n", __func__);  
        return err;        
    }

    return 0; 
}

/**
 * XVS & XHS are triggering pins
 *       XVS      XHS
 * 0x30 - hi,       hi
 * 0x00 - output,  output
 */
static int imx296_configure_triggering_pins(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
    struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
    int err;
    u8  xvs_xhs_drv = 0x30;

    switch (priv->current_operation_mode) {
    case MASTER_MODE:
        /* XVS - output, XHS - output */
        if (priv->current_streaming_mode == SYNC_STREAM)
            xvs_xhs_drv = 0;  
        else
            xvs_xhs_drv = 0x30;          
        dev_dbg(dev, "%s: Sensor is in Master mode\n", __func__);

        break;

    case SLAVE_MODE: 
        /* XVS - hi-z, XHS - hi-z */
        xvs_xhs_drv = 0x30;
        dev_dbg(dev, "%s: Sensor is in Slave mode\n", __func__);
   
        break;

    default:
        pr_err("%s: unknown synchronizing function.\n", __func__);
        return -EINVAL;
    }  

    err = imx296_write_reg(s_data, SYNCSEL, xvs_xhs_drv);
    if (err) {
        dev_err(dev, "%s: error setting XVS XHS pin\n", __func__);  
        return err;        
    }

    dev_dbg(dev, "%s: XVS_XHS driver register: %x\n", __func__, xvs_xhs_drv);

    return 0;
}

/**
 * Check that all sensors have the same streaming mode
 * Streaming in different modes could cause hardware damage
 */
static bool imx296_is_stream_configuration_valid(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
    struct imx296 *current_priv;
	struct device *dev = tc_dev->dev;
    u8 streaming_mode = priv->current_streaming_mode;

    list_for_each_entry(current_priv, &imx296_sensor_list, entry) {
        if (streaming_mode != current_priv->current_streaming_mode) {
            dev_err (dev,
                   "streaming mode not compatible with other sensors, all sensors must have the same streaming mode\n");
            return false;
        }       
    }

    return true;
}

/**
 * Check that two pins never drive the same line
 * Two pins driving the same line could cause hardware damage
 */
static bool imx296_is_pin_configuration_valid(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tc_dev->priv;
    struct imx296 *current_priv; 
	struct device *dev = tc_dev->dev;
    u8 xvs_drive = 0;
    u8 xhs_drive = 0;
    
    list_for_each_entry(current_priv, &imx296_sensor_list, entry) {
        if (current_priv->current_operation_mode == MASTER_MODE
            && current_priv->current_streaming_mode != STANDALONE_STREAM) {
            xvs_drive++;
            xhs_drive++;
            dev_dbg(dev, "%s: XVS & XHS as output\n", __func__);
        }
    }
    
    if (priv->current_streaming_mode == SYNC_STREAM 
            && (xvs_drive > 1 || xhs_drive > 1)) {
        dev_warn(dev, "More than one XVS/XHS are configured as OUTPUT!\n");
        return false;
    }
    else if (priv->current_streaming_mode == SYNC_STREAM 
                && (xvs_drive == 0 || xhs_drive == 0)) {
        dev_warn(dev, "Configure one sensor to drive XVS/XHS!\n");
        return false;
    }
    else if (priv->current_streaming_mode == EXTERNAL_HW_SYNC_STREAM 
                && (xvs_drive || xhs_drive)) {
        dev_warn(dev, "Configure all XVS/XHS as INPUT pin!\n");
        return false;
    }

    return true;
}

/**
 * According to the V4L2 documentation, driver should not return error when 
 * invalid settings are detected.
 * It should apply the settings closest to the ones that the user has requested.
 */
int imx296_check_unsupported_mode(struct camera_common_data *s_data,
                                     struct v4l2_mbus_framefmt *mf)
{
	struct device *dev = s_data->dev;
    bool unsupported_mode = false;

	dev_dbg(dev, "%s++\n", __func__);

    /**
     * Binning is supported only for monochrome sensor according to the 
     * imx296 datasheet
     */

    if (s_data->pdata->has_color_filter
        && s_data->mode == IMX296_MODE_H2V2_BINNING) {
        unsupported_mode = true;
        dev_warn(dev, 
        "%s: selected mode is not supported with color sensor, switching to default\n",
                  __func__);
    }

    if (unsupported_mode){
        mf->width	= s_data->frmfmt[s_data->def_mode].size.width;
        mf->height	= s_data->frmfmt[s_data->def_mode].size.height;
    }

    return 0;
}

static int imx296_set_mode(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

    err = imx296_write_table(priv, mode_table[IMX296_INIT_SETTINGS]);
    if (err) {
        dev_err(dev, "%s: unable to initialize sensor settings\n", __func__);
        return err;
    }

	err = imx296_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

    err = imx296_set_operation_mode(tc_dev, priv->current_operation_mode);
    if (err) {
        dev_err(dev, "%s: unable to operation mode\n", __func__);
        return err;   
    }
    
    err = imx296_configure_triggering_pins(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable configure XVS/XHS pins\n", __func__);
        return err;   
    }

    err = imx296_configure_shutter(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to configure exposure\n", __func__);
        return err;   
    }

    /* Override V4L GAIN, EXPOSURE and FRAME RATE controls */
    s_data->override_enable = true;

    if (!imx296_is_stream_configuration_valid(tc_dev)) {
        dev_err(dev, "%s: illegal stream configuration detected\n", __func__);
        return -EPERM;
    }

    if (!imx296_is_pin_configuration_valid(tc_dev)) {
        dev_err(dev, "%s: illegal pin configuration detected\n", __func__);
        return -EPERM;
    }

    err = imx296_calculate_line_time(priv->tc_dev);
	if (err)
		return err;

    err = imx296_update_framerate_range(tc_dev);
    if (err)
		return err;

	dev_dbg(dev, "%s: set mode %u\n", __func__, s_data->mode);

	return 0;
}

static int imx296_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx296_write_table(priv,
		mode_table[IMX296_MODE_START_STREAM]);
	if (err)
		return err;

	return 0;
}

static int imx296_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx296_write_table(priv, mode_table[IMX296_MODE_STOP_STREAM]);
	if (err)
		return err;

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline 
	 */
	usleep_range(priv->frame_length * priv->line_time / IMX296_K_FACTOR, 
                priv->frame_length * priv->line_time / IMX296_K_FACTOR + 1000);

	return 0;
}


static struct camera_common_sensor_ops imx296_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx296_frmfmt),
	.frmfmt_table = imx296_frmfmt,
	.power_on = imx296_power_on,
	.power_off = imx296_power_off,
	.write_reg = imx296_write_reg,
	.read_reg = imx296_read_reg,
	.parse_dt = imx296_parse_dt,
	.power_get = imx296_power_get,
	.power_put = imx296_power_put,
	.set_mode = imx296_set_mode,
	.start_streaming = imx296_start_streaming,
	.stop_streaming = imx296_stop_streaming,
    .check_unsupported_mode = imx296_check_unsupported_mode,
};


static int imx296_board_setup(struct imx296 *priv)
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

	err = imx296_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

    err = imx296_communication_verify(priv->tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to communicate with sensor\n",  __func__);
        goto error;
    }

    err = imx296_calculate_line_time(priv->tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to calculate line time\n", __func__);
        goto error;
    }

    priv->min_frame_length = IMX296_DEFAULT_HEIGHT 
                                + IMX296_MIN_FRAME_LENGTH_DELTA;

error:
	imx296_power_off(s_data);
	camera_common_mclk_disable(s_data);

	return err;
}

static int imx296_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx296_subdev_internal_ops = {
	.open = imx296_open,
};

static int imx296_ctrls_init(struct tegracam_device *tc_dev)
{
	struct imx296 *priv = (struct imx296 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = priv->s_data;
	struct v4l2_ctrl_config *ctrl_cfg;
    struct v4l2_ctrl *ctrl;
    struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
    int numctrls;
    int err, i;

    numctrls = ARRAY_SIZE(imx296_custom_ctrl_list);

    for (i = 0; i < numctrls; i++) {
		ctrl_cfg = &imx296_custom_ctrl_list[i];

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

static int imx296_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx296 *priv;
	struct sensor_control_properties *ctrlprops = NULL;
	int err;

	dev_info(dev, "probing v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx296), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx296", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx296_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx296_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx296_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

    priv->current_streaming_mode = STANDALONE_STREAM;
    priv->current_operation_mode = MASTER_MODE;
    priv->current_shutter_mode = NORMAL_EXPO;
    priv->s_data->blklvl_max_range = IMX296_MAX_BLACK_LEVEL;

    /* Get default device tree properties of first sensor mode */
    ctrlprops = 
		&priv->s_data->sensor_props.sensor_modes[0].control_properties;

    priv->s_data->exposure_min_range = ctrlprops->min_exp_time.val;
    priv->s_data->exposure_max_range = ctrlprops->max_exp_time.val;
    
    INIT_LIST_HEAD(&priv->entry);

	err = imx296_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

    err = imx296_ctrls_init(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera custom ctrl init failed\n");
		return err;
	}

    err = imx296_update_ctrl(tc_dev, priv->black_level);
    if (err)
		return err;

    err = v4l2_async_register_subdev(priv->subdev);
    if (err) {
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }

    list_add_tail(&priv->entry, &imx296_sensor_list);

	dev_info(dev, "Detected imx296 sensor\n");

	return 0;
}

static int
imx296_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx296 *priv = (struct imx296 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx296_id[] = {
	{ "fsm-imx296", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx296_id);

static struct i2c_driver imx296_i2c_driver = {
	.driver = {
		.name = "fsm-imx296",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx296_of_match),
	},
	.probe = imx296_probe,
	.remove = imx296_remove,
	.id_table = imx296_id,
};

module_i2c_driver(imx296_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX296");
MODULE_AUTHOR("FRAMOS GmbH");
MODULE_LICENSE("GPL v2");
