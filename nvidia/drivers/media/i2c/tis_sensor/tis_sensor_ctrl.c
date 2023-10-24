
#include "tis_sensor.h"
#include "tis_sensor_ctrl.h"

#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/gpio/consumer.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

static const char* tis_sensor_ctrl_trigger_mode_values[] =
{	
	"Off",
	"On"
};

static const char* tis_sensor_ctrl_strobe_mode_values[] =
{	
	"Exposure",
	"Fixed Duration"
};

static const char* tis_sensor_ctrl_trigger_source_values[] =
{	
	"FPD-Link",
	"PicoBlade"
};

static const char* tis_sensor_ctrl_trigger_in_polarity_values[] =
{	
	"Default",
	"Invert"
};

static const char* tis_sensor_ctrl_gpout_mode_values[] =
{	
	"Open Drain",
	"TTL"
};

static const char* tis_sensor_ctrl_gpout_function_values[] =
{
	"Constant Low",
	"Constant High",
	"Strobe",
	"Sync Out"
};

static const char* tis_sensor_ctrl_trigger_exposure_mode_values[] =
{
	"Timed",
	"Trigger Width"
};

static const char* tis_sensor_ctrl_strobe_polarity_values[] =
{
	"Active High",
	"Active Low"
};

static const char* tis_sensor_ctrl_ext_sync_values[] =
{
	"Off",
	"Master",
	"Slave"
};

static const char* tis_sensor_ctrl_gain_mode_values[] =
{
	"Low Conversion Gain",
	"High Conversion Gain"
};

static int tis_sensor_ctrl_get_device_valid(struct tis_sensor *priv, int* val);

int tis_sensor_ctrl_do_lazy_init(struct tis_sensor* priv)
{
	struct device *dev = priv->dev;
	int err = 0;

	if( !priv->ctrl.lazy_init )
	{
		dev_dbg(dev, "%s: priv->ctrl.lazy_init is NULL, skipping", __func__);
		return 0;
	}

	if( !tis_sensor_is_device_valid(priv) )
	{
		dev_err(dev, "%s: Device not valid, cancelling", __func__);
		return -ENODEV;
	}

	mutex_lock(&priv->ctrl.lazy_init_lock);

	if( priv->ctrl.lazy_init )
	{
		dev_info(dev, "%s: Running lazy sensor initialization", __func__);

		err = priv->ctrl.lazy_init(priv);

		if( !err )
		{
			priv->ctrl.lazy_init = 0;
		}
	}

	mutex_unlock(&priv->ctrl.lazy_init_lock);

	return err;
}
EXPORT_SYMBOL(tis_sensor_ctrl_do_lazy_init);

static int tis_sensor_ctrl_set_trigger(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_trigger )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.trigger_mode = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to configure sensor if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_trigger( priv );
}

static int tis_sensor_ctrl_set_strobe(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_strobe )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.strobe_mode = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to configure sensor if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_strobe( priv );
}

static int tis_sensor_ctrl_do_software_trigger(struct tis_sensor* priv)
{
	if( !priv->ctrl.ops->sensor_do_software_trigger )
	{
		dev_err(priv->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to fire if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_do_software_trigger(priv);
}


static int tis_sensor_ctrl_set_trigger_source(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_trigger_source )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.trigger_source = val;

	return priv->ctrl.ops->sensor_apply_trigger_source( priv );
}

static int tis_sensor_ctrl_set_trigger_in_polarity(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_trigger_in_polarity )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.trigger_in_polarity = val;

	return priv->ctrl.ops->sensor_apply_trigger_in_polarity( priv );
}

static int tis_sensor_ctrl_set_gpout_mode(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_gpout_mode )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.gpout_mode = val;

	return priv->ctrl.ops->sensor_apply_gpout_mode( priv );
}

static int tis_sensor_ctrl_set_gpout_function(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_gpout_function )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.gpout_function = val;

	return priv->ctrl.ops->sensor_apply_gpout_function( priv );
}

static int tis_sensor_ctrl_set_offset_x(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_offsets )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.offset_x = val;

	return priv->ctrl.ops->sensor_apply_offsets( priv );
}

static int tis_sensor_ctrl_set_offset_y(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_offsets )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.offset_y = val;

	return priv->ctrl.ops->sensor_apply_offsets( priv );
}

static int tis_sensor_ctrl_set_offset_auto_center(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_offsets )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.offset_auto_center = (val != 0);

	return priv->ctrl.ops->sensor_apply_offsets( priv );
}

static int tis_sensor_ctrl_set_black_level(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_black_level )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.black_level = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_black_level( priv );
}

static int tis_sensor_ctrl_set_trigger_exposure_mode(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_trigger_exposure_mode )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.trigger_exposure_mode = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_trigger_exposure_mode( priv );
}

static int tis_sensor_ctrl_set_strobe_polarity(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_strobe_polarity )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.strobe_polarity = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_strobe_polarity( priv );
}

static int tis_sensor_ctrl_set_strobe_delay(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_strobe_delay )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.strobe_delay = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_strobe_delay(priv);
}

static int tis_sensor_ctrl_set_strobe_duration(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_strobe_duration )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.strobe_duration = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_strobe_duration(priv);
}

static int tis_sensor_ctrl_set_ext_sync(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_ext_sync )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.ext_sync = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_ext_sync(priv);
}

static int tis_sensor_ctrl_set_gain_mode(struct tis_sensor* priv, int val)
{
	if( !priv->ctrl.ops->sensor_apply_gain_mode )
	{
		dev_err(priv->tc_dev->dev, "%s: Call unexpected, no handler registered", __func__);
		return -EINVAL;
	}

	priv->ctrl.gain_mode = val;

	if( !tis_sensor_is_power_on(priv) )
	{
		// Don't try to apply if power is not on
		return 0;
	}

	return priv->ctrl.ops->sensor_apply_gain_mode(priv);
}

static int tis_sensor_ctrl_get_device_valid(struct tis_sensor *priv, int* val)
{
	*val = tis_sensor_is_device_valid(priv) ? 1 : 0;

	return 0;
}

static int tis_sensor_ctrl_v4l2_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tis_sensor* priv = (struct tis_sensor*)ctrl->priv;
	struct device* dev = priv->tc_dev->dev;

	dev_dbg(dev, "%s: id = %x", __func__, ctrl->id);

	switch( ctrl->id )
	{
		case TIS_SENSOR_CID_DEVICE_VALID:
			return tis_sensor_ctrl_get_device_valid(priv, &ctrl->val);
	}

	dev_err(dev, "%s: Unexpected: id = %d", __func__, ctrl->id);

	return -EINVAL;
}

static int tis_sensor_ctrl_v4l2_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tis_sensor* priv = (struct tis_sensor*)ctrl->priv;
	struct device* dev = priv->dev;

	dev_dbg(dev, "%s: id = %x val = %d", __func__, ctrl->id, ctrl->val);

	if( ctrl->flags & V4L2_CTRL_FLAG_DISABLED )
	{
		dev_dbg(dev, "%s: s_ctrl for disabled ctrl id = %x, ignoring", __func__, ctrl->id);		
		return 0;		
	}

	switch( ctrl->id )
	{
		case TIS_SENSOR_CID_TRIGGER_MODE:
			return tis_sensor_ctrl_set_trigger( priv, ctrl->val );
		case TIS_SENSOR_CID_STROBE_MODE:
			return tis_sensor_ctrl_set_strobe( priv, ctrl->val );
		case TIS_SENSOR_CID_TRIGGER_SOFTWARE:
			return tis_sensor_ctrl_do_software_trigger( priv );
		case TIS_SENSOR_CID_TRIGGER_SOURCE:
			return tis_sensor_ctrl_set_trigger_source( priv, ctrl->val );
		case TIS_SENSOR_CID_TRIGGER_IN_POLARITY:
			return tis_sensor_ctrl_set_trigger_in_polarity( priv, ctrl->val );
		case TIS_SENSOR_CID_GPOUT_MODE:
			return tis_sensor_ctrl_set_gpout_mode( priv, ctrl->val );
		case TIS_SENSOR_CID_GPOUT_FUNCTION:
			return tis_sensor_ctrl_set_gpout_function( priv, ctrl->val );
		case TIS_SENSOR_CID_OFFSET_X:
			return tis_sensor_ctrl_set_offset_x( priv, ctrl->val );
		case TIS_SENSOR_CID_OFFSET_Y:
			return tis_sensor_ctrl_set_offset_y( priv, ctrl->val );
		case TIS_SENSOR_CID_OFFSET_AUTO_CENTER:
			return tis_sensor_ctrl_set_offset_auto_center( priv, ctrl->val );
		case TIS_SENSOR_CID_BLACK_LEVEL:
			return tis_sensor_ctrl_set_black_level( priv, ctrl->val );
		case TIS_SENSOR_CID_TRIGGER_EXPOSURE_MODE:
			return tis_sensor_ctrl_set_trigger_exposure_mode( priv, ctrl->val );
		case TIS_SENSOR_CID_STROBE_POLARITY:
			return tis_sensor_ctrl_set_strobe_polarity( priv, ctrl->val );
		case TIS_SENSOR_CID_STROBE_DELAY:
			return tis_sensor_ctrl_set_strobe_delay( priv, ctrl->val );
		case TIS_SENSOR_CID_STROBE_DURATION:
			return tis_sensor_ctrl_set_strobe_duration( priv, ctrl->val );
		case TIS_SENSOR_CID_EXT_SYNC:
			return tis_sensor_ctrl_set_ext_sync( priv, ctrl->val );
		case TIS_SENSOR_CID_GAIN_MODE:
			return tis_sensor_ctrl_set_gain_mode( priv, ctrl->val );
	}

	dev_err(dev, "%s: Unexpected: id = %d val = %d", __func__, ctrl->id, ctrl->val);

	return -EINVAL;
}

const struct v4l2_ctrl_ops tis_sensor_ctrl_v4l2_ctrl_ops = {
	.g_volatile_ctrl = tis_sensor_ctrl_v4l2_g_volatile_ctrl,
	.s_ctrl = tis_sensor_ctrl_v4l2_s_ctrl
};

int tis_sensor_ctrl_delayed_init(struct tis_sensor *priv)
{
	struct device *dev = priv->dev;	
	struct tis_sensor_ctrl_config config = {};
	int err;

	err = tis_sensor_fill_eeprom_info(priv);
	if( err )
	{
		dev_err(dev, "%s: Failed to query EEPROM (%d) - device not present?", __func__, err);
		goto put_gpios;
	}

	config.has_picoblade_connector = priv->adapter_board_part.caps->has_picoblade_connector;
	config.has_fpdlink = priv->adapter_board_part.caps->has_fpdlink;
	config.offset_x_max = priv->SENSOR_WIDTH;
	config.offset_y_max = priv->SENSOR_HEIGHT;
	
	if( priv->sensor_ops->sensor_update_common_ctrl_config )
	{
		priv->sensor_ops->sensor_update_common_ctrl_config(priv, &config);
	}

	err = tis_sensor_ctrl_update_controls(priv, &config);
	if( err )
	{
		dev_err(dev, "%s: tis_sensor_ctrl_update_controls failed: %d", __func__, err);
		goto put_gpios;
	}

	if( priv->sensor_ops->sensor_update_private_controls )
	{
		priv->sensor_ops->sensor_update_private_controls(priv);
	}

	return err;

put_gpios:
	tis_sensor_gpios_put(priv);

	return err;
}

int tis_sensor_ctrl_init(struct tis_sensor* priv)
{
	mutex_init(&priv->ctrl.lazy_init_lock);
	priv->ctrl.lazy_init = tis_sensor_ctrl_delayed_init;

	tis_sensor_ctrl_do_lazy_init(priv);

	return 0;
}
EXPORT_SYMBOL(tis_sensor_ctrl_init);


struct v4l2_ctrl *tis_sensor_ctrl_add_bool(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, bool default_value, const struct v4l2_ctrl_ops* ops )
{
	struct v4l2_ctrl* ctrl;	
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.id = cid,
		.name = name,
		.flags = V4L2_CTRL_FLAG_DISABLED,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def = default_value ? 1 : 0,
		.min = 0,
		.max = 1,
		.step = 1
	};	

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->tc_dev->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		return ERR_PTR(handler->error);
	}

    return ctrl;
}
EXPORT_SYMBOL(tis_sensor_ctrl_add_bool);

struct v4l2_ctrl *tis_sensor_ctrl_add_int(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, int min_value, int max_value, int step, int default_value, const struct v4l2_ctrl_ops* ops )
{
	struct v4l2_ctrl* ctrl;
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.id = cid,
		.name = name,
		.flags = V4L2_CTRL_FLAG_DISABLED,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = default_value,
		.min = min_value,
		.max = max_value,
		.step = step
	};	

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		return ERR_PTR(handler->error);
	}

    return ctrl;
}
EXPORT_SYMBOL(tis_sensor_ctrl_add_int);

struct v4l2_ctrl *tis_sensor_ctrl_add_menu(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const char* const *items, int num_items, int default_index, int skip_mask, const struct v4l2_ctrl_ops* ops )
{
	struct v4l2_ctrl* ctrl;
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.id = cid,
		.name = name,
		.flags = V4L2_CTRL_FLAG_DISABLED,
		.type = V4L2_CTRL_TYPE_MENU,
		.def = default_index,
		.min = 0,
		.max = num_items - 1,	
		.qmenu = items,
		.menu_skip_mask = skip_mask
	};	

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		dev_err(priv->dev, "(def = %lld, min = %lld, max = %lld, skip_mask = %lld)", cfg.def, cfg.min, cfg.max, cfg.menu_skip_mask);
		return ERR_PTR(handler->error);
	}

    return ctrl;
}
EXPORT_SYMBOL(tis_sensor_ctrl_add_menu);

struct v4l2_ctrl *tis_sensor_ctrl_add_string_const(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const char* value)
{
	int len = strlen(value);
	struct v4l2_ctrl* ctrl;
	struct v4l2_ctrl_config cfg = {
		.id = cid,
		.name = name,
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_DISABLED,
		.min = len,
		.max = len,
		.step = 1
	};

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		return ERR_PTR(handler->error);
	}

	ctrl->p_cur.p_char = (char*)value;

    return ctrl;
}

struct v4l2_ctrl *tis_sensor_ctrl_add_u32_1d_const(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const u32* data, int count)
{
	struct v4l2_ctrl* ctrl;
	struct v4l2_ctrl_config cfg = {
		.id = cid,
		.name = name,
		.type = V4L2_CTRL_TYPE_U32,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_DISABLED,
		.min = 0,
		.max = 0xFFFFFFFF,
		.step = 1,
		.def = 0,
		.dims = { count }
	};

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		return ERR_PTR(handler->error);
	}

	ctrl->p_cur.p_u32 = (u32*)data;

    return ctrl;
}

struct v4l2_ctrl *tis_sensor_ctrl_add_button(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const struct v4l2_ctrl_ops* ops )
{
	struct v4l2_ctrl* ctrl;
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.id = cid,
		.flags = V4L2_CTRL_FLAG_DISABLED,
		.name = name,
		.type = V4L2_CTRL_TYPE_BUTTON
	};

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		return ERR_PTR(handler->error);
	}

    return ctrl;
}

struct v4l2_ctrl *tis_sensor_ctrl_add_bool_volatile_readonly(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const struct v4l2_ctrl_ops* ops)
{
	struct v4l2_ctrl* ctrl;	
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.id = cid,
		.name = name,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def = 0,
		.min = 0,
		.max = 1,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY
	};	

	handler->error = 0;
	ctrl = v4l2_ctrl_new_custom(handler, &cfg, priv);
	if( !ctrl )
	{
		dev_err(priv->dev, "%s: v4l2_ctrl_new_custom failed, error = %d", __func__, handler->error);
		return ERR_PTR(handler->error);
	}

    return ctrl;	
}

static void tis_sensor_ctrl_update_string(struct v4l2_ctrl *ctrl, const char *s)
{
	WARN_ON(ctrl == NULL);
	if( !ctrl )
		return;

	ctrl->minimum = ctrl->maximum = strlen(s);
	ctrl->elem_size = ctrl->maximum + 1;
	ctrl->p_cur.p_char = (char*)s;
	ctrl->flags &= ~V4L2_CTRL_FLAG_DISABLED;
}

static void tis_sensor_ctrl_update_u32(struct v4l2_ctrl *ctrl, int min, int max, int def, int step)
{
	WARN_ON(ctrl == NULL);
	if( !ctrl )
		return;

	ctrl->minimum = min;
	ctrl->maximum = max;
	ctrl->cur.val = ctrl->default_value = def;
	ctrl->step = step;
	ctrl->flags &= ~V4L2_CTRL_FLAG_DISABLED;
}

static void tis_sensor_ctrl_update_u32_1d_const(struct v4l2_ctrl *ctrl, const u32 *arr, int num)
{
	WARN_ON(ctrl == NULL);
	if( !ctrl )
		return;
	
	ctrl->dims[0] = num;
	ctrl->p_cur.p_u32 = (u32*)arr;
	ctrl->flags &= ~V4L2_CTRL_FLAG_DISABLED;
}

static void tis_sensor_ctrl_update_menu(struct v4l2_ctrl *ctrl, int def, u64 skip_mask)
{
	WARN_ON(ctrl == NULL);
	if( !ctrl )
		return;

	ctrl->cur.val = ctrl->default_value = def;
	ctrl->menu_skip_mask = skip_mask;
	ctrl->flags &= ~V4L2_CTRL_FLAG_DISABLED;
}

void tis_sensor_ctrl_enable(struct v4l2_ctrl *ctrl)
{
	WARN_ON(ctrl == NULL);
	if( !ctrl )
		return;

	ctrl->flags &= ~V4L2_CTRL_FLAG_DISABLED;
}
EXPORT_SYMBOL(tis_sensor_ctrl_enable);

void tis_sensor_ctrl_disable(struct v4l2_ctrl *ctrl)
{
	WARN_ON(ctrl == NULL);
	if( !ctrl )
		return;

	ctrl->flags |= V4L2_CTRL_FLAG_DISABLED;
}
EXPORT_SYMBOL(tis_sensor_ctrl_disable);


int tis_sensor_ctrl_create_all(struct tis_sensor *priv, struct v4l2_ctrl_handler *handler)
{
	priv->ctrl.ctrl_exposure = v4l2_ctrl_find(handler, TEGRA_CAMERA_CID_EXPOSURE);
	priv->ctrl.ctrl_gain = v4l2_ctrl_find(handler, TEGRA_CAMERA_CID_GAIN);

	priv->ctrl.ctrl_device_valid = tis_sensor_ctrl_add_bool_volatile_readonly(priv, handler, TIS_SENSOR_CID_DEVICE_VALID, "device_valid", &tis_sensor_ctrl_v4l2_ctrl_ops);
	tis_sensor_ctrl_enable(priv->ctrl.ctrl_device_valid);

	priv->ctrl.ctrl_device_serial_number = tis_sensor_ctrl_add_string_const(priv, handler, TIS_SENSOR_CID_DEVICE_SERIAL_NUMBER, "Device Serial Number", "");
	priv->ctrl.ctrl_device_model_name = tis_sensor_ctrl_add_string_const(priv, handler, TIS_SENSOR_CID_DEVICE_MODEL_NAME, "Device Model Name", "");
	priv->ctrl.ctrl_device_vendor_name = tis_sensor_ctrl_add_string_const(priv, handler, TIS_SENSOR_CID_DEVICE_VENDOR_NAME, "Device Vendor Name", "");

	priv->ctrl.ctrl_mode_raw_fourcc = tis_sensor_ctrl_add_u32_1d_const(priv, handler, TIS_SENSOR_CID_MODE_RAW_FOURCC, "mode_raw_fourcc", priv->raw_fourcc, priv->s_data->numfmts);

	priv->ctrl.ctrl_trigger_mode = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_TRIGGER_MODE, "Trigger Mode",
					tis_sensor_ctrl_trigger_mode_values, ARRAY_SIZE(tis_sensor_ctrl_trigger_mode_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_strobe_mode = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_STROBE_MODE, "Strobe Mode",
					tis_sensor_ctrl_strobe_mode_values, ARRAY_SIZE(tis_sensor_ctrl_strobe_mode_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_strobe_delay = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_STROBE_DELAY, "Strobe Delay",
					0, 0, 1, 0,
					&tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_strobe_duration = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_STROBE_DURATION, "Strobe Duration",
					0, 0, 1, 0,
					&tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_trigger_software = tis_sensor_ctrl_add_button(priv, handler, TIS_SENSOR_CID_TRIGGER_SOFTWARE, "Trigger Software", &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_exposure_mode = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_TRIGGER_EXPOSURE_MODE, "Exposure Mode",
					tis_sensor_ctrl_trigger_exposure_mode_values, ARRAY_SIZE(tis_sensor_ctrl_trigger_exposure_mode_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_strobe_polarity = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_STROBE_POLARITY, "Strobe Polarity",
					tis_sensor_ctrl_strobe_polarity_values, ARRAY_SIZE(tis_sensor_ctrl_strobe_polarity_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_trigger_source = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_TRIGGER_SOURCE, "Trigger Source",
					tis_sensor_ctrl_trigger_source_values, ARRAY_SIZE(tis_sensor_ctrl_trigger_source_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_trigger_in_polarity = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_TRIGGER_IN_POLARITY, "Trigger In Polarity",
					tis_sensor_ctrl_trigger_in_polarity_values, ARRAY_SIZE(tis_sensor_ctrl_trigger_in_polarity_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_gpout_mode = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_GPOUT_MODE, "GPOut Mode",
					tis_sensor_ctrl_gpout_mode_values, ARRAY_SIZE(tis_sensor_ctrl_gpout_mode_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_gpout_function = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_GPOUT_FUNCTION, "GPOut Function",
					tis_sensor_ctrl_gpout_function_values, ARRAY_SIZE(tis_sensor_ctrl_gpout_function_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_offset_x = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_OFFSET_X, "Offset X", 0, 0, 1, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);
	priv->ctrl.ctrl_offset_y = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_OFFSET_Y, "Offset Y", 0, 0, 1, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);
	priv->ctrl.ctrl_offset_auto_center = tis_sensor_ctrl_add_bool(priv, handler, TIS_SENSOR_CID_OFFSET_AUTO_CENTER, "Offset Auto Center", true, &tis_sensor_ctrl_v4l2_ctrl_ops);	

	priv->ctrl.ctrl_black_level = tis_sensor_ctrl_add_int(priv, handler, TIS_SENSOR_CID_BLACK_LEVEL, "Black Level", 0, 0, 1, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_ext_sync = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_EXT_SYNC, "External Synchronization",
					tis_sensor_ctrl_ext_sync_values, ARRAY_SIZE(tis_sensor_ctrl_ext_sync_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	priv->ctrl.ctrl_gain_mode = tis_sensor_ctrl_add_menu(priv, handler, TIS_SENSOR_CID_GAIN_MODE, "Gain Mode",
					tis_sensor_ctrl_gain_mode_values, ARRAY_SIZE(tis_sensor_ctrl_gain_mode_values),
					0, 0, &tis_sensor_ctrl_v4l2_ctrl_ops);

	return 0;
}
EXPORT_SYMBOL(tis_sensor_ctrl_create_all);

int tis_sensor_ctrl_update_controls(struct tis_sensor *priv, struct tis_sensor_ctrl_config *config)
{
	BUG_ON(config == NULL);

	tis_sensor_ctrl_update_string(priv->ctrl.ctrl_device_serial_number, priv->eeprom_info.serial_number);
	tis_sensor_ctrl_update_string(priv->ctrl.ctrl_device_model_name, priv->eeprom_info.display_name);
	tis_sensor_ctrl_update_string(priv->ctrl.ctrl_device_vendor_name, priv->eeprom_info.manufacturer_name);

	tis_sensor_update_raw_fourcc(priv);
	tis_sensor_ctrl_update_u32_1d_const(priv->ctrl.ctrl_mode_raw_fourcc, priv->raw_fourcc, priv->s_data->numfmts);

	priv->ctrl.ops = config->ops;

	if( config->ops && config->ops->sensor_apply_trigger )
	{
		tis_sensor_ctrl_enable(priv->ctrl.ctrl_trigger_mode);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_trigger_mode);
	}

	if( config->ops && config->ops->sensor_apply_strobe )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_strobe_mode, config->strobe_mode_default, config->strobe_mode_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_trigger_mode);
	}

	if( config->ops && config->ops->sensor_apply_strobe_delay )
	{
		tis_sensor_ctrl_update_u32(priv->ctrl.ctrl_strobe_delay, config->strobe_delay_min, config->strobe_delay_max, config->strobe_delay_default, 1);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_strobe_delay);
	}

	if( config->ops && config->ops->sensor_apply_strobe_duration )
	{
		tis_sensor_ctrl_update_u32(priv->ctrl.ctrl_strobe_duration, config->strobe_duration_min, config->strobe_duration_max, config->strobe_duration_default, 1);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_strobe_duration);
	}

	if( config->ops && config->ops->sensor_do_software_trigger )
	{
		tis_sensor_ctrl_enable(priv->ctrl.ctrl_trigger_software);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_trigger_software);
	}

	if( config->ops && config->ops->sensor_apply_trigger_exposure_mode )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_exposure_mode, config->trigger_exposure_mode_default, config->trigger_exposure_mode_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_exposure_mode);
	}

	if( config->ops && config->ops->sensor_apply_strobe_polarity )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_strobe_polarity, config->strobe_polarity_default, config->strobe_polarity_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_strobe_polarity);
	}

	if( config->ops && config->ops->sensor_apply_trigger_source )
	{
		int extra_skip_mask = 0;
		int default_value = config->trigger_source_default;
		
		if( !config->has_picoblade_connector )
		{
			extra_skip_mask |= (1 << TIS_SENSOR_CTRL_TRIGGER_SOURCE_PICOBLADE);
		}
		if( !config->has_fpdlink )
		{
			extra_skip_mask |= (1 << TIS_SENSOR_CTRL_TRIGGER_SOURCE_FPDLINK);
			default_value = TIS_SENSOR_CTRL_TRIGGER_SOURCE_PICOBLADE;

			priv->ctrl.trigger_source = TIS_SENSOR_CTRL_TRIGGER_SOURCE_PICOBLADE;
		}

		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_trigger_source, default_value, config->trigger_source_skip_mask | extra_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_trigger_source);
	}

	if( config->ops && config->ops->sensor_apply_trigger_in_polarity && config->has_picoblade_connector )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_trigger_in_polarity, config->trigger_in_polarity_default, config->trigger_in_polarity_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_trigger_in_polarity);
	}

	if( config->ops && config->ops->sensor_apply_gpout_mode && config->has_picoblade_connector )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_gpout_mode, config->gpout_mode_default, config->gpout_mode_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_gpout_mode);
	}

	if( config->ops && config->ops->sensor_apply_gpout_function && config->has_picoblade_connector )
	{
		int extra_skip_mask = 0;

		if( !config->ops->sensor_apply_strobe )
		{
			extra_skip_mask |= (1 << TIS_SENSOR_CTRL_GPOUT_FUNCTION_STROBE);
		}
		if( !config->ops->sensor_apply_ext_sync || (config->ext_sync_skip_mask & (1 << TIS_SENSOR_CTRL_EXT_SYNC_MASTER)) )
		{
			extra_skip_mask |= (1 << TIS_SENSOR_CTRL_GPOUT_FUNCTION_SYNC_OUT);
		}

		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_gpout_function, config->gpout_function_default, config->gpout_function_skip_mask | extra_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_gpout_function);
	}

	if( config->ops && config->ops->sensor_apply_offsets )
	{
		priv->ctrl.offset_auto_center = true;

		tis_sensor_ctrl_update_u32(priv->ctrl.ctrl_offset_x, 0, config->offset_x_max, 0, config->offset_x_step);
		tis_sensor_ctrl_update_u32(priv->ctrl.ctrl_offset_y, 0, config->offset_y_max, 0, config->offset_y_step);
		tis_sensor_ctrl_enable(priv->ctrl.ctrl_offset_auto_center);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_offset_x);
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_offset_y);
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_offset_auto_center);
	}

	if( config->ops && config->ops->sensor_apply_black_level )
	{
		tis_sensor_ctrl_update_u32(priv->ctrl.ctrl_black_level, config->black_level_min, config->black_level_max, config->black_level_default, 1);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_black_level);
	}

	if( config->ops && config->ops->sensor_apply_ext_sync )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_ext_sync, config->ext_sync_default, config->ext_sync_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_ext_sync);
	}

	if( config->ops && config->ops->sensor_apply_gain_mode )
	{
		tis_sensor_ctrl_update_menu(priv->ctrl.ctrl_gain_mode, config->gain_mode_default, config->gain_mode_skip_mask);
	}
	else
	{
		tis_sensor_ctrl_disable(priv->ctrl.ctrl_gain_mode);
	}

	v4l2_ctrl_handler_setup(priv->tc_dev->s_data->ctrl_handler);

	return 0;
}
EXPORT_SYMBOL(tis_sensor_ctrl_update_controls);


void tis_sensor_ctrl_do_offset_auto_center(struct tis_sensor *priv)
{
	if( priv->ctrl.offset_auto_center )
	{
		int x_step = priv->ctrl.ctrl_offset_x->step;
		int y_step = priv->ctrl.ctrl_offset_x->step;

		priv->ctrl.offset_x = ((priv->SENSOR_WIDTH - priv->mode_active_w) / 2) / x_step * x_step;
		priv->ctrl.offset_y = ((priv->SENSOR_HEIGHT - priv->mode_active_h) / 2) / y_step * y_step;

		// Store the values in the corresponding v4l2_ctrl structures as well, so that they can be read from the outside
		priv->ctrl.ctrl_offset_x->cur.val = priv->ctrl.offset_x;
		priv->ctrl.ctrl_offset_y->cur.val = priv->ctrl.offset_y;

		dev_dbg(priv->dev, "%s: offset_x = %d, offset_y = %d", __func__, priv->ctrl.offset_x, priv->ctrl.offset_y);
	}
}
EXPORT_SYMBOL(tis_sensor_ctrl_do_offset_auto_center);

/*
 * General purpose Serializer Board controls:
 * - Select PicoBlade GP_OUT pin function [Sensor Strobe, Constant On, Constant Off]
 *   - MSER Type=FAKRA, Rev >= 1.10
 *   - Adapter Rev >= 1.00
 * - Select PicoBlade GP_OUT pin type [Open Drain, TTL]
 *   - MSER Type=FAKRA, Rev >= 1.10
 *   - Adapter Rev >= 1.00
 * - Select Trigger Source [FPDLink, PicoBlade]
 *   - MSER Type=FAKRA, Rev >= 1.10
 * - Select PicoBlade Trigger Polarity
 *   - MSER Type=FAKRA, Rev >= 1.10
 *   - Adapter Rev >= 1.00
 */