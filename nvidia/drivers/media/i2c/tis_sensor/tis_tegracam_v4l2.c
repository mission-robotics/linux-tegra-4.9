
/*
 * Copied from drivers/media/platform/tegra/camera/tegracam_v4l2.c in nvidia's kernel sources
 *
 * tegracam_v4l2subdev_register is split into
 *  begin_tegracam_v4l2subdev_register
 *  end_tegracam_v4l2subdev_register
 * 
 * This allows us to add controls into the v4l2 control handler
 */

#include <linux/kernel.h>
#include <media/camera_common.h>

static int v4l2sd_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct camera_common_sensor_ops *sensor_ops = s_data->ops;
	struct tegracam_device *tc_dev = to_tegracam_device(s_data);
	struct tegracam_sensor_data *sensor_data = &s_data->tegracam_ctrl_hdl->sensor_data;
	struct sensor_blob *ctrl_blob = &sensor_data->ctrls_blob;
	struct sensor_blob *mode_blob = &sensor_data->mode_blob;
	int err = 0;

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	/* reset control packet at start/stop streaming */
	memset(ctrl_blob, 0, sizeof(struct sensor_blob));
	memset(mode_blob, 0, sizeof(struct sensor_blob));
	if (enable) {
		/* increase ref count so module can't be unloaded while streaming */
		if (!try_module_get(s_data->owner))
			return -ENODEV;

		err = sensor_ops->set_mode(tc_dev);
		if (err) {
			dev_err(&client->dev, "Error writing mode\n");
			goto error;
		}

		/* update control ranges based on mode settings*/
		err = tegracam_init_ctrl_ranges_by_mode(
			s_data->tegracam_ctrl_hdl, (u32) s_data->mode);
		if (err) {
			dev_err(&client->dev, "Error updating control ranges\n");
			goto error;
		}

#if 0
		if (s_data->override_enable) {
			err = tegracam_ctrl_set_overrides(
					s_data->tegracam_ctrl_hdl);
			if (err) {
				dev_err(&client->dev,
					"overrides cannot be set\n");
				goto error;
			}
		}
#endif

		err = sensor_ops->start_streaming(tc_dev);
		if (err) {
			dev_err(&client->dev, "Error turning on streaming\n");
			goto error;
		}

		/* add done command for blobs */
#if 0
		prepare_done_cmd(mode_blob);
		prepare_done_cmd(ctrl_blob);
#endif
		tc_dev->is_streaming = true;
	} else {
		err = sensor_ops->stop_streaming(tc_dev);
		if (err) {
			dev_err(&client->dev, "Error turning off streaming\n");
			goto error;
		}

		/* add done command for blob */
#if 0
		prepare_done_cmd(ctrl_blob);
#endif
		tc_dev->is_streaming = false;

		module_put(s_data->owner);
	}

	return 0;

error:
	module_put(s_data->owner);
	return err;
}

static int v4l2sd_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct camera_common_power_rail *pw = s_data->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops v4l2sd_video_ops = {
	.s_stream	= v4l2sd_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = v4l2sd_g_input_status,
};

static struct v4l2_subdev_core_ops v4l2sd_core_ops = {
	.s_power	= camera_common_s_power,
};

static int v4l2sd_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int v4l2sd_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else {
		ret = camera_common_s_fmt(sd, &format->format);

		if (ret == 0) {
			/* update control ranges based on mode settings*/
			ret = tegracam_init_ctrl_ranges_by_mode(
				s_data->tegracam_ctrl_hdl, (u32) s_data->mode);
			if (ret) {
				dev_err(&client->dev, "Error updating control ranges %d\n", ret);
				return ret;
			}
		}
	}

	/* TODO: Add set mode for blob collection */

	return ret;
}

static struct v4l2_subdev_pad_ops v4l2sd_pad_ops = {
	.set_fmt = v4l2sd_set_fmt,
	.get_fmt = v4l2sd_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops v4l2sd_ops = {
	.core	= &v4l2sd_core_ops,
	.video	= &v4l2sd_video_ops,
	.pad = &v4l2sd_pad_ops,
};

static const struct media_entity_operations media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


int begin_tegracam_v4l2subdev_register(struct tegracam_device *tc_dev,
				bool is_sensor)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct tegracam_ctrl_handler *ctrl_hdl = s_data->tegracam_ctrl_hdl;
	struct v4l2_subdev *sd = NULL;
	struct device *dev = tc_dev->dev;
	int err = 0;

	/* init v4l2 subdevice for registration */
	sd = &s_data->subdev;
	if (!sd || !tc_dev->client) {
		dev_err(dev, "Invalid subdev context\n");
		return -ENODEV;
	}

	v4l2_i2c_subdev_init(sd, tc_dev->client, &v4l2sd_ops);

	ctrl_hdl->ctrl_ops = tc_dev->tcctrl_ops;
	err = tegracam_ctrl_handler_init(ctrl_hdl);
	if (err) {
		dev_err(dev, "Failed to init ctrls %s\n", tc_dev->name);
		return err;
	}
	if (ctrl_hdl->ctrl_ops != NULL)
		tc_dev->numctrls = ctrl_hdl->ctrl_ops->numctrls;
	else
		tc_dev->numctrls = 0;
	s_data->numctrls = tc_dev->numctrls;	

	sd->ctrl_handler = s_data->ctrl_handler = &ctrl_hdl->ctrl_handler;
	s_data->ctrls = ctrl_hdl->ctrls;
	sd->internal_ops = tc_dev->v4l2sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_EVENTS;
	s_data->owner = sd->owner;
	/* Set owner to NULL so we can unload the driver module */
	sd->owner = NULL;

#if defined(CONFIG_MEDIA_CONTROLLER)
	tc_dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.ops = &media_ops;
	err = tegra_media_entity_init(&sd->entity,
			1, &tc_dev->pad, true, is_sensor);
	if (err < 0) {
		dev_err(dev, "unable to init media entity\n");
		return err;
	}
#endif

	return err;
}

int end_tegracam_v4l2subdev_register(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct v4l2_subdev *sd = &s_data->subdev;

	return v4l2_async_register_subdev(sd);	
}

