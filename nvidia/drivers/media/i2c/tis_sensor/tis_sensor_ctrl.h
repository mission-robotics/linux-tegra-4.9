
#ifndef _TIS_SENSOR_CTRL_H_
#define _TIS_SENSOR_CTRL_H_

#include "tis_sensor.h"

struct tis_sensor_ctrl_config
{
    const struct tis_sensor_ctrl_ops* ops;

	bool has_picoblade_connector;
	bool has_fpdlink;

	int trigger_source_default;
	int trigger_source_skip_mask;

	int trigger_in_polarity_default;
	int trigger_in_polarity_skip_mask;

	int gpout_mode_default;
	int gpout_mode_skip_mask;

	int gpout_function_default;
	int gpout_function_skip_mask;

	int offset_x_max;
	int offset_x_step;
	int offset_y_max;
	int offset_y_step;

	int black_level_min;
	int black_level_max;
	int black_level_default;

	int trigger_exposure_mode_default;
	int trigger_exposure_mode_skip_mask;

	int strobe_polarity_default;
	int strobe_polarity_skip_mask;

	int strobe_mode_default;
	int strobe_mode_skip_mask;

	int strobe_delay_min;
	int strobe_delay_max;
	int strobe_delay_default;

	int strobe_duration_min;
	int strobe_duration_max;
	int strobe_duration_default;

	int ext_sync_default;
	int ext_sync_skip_mask;

	int gain_mode_default;
	int gain_mode_skip_mask;
};

int tis_sensor_ctrl_create_all(struct tis_sensor *priv, struct v4l2_ctrl_handler *handler);
int tis_sensor_ctrl_update_controls(struct tis_sensor *priv, struct tis_sensor_ctrl_config *config);

int tis_sensor_ctrl_init(struct tis_sensor* priv);
int tis_sensor_ctrl_do_lazy_init(struct tis_sensor* priv);

struct v4l2_ctrl *tis_sensor_ctrl_add_bool(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, bool default_value, const struct v4l2_ctrl_ops* ops );
struct v4l2_ctrl *tis_sensor_ctrl_add_int(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, int min_value, int max_value, int step, int default_value, const struct v4l2_ctrl_ops* ops );
struct v4l2_ctrl *tis_sensor_ctrl_add_menu(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const char* const *items, int num_items, int default_index, int skip_mask, const struct v4l2_ctrl_ops* ops );
struct v4l2_ctrl *tis_sensor_ctrl_add_string_const(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const char* value);
struct v4l2_ctrl *tis_sensor_ctrl_add_u32_1d_const(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const u32* data, int count);
struct v4l2_ctrl *tis_sensor_ctrl_add_button(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const struct v4l2_ctrl_ops* ops );
struct v4l2_ctrl *tis_sensor_ctrl_add_bool_volatile_readonly(struct tis_sensor* priv, struct v4l2_ctrl_handler *handler, int cid, const char* name, const struct v4l2_ctrl_ops* ops);
void tis_sensor_ctrl_enable(struct v4l2_ctrl *ctrl);
void tis_sensor_ctrl_disable(struct v4l2_ctrl *ctrl);

void tis_sensor_ctrl_do_offset_auto_center(struct tis_sensor *priv);


#define TIS_SENSOR_DRIVER_MODULE(driver_name) \
	static int __init driver_name##_init(void) \
	{ \
		return i2c_add_driver(&driver_name); \
	} \
	static void __exit driver_name##_exit(void) \
	{ \
		i2c_del_driver(&driver_name); \
	} \
	module_init(driver_name##_init); \
	module_exit(driver_name##_exit);

#endif // _TIS_SENSOR_CTRL_H_
