/*
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

#ifndef __TIS_SENSOR_H__
#define __TIS_SENSOR_H__

#include <linux/ioctl.h>	/* For IOCTL macros */
#include <linux/debugfs.h>
#include <media/camera_common.h>
// #include <media/nvc.h>
// #include <media/nvc_image.h>

#include "tis_sensor_parts.h"
#include "../include/tis_sensor_ctrl_ids.h"

#define MAX_MODES 16

enum {
	TIS_SENSOR_UNDEFINED = 0,
	TIS_SENSOR_IMX290,
	TIS_SENSOR_IMX296,
	TIS_SENSOR_IMX297,
	TIS_SENSOR_IMX334,
	TIS_SENSOR_IMX335,
	TIS_SENSOR_IMX390,
	TIS_SENSOR_IMX397,
};

#define TIS_EEPROM_SERIALNUMBER_OFFSET		8
#define TIS_EEPROM_SERIALNUMBER_LENGTH		16
#define TIS_EEPROM_DISPLAYNAME_OFFSET		24
#define TIS_EEPROM_DISPLAYNAME_LENGTH		32
#define TIS_EEPROM_MANUFACTURERAME_OFFSET	56
#define TIS_EEPROM_MANUFACTURERNAME_LENGTH	32
#define TIS_EEPROM_SENSORBOARDPART_OFFSET	88
#define TIS_EEPROM_SENSORBOARDPART_LENGTH	64
#define TIS_EEPROM_ADAPTERBOARDPART_OFFSET	152
#define TIS_EEPROM_ADAPTERBOARDPART_LENGTH	64

struct tis_sensor;
struct tis_sensor_ctrl_config;

struct tis_sensor_eeprom_info
{
	char serial_number[TIS_EEPROM_SERIALNUMBER_LENGTH + 1];

	char display_name[TIS_EEPROM_DISPLAYNAME_LENGTH + 1];
	char manufacturer_name[TIS_EEPROM_MANUFACTURERNAME_LENGTH + 1];

	char sensor_board_part[TIS_EEPROM_SENSORBOARDPART_LENGTH + 1];
	char adapter_board_part[TIS_EEPROM_ADAPTERBOARDPART_LENGTH + 1];
};

struct tis_sensor_ops
{
	// Perform the sensor's power-up sequence
	int (*sensor_power_on)(struct tis_sensor *priv);

	// This hand the sensor a tis_sensor_ctrl_config structure to fill in
	// - ops pointer for common controls
	// - limits, default values, skip_masks for common controls
	// This function is called after the hardware configuration is known
	int (*sensor_update_common_ctrl_config)(struct tis_sensor *priv, struct tis_sensor_ctrl_config *config);

	// This callback allows the sensor to register additional sensor-specific controls
	int (*sensor_register_private_controls)(struct tis_sensor *priv, struct v4l2_ctrl_handler *handler);	

	// In this callback, the registered controls can be configured
	// This function is called after the hardware configuration is known
	int (*sensor_update_private_controls)(struct tis_sensor *priv);
};

struct tis_sensor_ctrl_ops
{
    int (*sensor_apply_trigger)(struct tis_sensor* priv);
    int (*sensor_apply_strobe)(struct tis_sensor* priv);
	int (*sensor_do_software_trigger)(struct tis_sensor* priv);
	int (*sensor_apply_trigger_source)(struct tis_sensor* priv);
	int (*sensor_apply_trigger_in_polarity)(struct tis_sensor* priv);
	int (*sensor_apply_gpout_mode)(struct tis_sensor* priv);
	int (*sensor_apply_gpout_function)(struct tis_sensor* priv);
	int (*sensor_apply_offsets)(struct tis_sensor* priv);
	int (*sensor_apply_black_level)(struct tis_sensor* priv);
	int (*sensor_apply_trigger_exposure_mode)(struct tis_sensor* priv);
	int (*sensor_apply_strobe_polarity)(struct tis_sensor *priv);
	int (*sensor_apply_strobe_delay)(struct tis_sensor *priv);
	int (*sensor_apply_strobe_duration)(struct tis_sensor *priv);
	int (*sensor_apply_ext_sync)(struct tis_sensor *priv);
	int (*sensor_apply_gain_mode)(struct tis_sensor *priv);
};

struct tis_sensor_ctrl
{
	struct mutex				lazy_init_lock;
	int							(*lazy_init)(struct tis_sensor *priv);

	const struct tis_sensor_ctrl_ops*	ops;

	// References to v4l2_ctrl from tegra framework
	struct v4l2_ctrl			*ctrl_exposure;
	struct v4l2_ctrl			*ctrl_gain;

	// Keep references to some of the v4l2_ctrl structures, so that we can update their values
	struct v4l2_ctrl			*ctrl_device_valid;
	struct v4l2_ctrl			*ctrl_device_serial_number;
	struct v4l2_ctrl			*ctrl_device_model_name;
	struct v4l2_ctrl			*ctrl_device_vendor_name;
	struct v4l2_ctrl			*ctrl_mode_raw_fourcc;
	struct v4l2_ctrl			*ctrl_trigger_mode;
	struct v4l2_ctrl			*ctrl_strobe_mode;
	struct v4l2_ctrl			*ctrl_strobe_delay;
	struct v4l2_ctrl			*ctrl_strobe_duration;
	struct v4l2_ctrl			*ctrl_trigger_software;
	struct v4l2_ctrl			*ctrl_exposure_mode;
	struct v4l2_ctrl			*ctrl_strobe_polarity;
	struct v4l2_ctrl			*ctrl_trigger_source;
	struct v4l2_ctrl			*ctrl_trigger_in_polarity;
	struct v4l2_ctrl			*ctrl_gpout_mode;
	struct v4l2_ctrl			*ctrl_gpout_function;
	struct v4l2_ctrl			*ctrl_offset_x;
	struct v4l2_ctrl			*ctrl_offset_y;
	struct v4l2_ctrl			*ctrl_offset_auto_center;
	struct v4l2_ctrl			*ctrl_black_level;
	struct v4l2_ctrl			*ctrl_ext_sync;
	struct v4l2_ctrl			*ctrl_gain_mode;

	// These are the current values of the controls generated by tis_sensor_ctrl (read-only)
	int							trigger_mode;
	int							strobe_mode;
	int							trigger_source;
	int							trigger_in_polarity;
	int							gpout_mode;
	int							gpout_function;
	int							offset_x;
	int							offset_y;
	bool						offset_auto_center;
	int							black_level;
	int							trigger_exposure_mode;
	int							strobe_polarity;
	int							strobe_delay;
	int							strobe_duration;
	int							ext_sync;
	int							gain_mode;
};

struct tis_sensor_connection;

struct tis_sensor {
	struct camera_common_power_rail	power;
	struct i2c_client			   *i2c_client;
	struct device				*dev;
	struct device_node			*device_node;
	
	struct tegracam_device		   *tc_dev;

	u8							extsync_mode; // IMX335, 0 = disabled, 1 = master, 2 = extsync

	struct tis_sensor_ctrl		ctrl;

	u16							debug_addr;
	struct dentry*				debug_d;

	void 						*mode_table;
	struct camera_common_frmfmt frmfmt[MAX_MODES];
	u32							raw_fourcc[MAX_MODES];
	int 						sensor_model;

	u64                         fps;
	u64                         exposure_time;
	u32							gain;

	// Maximum size of the sensor
	// Mostly useful for ROI positioning purposes
	u32							SENSOR_WIDTH;
	u32							SENSOR_HEIGHT;

	// Size of the currently active mode (only valid after set_mode has been called once)
	u32							mode_active_w;
	u32							mode_active_h;
	u32							mode_lanes;

	u32							pixel_clock;

	struct gpio_desc*			gpio_cam_power;
	struct gpio_desc*			gpio_sensor_reset;
	struct gpio_desc*			gpio_gpout_level;
	struct gpio_desc*			gpio_gpout_pushpull;
	struct gpio_desc*			gpio_gpout_select;
	struct gpio_desc*			gpio_trig_lvl_sel;
	struct gpio_desc*			gpio_trig_src_sel;

	struct tis_sensor_eeprom_info	eeprom_info;
	struct tis_sensor_board_part	sensor_board_part;
	struct tis_adapter_board_part	adapter_board_part;

	struct camera_common_data	*s_data;

	void						*priv;

	struct tis_sensor_connection *connection;
	const struct tis_sensor_ops	*sensor_ops;
};

int tis_imx_read_8(struct camera_common_data *s_data, u16 addr, u8 *val);
int tis_imx_read_16(struct camera_common_data *s_data, u16 addr, u16 *val);
int tis_imx_read_24(struct camera_common_data *s_data, u16 addr, u32 *val);
int tis_imx_write_8(struct camera_common_data *s_data, u16 addr, u8 val);
int tis_imx_write_16(struct camera_common_data *s_data, u16 addr, u16 val);
int tis_imx_write_24(struct camera_common_data *s_data, u16 addr, u32 val);
int tis_imx_do_power_on(struct tis_sensor *priv);
int tis_ar_do_power_on(struct tis_sensor *priv);
int tis_imx_do_power_off(struct tis_sensor *priv);
int tis_imx_fake_power_on(struct camera_common_data *s_data);
int tis_imx_fake_power_off(struct camera_common_data *s_data);
int tis_imx_power_put(struct tegracam_device *tc_dev);
int tis_imx_power_get(struct tegracam_device *tc_dev);
int tis_imx_get_mode_from_dt(struct tegracam_device *tc_dev, int mode, u32 *active_w, u32 *active_h, u32 *mode_lanes, u32 *mode_lane_mbps, u32 *mode_bpp, u32 *mode_mclk_khz);
bool tis_sensor_is_on_fpdlink(struct tegracam_device *tc_dev);
void tis_sensor_init_defaults(struct tis_sensor *priv, const struct sensor_mode_properties *mode);
int tis_power_eeprom(struct device* dev, bool onoff);
bool tis_sensor_is_power_on(struct tis_sensor* priv);

void tis_sensor_gpios_put(struct tis_sensor *priv);

int tis_sensor_fill_eeprom_info(struct tis_sensor* priv);

int tis_ar_read(struct camera_common_data *s_data, u16 addr, u16 *val);
int tis_ar_write(struct camera_common_data *s_data, u16 addr, u16 val);
int
tis_ar_write_table_16_as_8(struct regmap *regmap,
				const struct reg_16 table[],
				const struct reg_16 override_list[],
				int num_override_regs,
				u16 wait_ms_addr, u16 end_addr);
int
tis_ar_write_table_16(struct regmap *regmap,
				const struct reg_16 table[],
				const struct reg_16 override_list[],
				int num_override_regs,
				u16 wait_ms_addr, u16 end_addr);

int tis_sensor_apply_trigger_source(struct tis_sensor *priv);
int tis_sensor_apply_trigger_in_polarity(struct tis_sensor *priv);
int tis_sensor_apply_gpout_mode(struct tis_sensor *priv);
int tis_sensor_apply_gpout_function(struct tis_sensor *priv);

bool tis_sensor_is_sensor_model(struct device *dev, struct device_node *node, const char *sensor_model);

struct tis_sensor *tis_sensor_create(struct i2c_client *client, u32 sensor_width, u32 sensor_height, int sensor_model, size_t alloc_sensor_priv_size, const struct tis_sensor_ops *sensor_ops);
struct camera_common_sensor_ops *tis_sensor_tegracam_build_sensor_ops(struct tis_sensor *priv, struct camera_common_sensor_ops *template);
int tis_sensor_tegracam_device_register(struct tis_sensor *priv, const struct regmap_config *sensor_regmap_config, const struct tegracam_ctrl_ops *tcctrl_ops, struct camera_common_sensor_ops *sensor_ops, const char *sensor_driver_name);
void tis_sensor_remove(struct tis_sensor *priv);

void tis_sensor_register_debug_imx(struct tis_sensor *priv);
void tis_sensor_register_debug_ar(struct tis_sensor *priv);

int tis_sensor_update_raw_fourcc(struct tis_sensor *priv);

bool tis_sensor_is_device_valid(struct tis_sensor *priv);
int tis_sensor_configure_connection(struct tis_sensor *priv, int num_csi_lanes, u32 clkout_hz, u32 *actual_clkout_hz);

#endif  /* __TIS_SENSOR_H__ */
