#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>

#include "tis_sensor.h"
#include "tis_sensor_regmap.h"
#include "tis_sensor_ctrl.h"
#include "tis_tegracam_v4l2.h"
#include "tis_sensor_connection.h"

int
tis_ar_write_table_16_as_8(struct regmap *regmap,
				const struct reg_16 table[],
				const struct reg_16 override_list[],
				int num_override_regs,
				u16 wait_ms_addr, u16 end_addr)
{
	int err;
	const struct reg_16 *next;
	int i;
	u16 val;

	int range_start = -1;
	int range_count = 0;
	u8 range_vals[256];
	int max_range_vals = ARRAY_SIZE(range_vals);

	for (next = table;; next++) {
		/* If we have a range open and */
		/* either the address doesn't match */
		/* or the temporary storage is full, flush*/
		if  ((next->addr != range_start + range_count) ||
		     (next->addr == end_addr) ||
		     (next->addr == wait_ms_addr) ||
		     (range_count == max_range_vals)) {

			if (range_count > 1) {
				err =
				    regmap_bulk_write(regmap, range_start,
						      &range_vals[0],
						      range_count);
			}

			if (err) {
				pr_err("%s:regmap_util_write_table:%d",
				       __func__, err);
				return err;
			}

			range_start = -1;
			range_count = 0;

			/* Handle special address values */
			if (next->addr == end_addr)
				break;

			if (next->addr == wait_ms_addr) {
				msleep_range(next->val);
				continue;
			}
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (range_start == -1)
			range_start = next->addr;

		range_vals[range_count++] = (u8) (val >> 8);
		range_vals[range_count++] = (u8) (val & 0xFF);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tis_ar_write_table_16_as_8);

int
tis_ar_write_table_16(struct regmap *regmap,
				const struct reg_16 table[],
				const struct reg_16 override_list[],
				int num_override_regs,
				u16 wait_ms_addr, u16 end_addr)
{
	int err;
	const struct reg_16 *next;
	int i;
	u16 val;

	int range_start = -1;
	int range_count = 0;
	u16 range_vals[256];
	int max_range_vals = ARRAY_SIZE(range_vals);

	for (next = table;; next++) {
		/* If we have a range open and */
		/* either the address doesn't match */
		/* or the temporary storage is full, flush*/
		if  ((next->addr != range_start + range_count) ||
		     (next->addr == end_addr) ||
		     (next->addr == wait_ms_addr) ||
		     (range_count == max_range_vals)) {

			if (range_count > 1) {
				err =
				    regmap_bulk_write(regmap, range_start,
						      range_vals,
						      range_count);
			}

			if (err) {
				pr_err("%s:regmap_util_write_table:%d",
				       __func__, err);
				return err;
			}

			range_start = -1;
			range_count = 0;

			/* Handle special address values */
			if (next->addr == end_addr)
				break;

			if (next->addr == wait_ms_addr) {
				msleep_range(next->val);
				continue;
			}
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (range_start == -1)
			range_start = next->addr;

		range_vals[range_count++] = val;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tis_ar_write_table_16);


int tis_ar_read(struct camera_common_data *s_data, u16 addr, u16 *val)
{
	return tis_sensor_regmap_read16_be(s_data->regmap, addr, val);
}
EXPORT_SYMBOL(tis_ar_read);

int tis_ar_write(struct camera_common_data *s_data, u16 addr, u16 val)
{
	int err = tis_sensor_regmap_write16_be(s_data->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}
EXPORT_SYMBOL(tis_ar_write);


int tis_imx_read_8(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	return tis_sensor_regmap_read8(s_data->regmap, addr, val);
}
EXPORT_SYMBOL(tis_imx_read_8);

int tis_imx_read_16(struct camera_common_data *s_data, u16 addr, u16 *val)
{
	return tis_sensor_regmap_read16_le(s_data->regmap, addr, val);
}
EXPORT_SYMBOL(tis_imx_read_16);

int tis_imx_read_24(struct camera_common_data *s_data, u16 addr, u32 *val)
{
	return tis_sensor_regmap_read24_le(s_data->regmap, addr, val);
}
EXPORT_SYMBOL(tis_imx_read_24);

int tis_imx_write_8(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;

	dev_dbg(s_data->dev, "write8: 0x%x = 0x%x", addr, val);

	err = tis_sensor_regmap_write8(s_data->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}
EXPORT_SYMBOL(tis_imx_write_8);

int tis_imx_write_16(struct camera_common_data *s_data, u16 addr, u16 val)
{
	int err;

	dev_dbg(s_data->dev, "write16: 0x%x = 0x%x", addr, val);

	err = tis_sensor_regmap_write16_le(s_data->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}
EXPORT_SYMBOL(tis_imx_write_16);

int tis_imx_write_24(struct camera_common_data *s_data, u16 addr, u32 val)
{
	int err;

	dev_dbg(s_data->dev, "write24: 0x%x = 0x%x", addr, val);

	err = tis_sensor_regmap_write24_le(s_data->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}
EXPORT_SYMBOL(tis_imx_write_24);

int tis_ar_do_power_on(struct tis_sensor *priv)
{
	struct device *dev = priv->dev;

	if( !priv->gpio_sensor_reset )
	{
		dev_err(dev, "%s: gpio_sensor_reset is not initialized", __func__);
		return -EINVAL;
	}
	if( !priv->gpio_cam_power )
	{
		dev_err(dev, "%s: gpio_cam_power is not initialized", __func__);
		return -EINVAL;
	}

	gpiod_set_value_cansleep(priv->gpio_sensor_reset, 1);
	gpiod_set_value_cansleep(priv->gpio_cam_power, 0);

	usleep_range(200, 400);

	gpiod_set_value_cansleep(priv->gpio_cam_power, 1);

	usleep_range(30000, 35000); // Xtal Settle time >=30 ms

	gpiod_set_value_cansleep(priv->gpio_sensor_reset, 0);

	usleep_range(1000, 1500); // Hard-Reset >=1ms

	gpiod_set_value_cansleep(priv->gpio_sensor_reset, 1);

	usleep_range(1000, 1500);

	return 0;
}
EXPORT_SYMBOL(tis_ar_do_power_on);

int tis_imx_do_power_on(struct tis_sensor *priv)
{
	struct device *dev = priv->dev;

	if( !priv->gpio_sensor_reset )
	{
		dev_err(dev, "%s: gpio_sensor_reset is not initialized", __func__);
		return -EINVAL;
	}
	if( !priv->gpio_cam_power )
	{
		dev_err(dev, "%s: gpio_cam_power is not initialized", __func__);
		return -EINVAL;
	}

	gpiod_set_value_cansleep(priv->gpio_sensor_reset, 0);
	gpiod_set_value_cansleep(priv->gpio_cam_power, 0);

	usleep_range(20, 40);

	gpiod_set_value_cansleep(priv->gpio_cam_power, 1);

	usleep_range(10, 20);

	gpiod_set_value_cansleep(priv->gpio_sensor_reset, 1);

	usleep_range(20000, 21000);

	return 0;
}
EXPORT_SYMBOL(tis_imx_do_power_on);

int tis_imx_do_power_off(struct tis_sensor *priv)
{
	if (priv->gpio_sensor_reset)
		gpiod_set_value_cansleep(priv->gpio_sensor_reset, 0);
	if (priv->gpio_cam_power)
		gpiod_set_value_cansleep(priv->gpio_cam_power, 0);

	usleep_range(1, 2);

	return 0;
}
EXPORT_SYMBOL(tis_imx_do_power_off);

int tis_imx_fake_power_on(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct device *dev = s_data->dev;
	struct tegracam_device *tc_dev = to_tegracam_device(s_data);
	struct tis_sensor *priv = tegracam_get_privdata(tc_dev);	
	int cam_power_gpio = desc_to_gpio(priv->gpio_cam_power);
	bool is_power_on = false;

	int err = tis_sensor_connection_get_gpio_out_status(priv->connection, cam_power_gpio, &is_power_on);
	if( !err && !is_power_on )
	{
		dev_warn(dev, "Power is not on, running sensor power-up sequence");

		priv->sensor_ops->sensor_power_on(priv);
	}

	pw->state = SWITCH_ON;
	return 0;
}
EXPORT_SYMBOL(tis_imx_fake_power_on);

int tis_imx_fake_power_off(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	pw->state = SWITCH_OFF;
	return 0;
}
EXPORT_SYMBOL(tis_imx_fake_power_off);

int tis_imx_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->iovdd = NULL;

	return 0;
}
EXPORT_SYMBOL(tis_imx_power_put);

int tis_imx_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;

	/* Sensor MCLK (aka. INCK) */
	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}


	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->af_gpio = pdata->af_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

done:
	pw->state = SWITCH_OFF;
	return err;
}
EXPORT_SYMBOL(tis_imx_power_get);

int tis_imx_get_mode_from_dt(struct tegracam_device *tc_dev, int mode, u32 *active_w, u32 *active_h, u32 *mode_lanes, u32 *mode_lane_mbps, u32 *mode_bpp, u32 *mode_mclk_khz)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
    struct device_node *child;
    char nodename[16];
    int ret = -1;

    sprintf(nodename, "mode%d", mode);

    of_node_get(node);
    child = of_find_node_by_name(node, nodename);
    if (child != NULL)
    {
    	const char *outs;
    	ret = of_property_read_string(child, "active_w", &outs);
    	if (ret == 0){
			sscanf(outs, "%d", active_w);
    		ret = of_property_read_string(child, "active_h", &outs);
    		if (ret == 0){
				sscanf(outs, "%d", active_h);
    		}
    	}

		if( mode_lanes )
		{
			ret = of_property_read_u32(child, "mode_lanes", mode_lanes);
		}

		if( mode_lane_mbps )
		{
			ret = of_property_read_u32(child, "mode_lane_mbps", mode_lane_mbps);
		}

		if( mode_bpp )
		{
			ret = of_property_read_string(child, "csi_pixel_bit_depth", &outs);
			if( ret == 0 )
			{
				sscanf(outs, "%d", mode_bpp);
			}
		}

		if( mode_mclk_khz )
		{			
			ret = of_property_read_string(child, "mclk_khz", &outs);
			if( ret == 0 )
			{
				sscanf(outs, "%d", mode_mclk_khz);
			}
		}

    	of_node_put(child);
    }
    return ret;
}
EXPORT_SYMBOL(tis_imx_get_mode_from_dt);

int tis_sensor_apply_trigger_source(struct tis_sensor *priv)
{
	struct device *dev = priv->tc_dev->dev;

	dev_info(dev, "%s: trigger_source = %d", __func__, priv->ctrl.trigger_source);

	switch( priv->ctrl.trigger_source )
	{
		case TIS_SENSOR_CTRL_TRIGGER_SOURCE_FPDLINK:
			gpiod_set_value_cansleep( priv->gpio_trig_src_sel, 0 );
			break;
		case TIS_SENSOR_CTRL_TRIGGER_SOURCE_PICOBLADE:
			gpiod_set_value_cansleep( priv->gpio_trig_src_sel, 1 );
			break;
		default:
			dev_err(dev, "%s: Unexpected trigger_source: %d", __func__, priv->ctrl.trigger_source);
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_apply_trigger_source);

int tis_sensor_apply_trigger_in_polarity(struct tis_sensor *priv)
{
	struct device *dev = priv->tc_dev->dev;

	dev_info(dev, "%s: trigger_in_polarity = %d", __func__, priv->ctrl.trigger_in_polarity);

	switch( priv->ctrl.trigger_in_polarity )
	{
		case TIS_SENSOR_CTRL_TRIGGER_IN_POLARITY_DEFAULT:
			gpiod_set_value_cansleep( priv->gpio_trig_lvl_sel, 0 );
			break;
		case TIS_SENSOR_CTRL_TRIGGER_IN_POLARITY_INVERT:
			gpiod_set_value_cansleep( priv->gpio_trig_lvl_sel, 1 );
			break;
		default:
			dev_err(dev, "%s: Unexpected trigger_in_polarity: %d", __func__, priv->ctrl.trigger_in_polarity);
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_apply_trigger_in_polarity);

int tis_sensor_apply_gpout_mode(struct tis_sensor *priv)
{
	struct device *dev = priv->tc_dev->dev;

	dev_info(dev, "%s: gpout_mode = %d", __func__, priv->ctrl.gpout_mode);

	switch( priv->ctrl.gpout_mode )
	{
		case TIS_SENSOR_CTRL_GPOUT_MODE_OPENDRAIN:
			gpiod_set_value_cansleep( priv->gpio_gpout_pushpull, 0 );
			break;
		case TIS_SENSOR_CTRL_GPOUT_MODE_TTL:
			gpiod_set_value_cansleep( priv->gpio_gpout_pushpull, 1 );
			break;
		default:
			dev_err(dev, "%s: Unexpected gpout_mode: %d", __func__, priv->ctrl.gpout_mode);
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_apply_gpout_mode);

int tis_sensor_apply_gpout_function(struct tis_sensor *priv)
{
	struct device *dev = priv->tc_dev->dev;

	dev_info(dev, "%s: gpout_function = %d", __func__, priv->ctrl.gpout_function);

	switch( priv->ctrl.gpout_function )
	{
		case TIS_SENSOR_CTRL_GPOUT_FUNCTION_STROBE:
		case TIS_SENSOR_CTRL_GPOUT_FUNCTION_SYNC_OUT:
			gpiod_set_value_cansleep( priv->gpio_gpout_select, 0 );
			break;		
		case TIS_SENSOR_CTRL_GPOUT_FUNCTION_CONSTANTLOW:
			gpiod_set_value_cansleep( priv->gpio_gpout_select, 1 );
			gpiod_set_value_cansleep( priv->gpio_gpout_level, 0 );
			break;		
		case TIS_SENSOR_CTRL_GPOUT_FUNCTION_CONSTANTHIGH:
			gpiod_set_value_cansleep( priv->gpio_gpout_select, 1 );
			gpiod_set_value_cansleep( priv->gpio_gpout_level, 1 );
			break;
		default:
			dev_err(dev, "%s: Unexpected gpout_function: %d", __func__, priv->ctrl.gpout_function);
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_apply_gpout_function);

static int tis_sensor_gpiod_get(struct device* dev, struct gpio_desc **pdesc, const char* gpio_name )
{
	int err = 0;

	*pdesc = gpiod_get(dev, gpio_name, GPIOD_ASIS);

	if( IS_ERR(*pdesc) )
	{
		err = PTR_ERR(*pdesc);
		dev_err(dev, "%s: Failed to acquire GPIO %s (%d)", __func__, gpio_name, err );
		*pdesc = NULL;
	}	

	return err;
}

static void tis_sensor_gpios_get(struct tis_sensor *priv)
{
	struct device *dev = priv->dev;

	tis_sensor_gpiod_get(dev, &priv->gpio_cam_power, "pwdn" );
	tis_sensor_gpiod_get(dev, &priv->gpio_sensor_reset, "reset" );
	tis_sensor_gpiod_get(dev, &priv->gpio_gpout_level, "gpoutlevel" );
	tis_sensor_gpiod_get(dev, &priv->gpio_gpout_pushpull, "gpoutpushpull" );
	tis_sensor_gpiod_get(dev, &priv->gpio_gpout_select, "gpoutselect" );
	tis_sensor_gpiod_get(dev, &priv->gpio_trig_lvl_sel, "triglvlsel" );
	tis_sensor_gpiod_get(dev, &priv->gpio_trig_src_sel, "trigsrcsel" );
}

static void tis_sensor_gpiod_put(struct gpio_desc **pdesc)
{
	gpiod_put(*pdesc);
	*pdesc = NULL;
}

void tis_sensor_gpios_put(struct tis_sensor *priv)
{
	tis_sensor_gpiod_put(&priv->gpio_cam_power);
	tis_sensor_gpiod_put(&priv->gpio_sensor_reset);
	tis_sensor_gpiod_put(&priv->gpio_gpout_level);
	tis_sensor_gpiod_put(&priv->gpio_gpout_pushpull);
	tis_sensor_gpiod_put(&priv->gpio_gpout_select);
	tis_sensor_gpiod_put(&priv->gpio_trig_lvl_sel);
	tis_sensor_gpiod_put(&priv->gpio_trig_src_sel);
}
EXPORT_SYMBOL(tis_sensor_gpios_put);

bool tis_sensor_is_on_fpdlink(struct tegracam_device *tc_dev)
{
	const char *outs = 0;
	return !of_property_read_string(tc_dev->dev->of_node, "fpdlink", &outs);
}
EXPORT_SYMBOL(tis_sensor_is_on_fpdlink);

static int v4l2_ctrl_update_value_s64(struct v4l2_ctrl *ctrl, s64 val)
{
	if( ctrl == NULL )
	{
		pr_err("%s: ctrl == NULL", __func__);
		return -EINVAL;
	}

	if( ctrl->p_cur.p_s64 == NULL )
	{
		pr_err("%s: ctrl->p_cur.p_s64 == NULL", __func__);
		return -EINVAL;
	}

	*ctrl->p_cur.p_s64 = val;

	return 0;
}

void tis_sensor_init_defaults(struct tis_sensor *priv, const struct sensor_mode_properties *mode)
{
	if( !priv->fps ) // Init fps with valid value
	{
		priv->fps = mode->control_properties.default_framerate;
		dev_info(priv->tc_dev->dev, "%s: fps was 0, initialized to %lld", __func__, priv->fps);
	}
	if( !priv->exposure_time )
	{
		priv->exposure_time = mode->control_properties.default_exp_time.val;

		v4l2_ctrl_update_value_s64(priv->ctrl.ctrl_exposure, (s64)priv->exposure_time);

		dev_info(priv->tc_dev->dev, "%s: exposure_time was 0, initialized to %lld", __func__, priv->exposure_time);
	}
	if( !priv->gain )
	{
		priv->gain = mode->control_properties.default_gain;

		v4l2_ctrl_update_value_s64(priv->ctrl.ctrl_gain, (s64)priv->gain);

		dev_info(priv->tc_dev->dev, "%s: gain was 0, initialized to %d", __func__, priv->gain);
	}
}
EXPORT_SYMBOL(tis_sensor_init_defaults);

int tis_power_eeprom(struct device* dev, bool onoff)
{
	int err = 0;
	struct gpio_desc* cam_power = gpiod_get(dev, "pwdn", 0);
	if( !cam_power )
	{
		dev_err(dev, "%s: gpiod_get returned NULL", __func__);
		return -EIO;
	}

	if( onoff )
	{
		usleep_range(1100, 1200);
	}

	err = gpiod_direction_output(cam_power, onoff ? 1 : 0);
	if( err )
	{
		dev_err(dev, "%s: Failed to set cam_power (%d)", __func__, err);		
	}

	//gpiod_set_value_cansleep(cam_power, onoff ? 1 : 0);
	gpiod_put(cam_power);

	if( onoff && !err )
	{
		usleep_range(110, 120);
	}

	return err;
}
EXPORT_SYMBOL(tis_power_eeprom);


static int regmap_raw_read_retry(struct device *dev, struct regmap *map, unsigned int reg, void *val, size_t val_len)
{
	int retry = 10;

	while( true )
	{
		int ret = regmap_raw_read(map, reg, val, val_len);
		if( ret < 0 && retry-- > 0)
		{
			dev_err(dev, "%s: regmap_raw_read(0x%x) failed (%d), retries = %d\n", __func__, reg, ret, retry);
			usleep_range(1000, 1020);
		}
		else
		{
			return ret;
		}
	}
}


static const struct regmap_config tis_sensor_eeprom_regmap_config =
{
	.reg_bits	= 16,
	.val_bits	= 8,
	.cache_type	= REGCACHE_NONE,
	.use_single_rw	= true,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
};

int tis_sensor_fill_eeprom_info(struct tis_sensor* priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct device *dev = priv->dev;
	struct device_node *node = dev->of_node;
	u32 addr;
	int err;
	struct i2c_client *tis_eeprom;
	struct regmap *tis_eeprom_regmap;

	err = of_property_read_u32(node, "reg-tis_eeprom", &addr );
	if( err || !addr )
	{
		dev_err(dev, "%s: reg-tis_eeprom not available: err = %d addr = %d", __func__, err, addr);
		return err;
	}

	memset(&priv->eeprom_info, 0, sizeof(priv->eeprom_info));

	tis_eeprom = i2c_new_dummy( client->adapter, addr );
	if( !tis_eeprom )
	{
		dev_err(dev, "%s: i2c_new_dummy returned null", __func__);
		return -EIO;
	}

	tis_eeprom_regmap = devm_regmap_init_i2c(tis_eeprom, &tis_sensor_eeprom_regmap_config);
	if( IS_ERR(tis_eeprom_regmap) )
	{
		err = PTR_ERR(tis_eeprom_regmap);
		dev_err(dev, "%s: devm_regmap_init_i2c failed (%d)", __func__, err);
		return err;
	}

	err = regmap_raw_read_retry(dev, tis_eeprom_regmap, TIS_EEPROM_SERIALNUMBER_OFFSET, priv->eeprom_info.serial_number, TIS_EEPROM_SERIALNUMBER_LENGTH);
	if( err )
	{
		dev_err(dev, "%s: Failed to read EEPROM (%d)", __func__, err);

		i2c_unregister_device(tis_eeprom);
		
		return err;
	}

	regmap_raw_read_retry(dev, tis_eeprom_regmap, TIS_EEPROM_DISPLAYNAME_OFFSET, priv->eeprom_info.display_name, TIS_EEPROM_DISPLAYNAME_LENGTH);
	regmap_raw_read_retry(dev, tis_eeprom_regmap, TIS_EEPROM_MANUFACTURERAME_OFFSET, priv->eeprom_info.manufacturer_name, TIS_EEPROM_MANUFACTURERNAME_LENGTH);
	regmap_raw_read_retry(dev, tis_eeprom_regmap, TIS_EEPROM_SENSORBOARDPART_OFFSET, priv->eeprom_info.sensor_board_part, TIS_EEPROM_SENSORBOARDPART_LENGTH);
	regmap_raw_read_retry(dev, tis_eeprom_regmap, TIS_EEPROM_ADAPTERBOARDPART_OFFSET, priv->eeprom_info.adapter_board_part, TIS_EEPROM_ADAPTERBOARDPART_LENGTH);

	i2c_unregister_device(tis_eeprom);

	if( priv->eeprom_info.serial_number[0] == 0xFF || priv->eeprom_info.serial_number[0] == 0x00 )
	{
		static int serial = 20129900;
		char buf[10];
		sprintf( buf, "%d", serial );
		serial += 1;

		strncpy(priv->eeprom_info.serial_number, buf, sizeof(priv->eeprom_info.serial_number));
		strncpy(priv->eeprom_info.display_name, "Unknown Device", sizeof(priv->eeprom_info.display_name));
		strncpy(priv->eeprom_info.manufacturer_name, "The Imaging Source", sizeof(priv->eeprom_info.manufacturer_name));
	}

	dev_info(dev, "%s: tis ee serial: %s", __func__, priv->eeprom_info.serial_number);
	dev_info(dev, "%s: tis ee display_name: %s", __func__, priv->eeprom_info.display_name);
	dev_info(dev, "%s: tis ee manufacturer_name: %s", __func__, priv->eeprom_info.manufacturer_name);
	dev_info(dev, "%s: tis ee sensor_board_part: %s", __func__, priv->eeprom_info.sensor_board_part);
	dev_info(dev, "%s: tis ee adapter_board_part: %s", __func__, priv->eeprom_info.adapter_board_part);

	tis_sensor_parse_sensor_board_part(priv->eeprom_info.sensor_board_part, &priv->sensor_board_part);
	tis_sensor_parse_adapter_board_part(priv->eeprom_info.adapter_board_part, &priv->adapter_board_part);

	dev_info(dev, "sensor part: %s", priv->sensor_board_part.description);
	dev_info(dev, "adapter part: %s", priv->adapter_board_part.description);
	dev_info(dev, "sensor_model = %d, sensor_type = %d", priv->sensor_board_part.sensor_model, priv->sensor_board_part.sensor_type);

	dev_info(dev, "adapter has picoblade: %d", priv->adapter_board_part.caps->has_picoblade_connector);
	dev_info(dev, "adapter has fpdlink: %d", priv->adapter_board_part.caps->has_fpdlink);

	return 0;
}
EXPORT_SYMBOL(tis_sensor_fill_eeprom_info);

bool tis_sensor_is_power_on(struct tis_sensor* priv)
{
	struct camera_common_data *s_data = priv->s_data;

	if( !s_data )
	{
		// If s_data is not initialized, the tegracam* functions have not been executed yet.
		// Power cannot be on yet.
		return false;
	}

	return true;
}
EXPORT_SYMBOL(tis_sensor_is_power_on);

bool tis_sensor_is_sensor_model(struct device *dev, struct device_node *node, const char *sensor_model)
{
	const char *outs;

	int err = of_property_read_string(node, "sensor_model", &outs);
	if( err ) {
		dev_err(dev, "%s: Missing sensor_model property (%d)", __func__, err);
		return false;
	}

	return !strcmp(outs, sensor_model);
}
EXPORT_SYMBOL(tis_sensor_is_sensor_model);

static void tis_sensor_init_struct(struct tis_sensor *priv, struct i2c_client *client, u32 sensor_width, u32 sensor_height, int sensor_model, const struct tis_sensor_ops *sensor_ops)
{
	static const struct tis_sensor_ops empty_ops = {};

	priv->i2c_client = client;
	priv->dev = &client->dev;
	priv->device_node = client->dev.of_node;
	priv->SENSOR_WIDTH = sensor_width;
	priv->SENSOR_HEIGHT = sensor_height;
	priv->sensor_model = sensor_model;
	priv->sensor_ops = sensor_ops ? sensor_ops : &empty_ops;
}

struct tis_sensor *tis_sensor_create(struct i2c_client *client, u32 sensor_width, u32 sensor_height, int sensor_model, size_t alloc_sensor_priv_size, const struct tis_sensor_ops *sensor_ops)
{
	struct device *dev = &client->dev;
	struct tis_sensor *priv;
	struct tis_sensor_connection *cxn;
	int err;

	priv = devm_kzalloc(dev, sizeof(struct tis_sensor), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	tis_sensor_init_struct(priv, client, sensor_width, sensor_height, sensor_model, sensor_ops);

	tis_sensor_gpios_get(priv);	

	if( sensor_ops && sensor_ops->sensor_power_on )
	{
		err = sensor_ops->sensor_power_on(priv);
		if( err )
		{
			dev_err(dev, "%s: Failed to power on sensor (%d)", __func__, err);
			goto exit_put_gpios;
		}
	}

	priv->priv = devm_kzalloc(dev, alloc_sensor_priv_size, GFP_KERNEL);
	if( !priv->priv )
		return ERR_PTR(-ENOMEM);

	cxn = tis_sensor_connection_create(&client->dev);
	if( IS_ERR(cxn) )
		return ERR_CAST(cxn);
	priv->connection = cxn;

	tis_sensor_init_sensor_board_part(&priv->sensor_board_part);
	tis_sensor_init_adapter_board_part(&priv->adapter_board_part);

	return priv;

exit_put_gpios:
	tis_sensor_gpios_put(priv);
	return ERR_PTR(err);
}
EXPORT_SYMBOL(tis_sensor_create);

void tis_sensor_remove(struct tis_sensor *priv)
{
	tegracam_v4l2subdev_unregister(priv->tc_dev);	
	tegracam_device_unregister(priv->tc_dev);

	tis_sensor_connection_release(priv->connection);

	tis_sensor_gpios_put(priv);
}
EXPORT_SYMBOL(tis_sensor_remove);

static int of_property_read_string_u32(struct device_node *node, const char *propname, u32 *value)
{
	const char *outs;

	int ret = of_property_read_string(node, propname, &outs);
	if( ret < 0 )
	{
		pr_err("%s: Error reading node property '%s': %d", __func__, propname, ret);
		return ret;
	}
	
	ret = sscanf(outs, "%u", value);
	if( ret != 1 )
	{
		pr_err("%s: Malformed node property '%s': '%s'", __func__, propname, outs);
		return -EINVAL;
	}

	return 0;
}

static int of_property_read_string_u64(struct device_node *node, const char *propname, u64 *value)
{
	const char *outs;

	int ret = of_property_read_string(node, propname, &outs);
	if( ret < 0 )
	{
		pr_err("%s: Error reading node property '%s': %d", __func__, propname, ret);
		return ret;
	}
	
	ret = sscanf(outs, "%llu", value);
	if( ret != 1 )
	{
		pr_err("%s: Malformed node property '%s': '%s'", __func__, propname, outs);
		return -EINVAL;
	}

	return 0;
}

static int build_framerate_list( struct device *dev, struct camera_common_frmfmt *frmfmt, int max_framerate )
{
	const int default_framerates[] = { 240, 120, 60, 30, 15, 10, 5, 1 };
	const int num_default_framerates = ARRAY_SIZE(default_framerates);
	int i, j;
	int num_frame_rates;
	int *frame_rate_list;

	// Skip through default_framerates until we find a value lower than max_framerate
	for( i = 0; i < num_default_framerates && default_framerates[i] >= max_framerate; ++i )
	{
	}

	// The resulting number of frame rates is 1 + the remaining default_framerates
	num_frame_rates = 1 + (num_default_framerates - i);

	frame_rate_list = devm_kzalloc(dev, sizeof(int) * num_frame_rates, GFP_KERNEL);
	if( !frame_rate_list )
		return -ENOMEM;

	// Put the maximum frame rate first
	frame_rate_list[0] = max_framerate;

	// Copy remaining default_framerates
	for( j = 1; i < num_default_framerates; ++i, ++j )
	{
		frame_rate_list[j] = default_framerates[i];
	}

	frmfmt->framerates = frame_rate_list;
	frmfmt->num_framerates = num_frame_rates;

	return 0;
}

struct camera_common_sensor_ops *tis_sensor_tegracam_build_sensor_ops(struct tis_sensor *priv, struct camera_common_sensor_ops *template)
{
	int mode = 0;
	struct device_node *child;
	struct camera_common_sensor_ops *sensor_ops;

	for_each_child_of_node(priv->device_node, child)
    {
    	if(!strncmp(child->name, "mode", strlen("mode")))
		{
			u32 width, height;
			u64 max_framerate, framerate_factor;
			const char *outs = 0;
			int ret;
			struct camera_common_frmfmt *frmfmt = &priv->frmfmt[mode];

			ret = of_property_read_string_u32(child, "active_w", &width);
			if( ret != 0 )
			{
				dev_warn(priv->dev, "%s: Invalid 'active_w', skipping mode %d", __func__, mode);
				continue;
			}
			ret = of_property_read_string_u32(child, "active_h", &height);
			if( ret != 0 )
			{
				dev_warn(priv->dev, "%s: Invalid 'active_h', skipping mode %d", __func__, mode);
				continue;
			}
			ret = of_property_read_string_u64(child, "max_framerate", &max_framerate);
			if( ret != 0 )
			{
				dev_warn(priv->dev, "%s: Invalid 'max_framerate', skipping mode %d", __func__, mode);
				continue;
			}
			ret = of_property_read_string_u64(child, "framerate_factor", &framerate_factor);
			if( ret != 0 )
			{
				dev_warn(priv->dev, "%s: Invalid 'framerate_factor', skipping mode %d", __func__, mode);
				continue;
			}

			ret = of_property_read_string(child, "raw_fourcc_mono", &outs);
			if( ret )
			{
				dev_warn(priv->dev, "%s: Missing 'raw_fourcc_mono', skipping mode %d", __func__, mode);
				continue;
			}
			ret = of_property_read_string(child, "raw_fourcc_color", &outs);
			if( ret )
			{
				dev_warn(priv->dev, "%s: Missing 'raw_fourcc_color', skipping mode %d", __func__, mode);
				continue;
			}

			if( framerate_factor == 0 )
			{
				dev_err(priv->dev, "%s: Unexpected framerate_factor: %lld, skipping mode %d", __func__, framerate_factor, mode );
				continue;
			}

			ret = build_framerate_list(priv->dev, frmfmt, max_framerate / framerate_factor);
			if( ret != 0 )
			{
				return ERR_PTR(ret);
			}

			frmfmt->size.width = width;
			frmfmt->size.height = height;
			frmfmt->hdr_en = 0;
			frmfmt->mode = mode;
			++mode;
		}
	}

	if (mode == 0)
	{
		dev_err(priv->dev, "%s: No valid mode found", __func__);
		return ERR_PTR(-EINVAL);
	}

	sensor_ops = devm_kzalloc(priv->dev, sizeof(struct camera_common_sensor_ops), GFP_KERNEL);
	memcpy (sensor_ops, template, sizeof(struct camera_common_sensor_ops));

	sensor_ops->numfrmfmts = mode;
	sensor_ops->frmfmt_table = priv->frmfmt;

	return sensor_ops;
}
EXPORT_SYMBOL(tis_sensor_tegracam_build_sensor_ops);

int tis_sensor_update_raw_fourcc(struct tis_sensor *priv)
{
	int mode = 0;
	struct device_node *child;

	dev_dbg(priv->dev, "%s Updating raw_fourcc values", __func__ );

	for_each_child_of_node(priv->device_node, child)
    {
    	if(!strncmp(child->name, "mode", strlen("mode")))
		{
			const char *outs = 0;
			int ret;

			if( priv->sensor_board_part.sensor_type == TIS_SENSOR_TYPE_MONO )
			{
				ret = of_property_read_string(child, "raw_fourcc_mono", &outs);
				if( ret == 0 )
				{
					dev_dbg(priv->dev, "%s: mono mode %d fourcc = %s", __func__, mode, outs);
					priv->raw_fourcc[mode] = *(u32*)outs;
				}
				else
				{
					dev_warn(priv->dev, "%s: Missing 'raw_fourcc_mono', mode %d raw_fourcc is null", __func__, mode);
				}
			}
			else
			{
				ret = of_property_read_string(child, "raw_fourcc_color", &outs);
				if( ret == 0 )
				{
					dev_dbg(priv->dev, "%s: color mode %d fourcc = %s", __func__, mode, outs);
					priv->raw_fourcc[mode] = *(u32*)outs;	
				}
				else
				{
					dev_warn(priv->dev, "%s: Missing 'raw_fourcc_mono', mode %d raw_fourcc is null", __func__, mode);
				}
			}

			mode += 1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_update_raw_fourcc);



#if 1

#endif


int tis_sensor_tegracam_device_register(struct tis_sensor *priv, const struct regmap_config *sensor_regmap_config, const struct tegracam_ctrl_ops *tcctrl_ops, struct camera_common_sensor_ops *sensor_ops, const char *sensor_driver_name)
{
	struct tegracam_device *tc_dev;
	int err;

	tc_dev = devm_kzalloc(priv->dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev) {
		return -ENOMEM;
	}

	tc_dev->client = priv->i2c_client;
	tc_dev->dev = priv->dev;
	tc_dev->dev_regmap_config = sensor_regmap_config;
	tc_dev->sensor_ops = sensor_ops;
	tc_dev->tcctrl_ops = tcctrl_ops;
	strncpy(tc_dev->name, sensor_driver_name, sizeof(tc_dev->name));

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(priv->dev, "tegra camera driver registration failed\n");
		return err;
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = begin_tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(priv->dev, "begin tegra camera subdev registration failed\n");
		return err;
	}

	tis_sensor_ctrl_create_all(priv, priv->tc_dev->s_data->ctrl_handler);

	if( priv->sensor_ops->sensor_register_private_controls )
	{
		priv->sensor_ops->sensor_register_private_controls(priv, priv->tc_dev->s_data->ctrl_handler);
	}

	err = end_tegracam_v4l2subdev_register(tc_dev);
	if (err) {
		dev_err(priv->dev, "end tegra camera subdev registration failed\n");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_tegracam_device_register);

static int tis_sensor_test_gpio(struct tis_sensor *priv, struct gpio_desc *gpio_trig_lvl_sel)
{
	return gpiod_direction_output(gpio_trig_lvl_sel, priv->ctrl.trigger_in_polarity);
}

bool tis_sensor_is_device_valid(struct tis_sensor *priv)
{
	struct device *dev = priv->dev;

	bool is_present = false;	
	int err = tis_sensor_connection_check_device_present(priv->connection, &is_present);
	if( !err )
		return is_present;	

	dev_dbg(dev, "%s: trying via GPIO", __func__);

	// Try setting trig_lvl_sel GPIO on the adapter board
	if( priv->gpio_trig_lvl_sel )
	{
		is_present = (tis_sensor_test_gpio(priv, priv->gpio_trig_lvl_sel) == 0);
	}
	else
	{
		struct gpio_desc *gpio_trig_level_sel = gpiod_get(dev, "triglvlsel", GPIOD_ASIS );
		if( PTR_ERR_OR_ZERO(gpio_trig_level_sel) )
		{
			// If the GPIO is not found, it is probably because it is missing from the device tree,
			// for example on a custom platform. Assume everything is OK
			dev_warn_once(dev, "%s: Failed to acquire 'triglvlsel' gpio (%ld), assuming modified device tree", __func__, PTR_ERR(gpio_trig_level_sel));
			is_present = 1;
		}
		else
		{
			is_present = (tis_sensor_test_gpio(priv, gpio_trig_level_sel) == 0);		
			gpiod_put(gpio_trig_level_sel);
		}
	}

	return is_present;
}

int tis_sensor_configure_connection(struct tis_sensor *priv, int num_csi_lanes, u32 clkout_hz, u32 *actual_clkout_hz)
{
	struct device *dev = priv->dev;

	int err = tis_sensor_connection_configure_csi_lanes(priv->connection, priv->mode_lanes);
	if( err )
	{
		dev_err(dev, "%s: tis_sensor_connection_configure_csi_lanes failed: %d", __func__, err);
		return err;
	}

	err = tis_sensor_connection_configure_clkout(priv->connection, clkout_hz, actual_clkout_hz);
	if( err )
	{
		dev_err(dev, "%s: tis_sensor_connection_configure_clkout failed: %d", __func__, err);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tis_sensor_configure_connection);

static int __init tis_sensor_common_init(void)
{
	return 0;
}

static void __exit tis_sensor_common_exit(void)
{
}

module_init(tis_sensor_common_init);
module_exit(tis_sensor_common_exit);

MODULE_DESCRIPTION("Common functions for all The Imaging Source sensor modules");
MODULE_AUTHOR("Arne Caspari <arne.caspari@theimagingsource.com");
MODULE_LICENSE("GPL v2");
