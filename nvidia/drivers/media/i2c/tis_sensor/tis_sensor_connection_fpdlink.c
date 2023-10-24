
#include "tis_sensor_connection_fpdlink.h"
#include "tis_sensor_connection_internal.h"

#include "../tis_fpdlink/tis_fpdlink.h"

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>

struct tis_sensor_connection_fpdlink
{
    struct tis_sensor_connection    parent;

    struct device                  *dev;
    struct i2c_client              *fpdlink_i2c;
    u8                              vc_id;

    pfn_tis_fpdlink_is_serializer_connected         is_serializer_connected;
    pfn_tis_fpdlink_configure_serializer_clkout     configure_serializer_clkout;
    pfn_tis_fpdlink_configure_serializer_csi_lanes  configure_serializer_csi_lanes;
    pfn_tis_fpdlink_get_gpio_out_status             get_gpio_out_status;
};

static struct tis_sensor_connection *to_sensor_connection(struct tis_sensor_connection_fpdlink *cxn)
{
    return &cxn->parent;
}
static struct tis_sensor_connection_fpdlink *to_sensor_connection_fpdlink(struct tis_sensor_connection *cxn)
{
    return container_of(cxn, struct tis_sensor_connection_fpdlink, parent);
}

static int tis_sensor_connection_fpdlink_lazy_init_functions(struct tis_sensor_connection_fpdlink *cxn)
{
    if( !cxn->is_serializer_connected )
    {
        cxn->is_serializer_connected = symbol_get(tis_fpdlink_is_serializer_connected);
    }
    if( !cxn->is_serializer_connected )
    {
		dev_warn(cxn-> dev, "%s: Failed to find symbol: tis_fpdlink_is_serializer_connected, call will fail", __func__);
        return -EINVAL;
    }

    if( !cxn->configure_serializer_clkout )
    {
	    cxn->configure_serializer_clkout = symbol_get(tis_fpdlink_configure_serializer_clkout);
    }
	if( !cxn->configure_serializer_clkout )
	{
		dev_warn(cxn-> dev, "%s: Failed to find symbol: tis_fpdlink_configure_serializer_clkout, call will fail", __func__);
        return -EINVAL;
	}

    if( !cxn->configure_serializer_csi_lanes )
    {
	    cxn->configure_serializer_csi_lanes = symbol_get(tis_fpdlink_configure_serializer_csi_lanes);	
    }
	if( !cxn->configure_serializer_csi_lanes )
	{
		dev_warn(cxn-> dev, "%s: Failed to find symbol: tis_fpdlink_configure_serializer_csi_lanes, call will fail", __func__);
        return -EINVAL;
	}

    if( !cxn->get_gpio_out_status )
    {
	    cxn->get_gpio_out_status = symbol_get(tis_fpdlink_get_gpio_out_status);	
    }
	if( !cxn->get_gpio_out_status )
	{
		dev_warn(cxn-> dev, "%s: Failed to find symbol: pfn_tis_fpdlink_get_gpio_out_status, call will fail", __func__);
        return -EINVAL;
	}

    return 0;
}

static void tis_sensor_connection_fpdlink_release(struct tis_sensor_connection *cxn)
{
    struct tis_sensor_connection_fpdlink *cxn_fpdlink = to_sensor_connection_fpdlink(cxn);

    if( cxn_fpdlink->is_serializer_connected )
        symbol_put_addr(cxn_fpdlink->is_serializer_connected);
    if( cxn_fpdlink->configure_serializer_clkout )
	    symbol_put_addr(cxn_fpdlink->configure_serializer_clkout);
    if( cxn_fpdlink->configure_serializer_csi_lanes )
        symbol_put_addr(cxn_fpdlink->configure_serializer_csi_lanes);
    if( cxn_fpdlink->get_gpio_out_status )
        symbol_put_addr(cxn_fpdlink->get_gpio_out_status);

    put_device(&cxn_fpdlink->fpdlink_i2c->dev);
}

static int tis_sensor_connection_fpdlink_check_device_present(struct tis_sensor_connection *cxn, bool *is_present)
{
    struct tis_sensor_connection_fpdlink *cxn_fpdlink = to_sensor_connection_fpdlink(cxn);

    int err = tis_sensor_connection_fpdlink_lazy_init_functions(cxn_fpdlink);
    if( err )
        return err;

    *is_present = cxn_fpdlink->is_serializer_connected(cxn_fpdlink->fpdlink_i2c, cxn_fpdlink->vc_id);

    return 0;
}

static int tis_sensor_connection_fpdlink_configure_clkout(struct tis_sensor_connection *cxn, u32 clkout_hz, u32 *actual_clkout_hz)
{
    struct tis_sensor_connection_fpdlink *cxn_fpdlink = to_sensor_connection_fpdlink(cxn);

    int err = tis_sensor_connection_fpdlink_lazy_init_functions(cxn_fpdlink);
    if( err )
        return err;

    return cxn_fpdlink->configure_serializer_clkout(cxn_fpdlink->fpdlink_i2c, cxn_fpdlink->vc_id, clkout_hz, actual_clkout_hz);
}

static int tis_sensor_connection_fpdlink_configure_csi_lanes(struct tis_sensor_connection *cxn, int num_lanes)
{
    struct tis_sensor_connection_fpdlink *cxn_fpdlink = to_sensor_connection_fpdlink(cxn);

    int err = tis_sensor_connection_fpdlink_lazy_init_functions(cxn_fpdlink);
    if( err )
        return err;

    return cxn_fpdlink->configure_serializer_csi_lanes(cxn_fpdlink->fpdlink_i2c, cxn_fpdlink->vc_id, num_lanes);
}

static int tis_sensor_connection_fpdlink_get_gpio_out_status(struct tis_sensor_connection *cxn, int gpio, bool *status)
{
    struct tis_sensor_connection_fpdlink *cxn_fpdlink = to_sensor_connection_fpdlink(cxn);

    int err = tis_sensor_connection_fpdlink_lazy_init_functions(cxn_fpdlink);
    if( err )
        return err;
    
    return cxn_fpdlink->get_gpio_out_status(cxn_fpdlink->fpdlink_i2c, cxn_fpdlink->vc_id, gpio, status);
}

struct tis_sensor_connection *tis_sensor_connection_fpdlink_create(struct device *dev, struct device_node *fpdlink_node)
{
    struct i2c_client *fpdlink_i2c;
    struct tis_sensor_connection_fpdlink *cxn;
    u8 vc_id;
    int err = 0;

    fpdlink_i2c = of_find_i2c_device_by_node(fpdlink_node);
    if( !fpdlink_i2c )
    {
        dev_err(dev, "%s: Failed to get fpdlink i2c device", __func__);
        return ERR_PTR(-EINVAL);
    }

    err = of_property_read_u8(dev->of_node, "fpdlink_vc_map", &vc_id);
    if( err )
    {
        dev_err(dev, "%s: Failed to read fpdlink_vc_map from device tree (%d)", __func__, err);
        goto exit_put_device;
    }

    cxn = devm_kzalloc(dev, sizeof(struct tis_sensor_connection_fpdlink), GFP_KERNEL);
    if( !cxn )
    {
        err = -ENOMEM;
        goto exit_put_device;
    }
    
    cxn->parent.release = tis_sensor_connection_fpdlink_release;
    cxn->parent.check_device_present = tis_sensor_connection_fpdlink_check_device_present;
    cxn->parent.configure_clkout = tis_sensor_connection_fpdlink_configure_clkout;
    cxn->parent.configure_csi_lanes = tis_sensor_connection_fpdlink_configure_csi_lanes;
    cxn->parent.get_gpio_out_status = tis_sensor_connection_fpdlink_get_gpio_out_status;

    cxn->dev = dev;
    cxn->fpdlink_i2c = fpdlink_i2c;
    cxn->vc_id = vc_id;

    dev_dbg(dev, "%s: Created connection object vc_id = %d", __func__, cxn->vc_id);

    return to_sensor_connection(cxn);

exit_put_device:
    put_device(&fpdlink_i2c->dev);
    return ERR_PTR(err);
}
