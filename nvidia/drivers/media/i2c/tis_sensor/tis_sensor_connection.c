
#include "tis_sensor_connection.h"
#include "tis_sensor_connection_internal.h"
#include "tis_sensor_connection_fpdlink.h"

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>

static int tis_sensor_connection_direct_configure_clkout(struct tis_sensor_connection *cxn, u32 clkout_hz, u32 *actual_clkout_hz)
{
    if( actual_clkout_hz )
    {
        *actual_clkout_hz = clkout_hz;        
    }

    return 0;
}

static int tis_sensor_connection_direct_configure_csi_lanes(struct tis_sensor_connection *cxn, int num_lanes)
{
    return 0;    
}

static int tis_sensor_connection_direct_get_gpio_out_status(struct tis_sensor_connection *cxn, int gpio, bool *status)
{
    *status = true;
    return 0;    
}

struct tis_sensor_connection *tis_sensor_connection_direct_create(struct device *dev)
{
    struct tis_sensor_connection *cxn = devm_kzalloc(dev, sizeof(struct tis_sensor_connection), GFP_KERNEL);

    cxn->configure_clkout = tis_sensor_connection_direct_configure_clkout;
    cxn->configure_csi_lanes = tis_sensor_connection_direct_configure_csi_lanes;
    cxn->get_gpio_out_status = tis_sensor_connection_direct_get_gpio_out_status;

    return cxn;
}

struct tis_sensor_connection *tis_sensor_connection_create(struct device *dev)
{
	struct device_node *fpdlink_node = of_parse_phandle(dev->of_node, "fpdlink", 0);
    struct tis_sensor_connection *cxn;

    if( !fpdlink_node )
    {
        dev_info(dev, "%s: No fpdlink node in device tree, direct connection", __func__);
        return tis_sensor_connection_direct_create(dev);
    }

    cxn = tis_sensor_connection_fpdlink_create(dev, fpdlink_node);
    if( IS_ERR(cxn) )
    {
        dev_err(dev, "%s: Failed to create fpdlink connection object (%ld)", __func__, PTR_ERR(cxn));        
    }

    of_node_put(fpdlink_node);

    return cxn;
}

int tis_sensor_connection_check_device_present(struct tis_sensor_connection *cxn, bool *is_present)
{
    if( !cxn->check_device_present )
        return -EINVAL;

    return cxn->check_device_present(cxn, is_present);
}

int tis_sensor_connection_configure_clkout(struct tis_sensor_connection *cxn, u32 clkout_hz, u32 *actual_clkout_hz)
{
    if( !cxn->configure_clkout )
        return -EINVAL;

    return cxn->configure_clkout(cxn, clkout_hz, actual_clkout_hz);
}
int tis_sensor_connection_configure_csi_lanes(struct tis_sensor_connection *cxn, int num_lanes)
{
    if( !cxn->configure_csi_lanes )
        return -EINVAL;

    return cxn->configure_csi_lanes(cxn, num_lanes);
}

int tis_sensor_connection_get_gpio_out_status(struct tis_sensor_connection *cxn, int gpio, bool *status)
{
    if( !cxn->get_gpio_out_status )
        return -EINVAL;
    
    return cxn->get_gpio_out_status(cxn, gpio, status);
}

void tis_sensor_connection_release(struct tis_sensor_connection *cxn)
{
    if( !cxn->release )
        return;
    
    cxn->release(cxn);
}