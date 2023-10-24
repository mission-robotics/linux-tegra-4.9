
#include <linux/kernel.h>
#include <linux/i2c.h>

extern bool tis_fpdlink_is_serializer_connected(struct i2c_client *client, int channel);
extern int tis_fpdlink_configure_serializer_clkout(struct i2c_client *client, int channel, u32 clkout_hz, u32 *actual_clkout_hz);
extern int tis_fpdlink_configure_serializer_csi_lanes(struct i2c_client *client, int channel, int num_serializer_csi_lanes);
extern int tis_fpdlink_get_gpio_out_status(struct i2c_client *client, int channel, int gpio, bool *status);

typedef bool (*pfn_tis_fpdlink_is_serializer_connected)(struct i2c_client *client, int channel);
typedef int (*pfn_tis_fpdlink_configure_serializer_clkout)(struct i2c_client *client, int channel, u32 clkout_hz, u32 *actual_clkout_hz);
typedef int (*pfn_tis_fpdlink_configure_serializer_csi_lanes)(struct i2c_client *client, int channel, int num_serializer_csi_lanes);
typedef int (*pfn_tis_fpdlink_get_gpio_out_status)(struct i2c_client *client, int channel, int gpio, bool *status);

