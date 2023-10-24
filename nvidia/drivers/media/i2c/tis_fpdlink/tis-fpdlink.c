#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/driver.h>

#define TIS_FPDLINK_FLAG_VERIFY 1

#define TIS_FPDLINK_UB954_SLAVE_ID_BASE    0x5d
#define TIS_FPDLINK_UB954_SLAVE_ALIAS_BASE 0x65

#define TIS_FPDLINK_GPIO_TYPE_INITIALIZING           (0)
#define TIS_FPDLINK_GPIO_TYPE_TCA7408                (1)
#define TIS_FPDLINK_GPIO_TYPE_TCA6408                (2)
#define TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT            (3)

#define TCA7408_DEVICE_ID           0x1
#define TCA7408_DIRECTION           0x3
#define TCA7408_OUTPUT              0x5
#define TCA7408_OUTPUT_IMPEDANCE    0x7
#define TCA7408_INPUT_DEFAULT       0x9
#define TCA7408_PULL_UP_ENABLE      0xb
#define TCA7408_PULL_UP_SELECT      0xd
#define TCA7408_INPUT_STATUS        0xf
#define TCA7408_IRQ_MASK            0x11
#define TCA7408_IRQ_STATUS          0x13


#define MAX_CHANNELS				2
#define MAX_BACKCHANNEL_SLOTS		4

#define MAX_GPIO_SERIALIZER			4
#define MAX_GPIO_DESERIALIZER		7

enum tis_fpdlink_backchannel_direction {
	backchannel_unused = 0,
	backchannel_up,
	backchannel_down,
	backchannel_output_0,
	backchannel_output_1,
};

struct tis_fpdlink_backchannel {
	enum tis_fpdlink_backchannel_direction direction;
	u8 deser_gpio;
	u8 ser_gpio;
};

struct tis_fpdlink {
	struct delayed_work detect_link_work;

	struct regmap *deser_regmap;
	struct regmap *ser_regmap[MAX_CHANNELS];
	struct regmap *deser_chan_regmap[MAX_CHANNELS];
	struct regmap *gpio_regmap[MAX_CHANNELS];
	struct i2c_client *deser_client;
	struct i2c_client *ser_client[MAX_CHANNELS];
	struct i2c_client *deser_chan_client[MAX_CHANNELS];
    struct i2c_client *gpio_client[MAX_CHANNELS];
	struct dentry *fpdlink_dir;
	struct dentry *deser_d;
	struct dentry *ser_d;
	u8 debug_addr;
	u8 debug_channel;

	u8 slave_addr[MAX_CHANNELS][8];
	u8 slave_map_addr[MAX_CHANNELS][8];
	int slave_count[MAX_CHANNELS];
	u8 ser_map_addr[MAX_CHANNELS];
	u8 deser_chan_addr[MAX_CHANNELS];
	u8 n_mappings[MAX_CHANNELS];
	u8 clkout_ctrl1[MAX_CHANNELS];
	u8 clkout_ctrl2[MAX_CHANNELS];
	u32 mipi_lane_count[MAX_CHANNELS];
	u32 deser_lane_count;
	u8 vc_map[MAX_CHANNELS];
	u8 coax_power_gpio[MAX_CHANNELS];

	int n_busses;
	int current_channel;
	int ser_lock[MAX_CHANNELS];

	u8 gpio_addr_base;
	int gpio_type[MAX_CHANNELS];
    struct gpio_chip gpio_chip;
	u64 prev_gpio_warn;

	struct tis_fpdlink_backchannel backchannels[MAX_CHANNELS][MAX_BACKCHANNEL_SLOTS];

	struct mutex i2c_mutex;
};

static int tis_fpdlink_direction_output(struct gpio_chip *gpio, unsigned int offset, int val);
static int tis_fpdlink_direction_input(struct gpio_chip *gpio, unsigned int offset);
static int tis_fpdlink_gpio_get(struct gpio_chip *gpio, unsigned int offset);
static void tis_fpdlink_gpio_set(struct gpio_chip *gpio, unsigned int offset, int val);
static int tis_fpdlink_map_i2c(struct tis_fpdlink *priv, unsigned int channel,
                               u8 src_addr, u8 map_addr);
static int tis_fpdlink_add_gpiochip_once(struct tis_fpdlink *priv);


static const struct gpio_chip tis_fpdlink_gpio_chip = {
    .label          = "gpio-tis-fpdlink",
    .owner          = THIS_MODULE,
    .direction_input    = tis_fpdlink_direction_input,
    .direction_output   = tis_fpdlink_direction_output,
    .set            = tis_fpdlink_gpio_set,
    .get            = tis_fpdlink_gpio_get,
    .base           = -1,
    .ngpio          = 8,
    .can_sleep      = true,
};


static const struct regmap_config tis_fpdlink_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.cache_type	= REGCACHE_NONE,
	.use_single_rw	= true,
};

static const struct regmap_config tis_fpdlink_gpio_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.cache_type	= REGCACHE_RBTREE,
	.use_single_rw	= true,
};

struct fpdlink_reg {
	char addr;
	char val;
	unsigned int flags;
};

static const struct of_device_id tis_fpdlink_of_match [] = {
	{ .compatible = "tis,tis-fpdlink" },
	{ }
};

static const struct fpdlink_reg fpdlink_deser_reglist[] = {
	{0x0d, 0x09, 0x0}, // IO_CTL ( 0x9 = Detect IO Voltage level)
	{0x7d, 0x05, 0x0}, // PORT_PASS_CTL (0x05 = Enable Watchdog, Assert PASS after 1 Frame)
	{0x0f, 0x7f, 0x0}, // GPIO_INPUT_CTL,
	{0x10, 0x0, 0x0}, // GPIO0_PIN_CTL,
	{0x11, 0x0, 0x0}, // GPIO1_PIN_CTL,
	{0x4c, 0x01, 0x0}, // FPD3_PORT_SEL,

	{0x59, 0x83, TIS_FPDLINK_FLAG_VERIFY}, // DATAPATH_CTL1 ( 0x83 = Disable loading from FC; Forward four GPIOs )
	{0x6e, 0x10, TIS_FPDLINK_FLAG_VERIFY}, // BC_GPIO_CTL0 ( 0x10 = GPIO0 on BC_GPIO0 and GPIO1 on BC_GPIO1 )

	// {0x5c, 0x20, 0x0}, // SER_ALIAS_ID

	{0x1f, 0x00, 0x0}, // CSI_PLL_CTL (0x0 = 1.6gps, 0x2 = 800Mbps)
	{0x33, 0x21, 0x0}, // CSI_CTL ( 0x21 = 2 lanes, CSI Enable, Continuous Clock Disable )
	{0x21, 0x01, 0x0}, // FWD_CTL2 ( 0x1 = round robin forward )
	{0x20, 0x00, 0x0}, // FWD_CTL1 ( 0x20 = disable rx port 1 / 0x00 = forward both ports )

	{ },
};

static const struct fpdlink_reg fpdlink_deser_chan_reglist[] = {
	{0x58, 0x5e, 0x0}, // BCC_CONFIG
	{0x6d, 0x44, 0x0}, // Configures port to coax mode and FPD III to CSI mode
	{}
};

static const struct fpdlink_reg fpdlink_ser_reglist[] = {
	// {0x06, 0x41, 0x0},
	// {0x07, 0x1b, 0x0},
	{0x0b, 0x19, 0x0}, // SCL_HIGH_TIME
	{0x0c, 0x19, 0x0}, // SCL_LOW_TIME
	{0x0d, 0xF0, 0x0}, // LOCAL_GPIO_DATA: (default)
	{0x0e, 0x0F, 0x0}, // GPIO_INPUT_CTL: Set GPIO0-3 to input (default)
	{0x33, 0x03, 0x0}, // DATAPATH_CTL1 ( 0x0 = Forward four GPIO from serializer )
	//{0x02, 0x13, 0x0}, // GENERAL_CFG ( 0xb = 2-lane config, CRC enable, 1.8V Mode)
	{0x02, 0x33, 0x0}, // GENERAL_CFG ( 0xb = 4-lane config, CRC enable, 1.8V Mode)

	// {0x0d, 0x0, 0x0},
	// {0x0e, 0x03, 0x0},
	{ },
};

// 1948 x 1096
// 0x79c x 0x448
static const struct fpdlink_reg fpdlink_ser_patgen[] = {
	{0xB0, 0x00, 0x0}, // # Indirect Pattern Gen Registers
	{0xB1, 0x01, 0x0}, // # PGEN_CTL
	{0xB2, 0x01, 0x0}, //
	{0xB1, 0x02, 0x0}, // # PGEN_CFG
	{0xB2, 0x33, 0x0}, //
	{0xB1, 0x03, 0x0}, // # PGEN_CSI_DI
	{0xB2, 0x24, 0x0}, //
	{0xB1, 0x04, 0x0}, // # PGEN_LINE_SIZE1
	{0xB2, 0x07, 0x0}, //
	{0xB1, 0x05, 0x0}, // # PGEN_LINE_SIZE0
	{0xB2, 0x9c, 0x0}, //
	{0xB1, 0x06, 0x0}, // # PGEN_BAR_SIZE1
	{0xB2, 0x01, 0x0}, //
	{0xB1, 0x07, 0x0}, // # PGEN_BAR_SIZE0
	{0xB2, 0xE0, 0x0}, //
	{0xB1, 0x08, 0x0}, // # PGEN_ACT_LPF1
	{0xB2, 0x04, 0x0}, //
	{0xB1, 0x09, 0x0}, // # PGEN_ACT_LPF0
	{0xB2, 0x48, 0x0}, //
	{0xB1, 0x0A, 0x0}, // # PGEN_TOT_LPF1
	{0xB2, 0x04, 0x0}, //
	{0xB1, 0x0B, 0x0}, // # PGEN_TOT_LPF0
	{0xB2, 0x48, 0x0}, //
	{0xB1, 0x0C, 0x0}, // # PGEN_LINE_PD1
	{0xB2, 0x0C, 0x0}, //
	{0xB1, 0x0D, 0x0}, // # PGEN_LINE_PD0
	{0xB2, 0x67, 0x0}, //
	{0xB1, 0x0E, 0x0}, // # PGEN_VBP
	{0xB2, 0x21, 0x0}, //
	{0xB1, 0x0F, 0x0}, // # PGEN_VFP
	{0xB2, 0x0A, 0x0}, //
	{},
};

static int tis_fpdlink_deser_debug_read(void *data, u64 *val)
{
	struct tis_fpdlink *priv = (struct tis_fpdlink*)data;
	int err;
	unsigned int tmpval;

	err = regmap_read(priv->deser_chan_regmap[priv->debug_channel], priv->debug_addr, &tmpval);
	if (err)
		pr_err("%s:i2c read failed, %x = %x\n",
			__func__, priv->debug_addr, tmpval);
	*val = tmpval;
	pr_info("%s:i2c read, %x = %x\n",
		__func__, priv->debug_addr, tmpval);

	return err;
}

static int tis_fpdlink_deser_debug_write(void *data, u64 val)
{
	struct tis_fpdlink *priv = (struct tis_fpdlink*)data;
	int err;

	err = regmap_write(priv->deser_chan_regmap[priv->debug_channel], priv->debug_addr, val & 0xff);
	if (err)
		pr_err("%s:i2c write failed, %x = %llx\n",
			__func__, priv->debug_addr, val & 0xff);
	return err;
}

static int tis_fpdlink_ser_debug_read(void *data, u64 *val)
{
	struct tis_fpdlink *priv = (struct tis_fpdlink*)data;
	int err;
	unsigned int tmpval;

	err = regmap_read(priv->ser_regmap[priv->debug_channel], priv->debug_addr, &tmpval);
	if (err)
		pr_err("%s:i2c read failed, %x = %x\n",
			__func__, priv->debug_addr, tmpval);
	*val = tmpval;

	return err;
}

static int tis_fpdlink_ser_debug_write(void *data, u64 val)
{
	struct tis_fpdlink *priv = (struct tis_fpdlink*)data;
	int err;

	err = regmap_write(priv->ser_regmap[priv->debug_channel], priv->debug_addr, val & 0xff);
	if (err)
		pr_err("%s:i2c write failed, %x = %llx\n",
			__func__, priv->debug_addr, val & 0xff);
	return err;
}

DEFINE_SIMPLE_ATTRIBUTE(tis_fpdlink_fops_ser, tis_fpdlink_ser_debug_read,
						tis_fpdlink_ser_debug_write, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(tis_fpdlink_fops_deser, tis_fpdlink_deser_debug_read,
						tis_fpdlink_deser_debug_write, "0x%02llx\n");

static struct i2c_board_info ds90ub953_serializer_info = {
	I2C_BOARD_INFO("ds90ub953", 0x18),
};


static struct dentry *tis_fpdlink_debug_create(unsigned short addr, struct tis_fpdlink *priv)
{
	struct dentry *fpdlink_dir;
	struct dentry *retval;
	char dirname[32];
	static int i=0;

	snprintf(dirname, sizeof(dirname), "tis-fpdlink-%x_%d", addr, i++);

	fpdlink_dir = debugfs_create_dir(dirname, NULL);
	if (!fpdlink_dir)
	{
		pr_err("%s: failed to create debug dir\n", __func__);
		return NULL;
	}

	retval = debugfs_create_x8("addr", S_IRUGO | S_IWUGO, fpdlink_dir, &priv->debug_addr);
	if (!retval)
		goto error_out;

	retval = debugfs_create_x8("channel", S_IRUGO | S_IWUGO, fpdlink_dir, &priv->debug_channel);
	if (!retval)
		goto error_out;

	priv->deser_d = debugfs_create_file("data-deser", S_IRUGO | S_IWUGO, fpdlink_dir,
		priv, &tis_fpdlink_fops_deser);
	if (!priv->deser_d)
		goto error_out;

	priv->ser_d = debugfs_create_file("data-ser", S_IRUGO | S_IWUGO, fpdlink_dir,
		priv, &tis_fpdlink_fops_ser);
	if (!priv->ser_d)
		goto error_out;

	return fpdlink_dir;

	error_out:
	debugfs_remove_recursive(fpdlink_dir);
	return NULL;
}


struct tis_fpdlink_pll_settings {
	u8 hs_clk_div;
	u8 m;
	u8 n;
};

static u32 tis_fpdlink_calc_output_clock(u32 refclk_hz, int hs_clk_div, int m, int n)
{
	u64 FC = refclk_hz * 160ull;
	u64 output_clock = FC * m / ((1ull << hs_clk_div) * n);
	if (output_clock > UINT_MAX) {
			return 0;
	}

	return (u32)output_clock;
}

static int tis_fpdlink_find_pll_settings(u32 refclk_hz, u32 target_hz,
	struct tis_fpdlink_pll_settings *settings)
{
    u64 FC = refclk_hz * 160ull;
    u32 best_diff = target_hz;
    int hs_clk_div;

    for(hs_clk_div = 0; hs_clk_div <= 4; hs_clk_div += 1)
    {
    	int m;
        if( FC / (1ull << hs_clk_div) > 1050000000 ) // FC/HS_CLK_DIV is not allowed above 1.05 GHz
            continue;

        for(m = 1; m <= 31;m++)
        {
        	int n;
            for(n = 1; n <= 255;n++)
            {
                u32 test = tis_fpdlink_calc_output_clock(refclk_hz, hs_clk_div, m, n);
                u32 diff = (test > target_hz) ? test - target_hz : target_hz - test;

                if( diff < best_diff )
                {
                    settings->hs_clk_div = hs_clk_div;
                    settings->m = m;
                    settings->n = n;
                    best_diff = diff;
                }
                if( diff == 0 )
                {
                    return 0;
                }
            }
        }
    }

    return 0;
}


static int tis_fpdlink_parse_subnode(struct tis_fpdlink *priv, struct device_node *node, int index)
{
	struct i2c_client *client = priv->deser_client;
	u8 data[16];
	u32 clkout_hz = 0;
	int len;
	int i;
	int err;
	u32 deser_gpio = 0, ser_gpio = 0;
	struct device_node *bc_node, *bcs_node;
	const char *outs = NULL;

	err = of_property_read_u32(node, "fpdlink_clkout_hz", &clkout_hz);
	if(err)
	{
		err = of_property_read_u8_array(node, "clkout_ctrl", data, 2);
		if (err) {
			dev_err(&client->dev, "%s:dt property read failed: %s\n",
				__func__, "clkout_ctrl");
			return err;
		}
		priv->clkout_ctrl1[index] = data[0];
		priv->clkout_ctrl2[index] = data[1];
	} else {
		struct tis_fpdlink_pll_settings pll_settings = {0};
		tis_fpdlink_find_pll_settings(25000000, clkout_hz, &pll_settings);
		priv->clkout_ctrl1[index] = (pll_settings.hs_clk_div << 5) | pll_settings.m;
		priv->clkout_ctrl2[index] = pll_settings.n;
	}

	if (!of_get_property(node, "fpdlink_slave_map", &len)) {
		dev_err(&client->dev, "fpdlink_slave_map property is missing!\n");
		return -ENOENT;
	}
	if (len&1)
	{
		dev_err(&client->dev, "fpdlink_slave_map property must have even number of elements\n");
		return -EINVAL;
	}

	if (len>16)
	{
		dev_err(&client->dev, "only up to 8 slave maps are supported\n");
		return -EINVAL;
	}

	err = of_property_read_u8_array(node, "fpdlink_slave_map", data, len);
	if (err) {
		pr_err("%s:dt property read failed: %s\n",
			__func__, "slave_map");
		return err;
	}

	priv->slave_count[index] = len/2;
	for (i=0; i < len; i+=2)
	{
		priv->slave_addr[index][i/2] = data[i];
		priv->slave_map_addr[index][i/2] = data[i+1];
	}

	err = of_property_read_u8(node, "fpdlink_ser_map_addr", &priv->ser_map_addr[index]);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read ser_map_addr\n");
		return err;
	}

	err = of_property_read_u32(node, "fpdlink_ser_lanes", &priv->mipi_lane_count[index]);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read ser_lanes\n");
		return err;
	}
	err = of_property_read_u8(node, "fpdlink_vc_map", &priv->vc_map[index]);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read vc_map\n");
		return err;
	}

	err = of_property_read_u8(node, "fpdlink_coax_pwr_gpio", &priv->coax_power_gpio[index]);
	if( err < 0 )
	{
		// Ignore errors, not all platforms have this
		priv->coax_power_gpio[index] = 0xFF;
		err = 0;
	}

    bcs_node = of_get_child_by_name(node, "fpdlink_backchannel_signals");
	if( bcs_node != NULL )
	{
		i = 0;

		for_each_child_of_node(bcs_node, bc_node)
		{
			if( i >= MAX_BACKCHANNEL_SLOTS )
			{
				dev_info(&client->dev, "%s: Already found %d backchannel signals, ignoring all others", __func__, i);
				break;
			}

			err = of_property_read_string(bc_node, "direction", &outs );
			if( err )
			{
				dev_err(&client->dev, "%s: Signal %s -- Failed to read 'direction'", __func__, bc_node->name);
				continue;
			}
			if( strcmp(outs, "up") == 0 )
			{
				priv->backchannels[index][i].direction = backchannel_up;
			}
			else if( strcmp(outs, "down") == 0 )
			{
				priv->backchannels[index][i].direction = backchannel_down;
			}
			else if( strcmp(outs, "constant-0") == 0 )
			{
				priv->backchannels[index][i].direction = backchannel_output_0;
			}
			else if( strcmp(outs, "constant-1") == 0 )
			{
				priv->backchannels[index][i].direction = backchannel_output_1;
			}			
			else
			{
				dev_err(&client->dev, "%s: Signal %s -- unexpected 'direction': %s", __func__, bc_node->name, outs );
				continue;
			}

			if( priv->backchannels[index][i].direction == backchannel_up || priv->backchannels[index][i].direction == backchannel_down )
			{
				err = of_property_read_u32( bc_node, "deser-gpio", &deser_gpio );
				if( err )
				{
					dev_err(&client->dev, "%s: Signal %s -- Failed to read 'deser-gpio'", __func__, bc_node->name);
					continue;
				}
				if( deser_gpio >= MAX_GPIO_DESERIALIZER )
				{
					dev_err(&client->dev, "%s: Signal %s -- 'deser-gpio' out of range (0..%d): %d", __func__, bc_node->name, MAX_GPIO_DESERIALIZER-1, deser_gpio );
					continue;
				}

				priv->backchannels[index][i].deser_gpio = deser_gpio;
			}

			err = of_property_read_u32( bc_node, "ser-gpio", &ser_gpio );
			if( err )
			{
				dev_err(&client->dev, "%s: Signal %s -- Failed to read 'ser-gpio'", __func__, bc_node->name);
				continue;
			}
			if( ser_gpio >= MAX_GPIO_SERIALIZER )
			{
				dev_err(&client->dev, "%s: Signal %s -- 'ser-gpio' out of range (0..%d): %d", __func__, bc_node->name, MAX_GPIO_SERIALIZER-1, ser_gpio );
				continue;
			}
			
			priv->backchannels[index][i].ser_gpio = ser_gpio;

			switch( priv->backchannels[index][i].direction )
			{
			case backchannel_up:
				dev_info(&client->dev, "%s: found backchannel signal %s: %d -> %d",
					__func__, bc_node->name, priv->backchannels[index][i].deser_gpio, priv->backchannels[index][i].ser_gpio );
				break;
			case backchannel_down:
				dev_info(&client->dev, "%s: found backchannel signal %s: %d <- %d",
					__func__, bc_node->name, priv->backchannels[index][i].deser_gpio, priv->backchannels[index][i].ser_gpio );
				break;
			case backchannel_output_0:
				dev_info(&client->dev, "%s: serializer gpio %s: %d := 0", __func__, bc_node->name, priv->backchannels[index][i].ser_gpio );
				break;
			case backchannel_output_1:
				dev_info(&client->dev, "%s: serializer gpio %s: %d := 1", __func__, bc_node->name, priv->backchannels[index][i].ser_gpio );
				break;
			case backchannel_unused:
				break;
			}

			i += 1;
		}

		of_node_put(bcs_node);
	}

	return 0;
}
static int tis_fpdlink_parse_dt(struct i2c_client *client, struct tis_fpdlink *priv)
{
	struct device_node *node = client->dev.of_node;
	const struct of_device_id *match;
    int count = 0;
	int err;

	match = of_match_device(tis_fpdlink_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	err = of_property_read_u32(node, "deser_lanes", &priv->deser_lane_count);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read deser_lanes\n");
		return err;
	}

	err = of_property_read_u8(node, "gpio_addr_base", &priv->gpio_addr_base);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read gpio_addr_base\n");
		return err;
	}

	count = 0;
	while( true )
	{
		struct device_node *sensor_node = of_parse_phandle(node, "sensors", count);
		if( !sensor_node )
			break;

		dev_info(&client->dev, "%s: found sensor node %s", __func__, sensor_node->name);
		
        err = tis_fpdlink_parse_subnode(priv, sensor_node, count);
		count += 1;
	}

	priv->n_busses = count;

	if( err < 0 )
	{
		return err;
	}

	err = of_property_read_u8_array(node, "deser_chan_addrs", priv->deser_chan_addr, priv->n_busses);
	if( err < 0 )
	{
		dev_err(&client->dev, "%s: Failed to read %d u8 from deser_chan_addrs (%d)\n", __func__, priv->n_busses, err);
		return err;
	}

	return err;
}


static int tis_fpdlink_init_gpio(struct tis_fpdlink *priv, int channel)
{
	union i2c_smbus_data dummy;
	struct i2c_adapter *adap = priv->deser_client->adapter;
	static const u8 gpio_tca6408_addr = 0x20;
	static const u8 gpio_tca7408_addr = 0x43;
	int ret;
	u8 gpio_mapped_addr = priv->gpio_addr_base + channel;
	int i;
	int gpio_type = TIS_FPDLINK_GPIO_TYPE_INITIALIZING;

	dev_info(&priv->ser_client[channel]->dev, "algo: %p\n", adap->algo->smbus_xfer);

	tis_fpdlink_map_i2c(priv, channel, gpio_tca6408_addr, gpio_mapped_addr);
    ret = i2c_smbus_xfer(adap, gpio_mapped_addr, 0, I2C_SMBUS_WRITE, 0x1, I2C_SMBUS_BYTE, &dummy);
    if (ret == 0)
    {
		dev_info(&priv->ser_client[channel]->dev, "Found tca6408 gpio @%x (mapped to %x)\n", gpio_tca6408_addr, gpio_mapped_addr);
    	gpio_type = TIS_FPDLINK_GPIO_TYPE_TCA6408;
    	priv->n_mappings[channel]++;
    } else {
		tis_fpdlink_map_i2c(priv, channel, gpio_tca7408_addr, gpio_mapped_addr);
	    ret = i2c_smbus_xfer(adap, gpio_mapped_addr, 0, I2C_SMBUS_READ, 0x1, I2C_SMBUS_BYTE, &dummy);
	    if (ret == 0)
	    {
			dev_info(&priv->ser_client[channel]->dev, "Found tca7408 gpio @%x (mapped to %x)\n", gpio_tca7408_addr, gpio_mapped_addr);
	    	gpio_type = TIS_FPDLINK_GPIO_TYPE_TCA7408;
	    	priv->n_mappings[channel]++;
	    }
    }

    if (ret != 0)
    {
    	dev_err(&priv->ser_client[channel]->dev, "gpio chip probe failed\n");
    	return -ENODEV;
    }

	if( !priv->gpio_client[channel] ) // Only create this device once
	{
		dev_info(&priv->ser_client[channel]->dev, "Creating dummy gpio device");
		priv->gpio_client[channel] = i2c_new_dummy(adap, gpio_mapped_addr);

		priv->gpio_regmap[channel] = devm_regmap_init_i2c(priv->gpio_client[channel], &tis_fpdlink_gpio_regmap_config);
		if (IS_ERR(priv->gpio_regmap[channel]))
		{
			dev_err(&priv->gpio_client[channel]->dev,
				"gpio_regmap init failed: %ld\n", PTR_ERR(priv->gpio_regmap[channel]));
			return -ENODEV;
		}
	}

	// Set this last because this is our access guard
	priv->gpio_type[channel] = gpio_type;

    for (i = (channel*8); i < (channel*8)+7; i++)
    {
    	tis_fpdlink_direction_output(&priv->gpio_chip, i , 0);
    }
    tis_fpdlink_direction_input(&priv->gpio_chip, (channel*8)+7);	

    return ret;
}

#define TIS_DESER_REG_GPIOx_PIN_CTL(deser_gpio)	(0x10 + (deser_gpio))
#define TIS_DESER_REG_BC_GPIO_CTL(ser_gpio)		(0x6E + (ser_gpio/2))
#define BC_GPIO_CTL_SHIFT(ser_gpio)				(((ser_gpio) % 2) * 4)

static int tis_fpdlink_init_deserializer_backchannel( struct device* dev, struct regmap* deser_regs, struct tis_fpdlink_backchannel* bc, int deser_port )
{
	unsigned int val;

	switch( bc->direction )
	{
	case backchannel_down:
		val = (0x01)				// GPIOX_OUT EN = Output
			| (bc->ser_gpio << 5)	// GPIOX_OUTPUT_SEL
			| (deser_port << 2);	// GPIOX_OUT_SRC

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_DESER_REG_GPIOx_PIN_CTL(bc->deser_gpio), val );
		regmap_write(deser_regs, TIS_DESER_REG_GPIOx_PIN_CTL(bc->deser_gpio), val);

		break;
	case backchannel_up:
		val = (0x0)					// GPIOX_OUT EN = Input
			| (bc->ser_gpio << 5)	// GPIOX_OUTPUT_SEL
			| (deser_port << 2);	// GPIOX_OUT_SRC

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_DESER_REG_GPIOx_PIN_CTL(bc->deser_gpio), val );
		regmap_write(deser_regs, TIS_DESER_REG_GPIOx_PIN_CTL(bc->deser_gpio), val);

		val = 0;
		regmap_read(deser_regs, TIS_DESER_REG_BC_GPIO_CTL(bc->ser_gpio), &val);

		val &= ~(0xF << BC_GPIO_CTL_SHIFT(bc->ser_gpio));
		val |= (bc->deser_gpio << BC_GPIO_CTL_SHIFT(bc->ser_gpio));

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_DESER_REG_BC_GPIO_CTL(bc->ser_gpio), val );
		regmap_write(deser_regs, TIS_DESER_REG_BC_GPIO_CTL(bc->ser_gpio), val);

		break;
	default:
		break;
	}

	return 0;
}

#define TIS_SER_REG_LOCAL_GPIO_DATA 0x0D
#define TIS_SER_REG_GPIO_INPUT_CTRL 0x0E
#define TIS_SER_REG_DATAPATH_CTL1 0x33

static int tis_fpdlink_init_serializer_backchannel( struct device* dev, struct regmap* ser_regs, struct tis_fpdlink_backchannel* bc )
{
	unsigned int val;

	switch( bc->direction )
	{
	case backchannel_down:
		val = 0;
		regmap_read(ser_regs, TIS_SER_REG_GPIO_INPUT_CTRL, &val);

		val &= ~(1 << (4 + bc->ser_gpio));	// Clear GPIOx_OUT_EN
		val |= (1 << (bc->ser_gpio));		// Set GPIOx_INPUT_EN

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_SER_REG_GPIO_INPUT_CTRL, val );
		regmap_write(ser_regs, TIS_SER_REG_GPIO_INPUT_CTRL, val);

		break;
	case backchannel_up:
		val = 0;
		regmap_read(ser_regs, TIS_SER_REG_LOCAL_GPIO_DATA, &val);

		val |= (1 << (4 + bc->ser_gpio));	// Set GPIO_RMTEN

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_SER_REG_LOCAL_GPIO_DATA, val );
		regmap_write(ser_regs, TIS_SER_REG_LOCAL_GPIO_DATA, val);

		val = 0;
		regmap_read(ser_regs, TIS_SER_REG_GPIO_INPUT_CTRL, &val);

		val |= (1 << (4 + bc->ser_gpio));	// Set GPIOx_OUT_EN
		val &= ~(1 << (bc->ser_gpio));		// Clear GPIOx_INPUT_EN

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_SER_REG_GPIO_INPUT_CTRL, val );
		regmap_write(ser_regs, TIS_SER_REG_GPIO_INPUT_CTRL, val);

		break;

	case backchannel_output_0:
	case backchannel_output_1:
		val = 0;
		regmap_read(ser_regs, TIS_SER_REG_LOCAL_GPIO_DATA, &val);

		val &= ~(1 << (4 + bc->ser_gpio));	// Clear GPIO_RMTEN

		if(bc->direction == backchannel_output_1)
			val |= (1 << bc->ser_gpio);		// Set GPIO_OUT_SRC[x] = 1
		else //	bc->direction == backchannel_output_0
			val &= ~(1 << bc->ser_gpio);	// Set GPIO_OUT_SRC[x] = 0

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_SER_REG_LOCAL_GPIO_DATA, val );
		regmap_write(ser_regs, TIS_SER_REG_LOCAL_GPIO_DATA, val);

		val = 0;
		regmap_read(ser_regs, TIS_SER_REG_GPIO_INPUT_CTRL, &val);

		val |= (1 << (4 + bc->ser_gpio));	// Set GPIOx_OUT_EN
		val &= ~(1 << (bc->ser_gpio));		// Clear GPIOx_INPUT_EN

		dev_info(dev, "%s: write [%x] = %x", __func__, TIS_SER_REG_GPIO_INPUT_CTRL, val );
		regmap_write(ser_regs, TIS_SER_REG_GPIO_INPUT_CTRL, val);

	default:
		break;
	}

	return 0;
}

static int tis_fpdlink_init_serializer(struct i2c_client *client, int channel)
{
	struct tis_fpdlink *priv;
	struct i2c_board_info ser_board_info;
	const struct fpdlink_reg *reg;
	struct i2c_adapter *adap;
	int ret;
	int i;

	priv = (struct tis_fpdlink *)i2c_get_clientdata(client);

	adap = priv->deser_client->adapter;

	dev_dbg(&client->dev, "Initializing SERIALIZER on '%s'\n", adap->name);

	memcpy(&ser_board_info, &ds90ub953_serializer_info, sizeof(struct i2c_board_info));
	ser_board_info.addr = priv->ser_map_addr[channel];

	if( !priv->ser_client[channel] ) // Only create this device once
	{
		priv->ser_client[channel] = i2c_new_device(adap, &ser_board_info);
		if (!priv->ser_client[channel])
		{
			dev_err(&client->dev, "Instantiation of serializer failed\n");
			return -ENODEV;
		}

		priv->ser_regmap[channel] = devm_regmap_init_i2c(priv->ser_client[channel], &tis_fpdlink_regmap_config);
		if (IS_ERR(priv->ser_regmap[channel]))
		{
			dev_err(&priv->ser_client[channel]->dev,
				"regmap init failed: %ld\n", PTR_ERR(priv->ser_regmap[channel]));
			return -ENODEV;
		}
	}

	// Set CLKOUT config
	dev_dbg(&client->dev, "CLKOUT_CONFIG: 0x6 = 0x%x  0x7 = 0x%x\n",
		priv->clkout_ctrl1[channel], priv->clkout_ctrl2[channel]);
	ret = regmap_write(priv->ser_regmap[channel], 0x06, priv->clkout_ctrl1[channel]);
	if (ret)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, 0x06, priv->clkout_ctrl1[channel]);
	ret = regmap_write(priv->ser_regmap[channel], 0x07, priv->clkout_ctrl2[channel]);
	if (ret)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, 0x07, priv->clkout_ctrl2[channel]);


	for (reg = fpdlink_ser_reglist; reg->addr != 0; reg++) {
		int err;
		dev_dbg(&client->dev, "Writing ser: 0x%x = 0x%x\n", reg->addr, reg->val);
		err = regmap_write(priv->ser_regmap[channel], reg->addr, reg->val);
		if (err)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, reg->addr, reg->val);
	}

	ret = regmap_write(priv->ser_regmap[channel], 0x02, (priv->mipi_lane_count[channel]-1)<<4 | 0x3);

	for( i = 0; i < MAX_BACKCHANNEL_SLOTS; ++i )
	{
		ret = tis_fpdlink_init_serializer_backchannel(&client->dev, priv->ser_regmap[channel], &priv->backchannels[channel][i]);
	}

	// for (reg = fpdlink_ser_patgen; reg->addr != 0; reg++) {
	// 	int err;
	// 	dev_dbg(&client->dev, "Writing ser: 0x%x = 0x%x\n", reg->addr, reg->val);
	// 	err = regmap_write(priv->ser_regmap, reg->addr, reg->val);
	// 	if (err)
	// 		pr_err("%s:i2c write failed, %x = %x\n",
	// 			__func__, reg->addr, reg->val);
	// }

	ret = tis_fpdlink_init_gpio(priv, channel);

	return ret;
}

static int tis_fpdlink_remove_serializer(struct tis_fpdlink *priv, int channel)
{
	// Reset n_mappings to its state after tis_fpdlink_probe
	priv->n_mappings[channel] = priv->slave_count[channel];

	if( priv->gpio_client[channel] )
	{
		i2c_unregister_device(priv->gpio_client[channel]);
		priv->gpio_client[channel] = NULL;
	}	

	// GPIO type is unknown again, so that gpio_chip acces doesn't try to use it
	priv->gpio_type[channel] = TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT;

	return 0;
}

/*
	Get lock status of given channel
*/
static int inline tis_fpdlink_get_lock_sts(struct tis_fpdlink *priv, int channel, int *lock_sts, int *lock_sts_chg)
{
	int err;
	unsigned int val;

	err = regmap_read(priv->deser_chan_regmap[channel], 0x4d, &val);
	if (err)
	{
		pr_err("%s:i2c read failed\n", __func__);
	}
	else
	{
		*lock_sts = val & (1<<0);
		*lock_sts_chg = val & (1<<4);
	}

	return err;
}


/*
	Worker function to check for lock on all configured channels
*/
static void tis_fpdlink_check_link_func(struct work_struct *ws)
{
	struct i2c_client *client;
	struct tis_fpdlink *priv;
	int i;
	int rt = HZ * 2;
	int err;
	int lock_sts, lock_sts_chg;

	priv = (struct tis_fpdlink *)container_of((struct delayed_work *)ws,
		struct tis_fpdlink,	detect_link_work);
	client = priv->deser_client;

	for(i=0; i < priv->n_busses; i++)
	{
		err = tis_fpdlink_get_lock_sts(priv, i, &lock_sts, &lock_sts_chg);
		if( err )
		{
			dev_err(&client->dev, "%s: tis_fpdlink_get_lock_sts failed: %d", __func__, err);
			continue;
		}

		if( !lock_sts && priv->ser_lock[i] )
		{
			if( !--priv->ser_lock[i] )
			{
				dev_err(&client->dev, "LOST LOCK on channel %d\n", i);
				tis_fpdlink_remove_serializer(priv, i);				
			}
		}
		else if( lock_sts )
		{
			if( !priv->ser_lock[i] )
			{
				dev_dbg(&client->dev, "Got LOCK\n");
				tis_fpdlink_init_serializer(client, i);
			}

			priv->ser_lock[i] = 2;
		}

		if( !lock_sts )
			rt = HZ/2;
	}

	schedule_delayed_work(&priv->detect_link_work, rt);
}

static int tis_fpdlink_map_i2c(struct tis_fpdlink *priv, unsigned int channel,
                               u8 src_addr, u8 map_addr)
{
	dev_dbg(&priv->deser_client->dev, "map[%d] %d: %x -> %x\n", channel, priv->n_mappings[channel], src_addr, map_addr);

	regmap_write(priv->deser_chan_regmap[channel], TIS_FPDLINK_UB954_SLAVE_ID_BASE + priv->n_mappings[channel],
		src_addr<<1);
	regmap_write(priv->deser_chan_regmap[channel],
		TIS_FPDLINK_UB954_SLAVE_ALIAS_BASE + priv->n_mappings[channel],
		map_addr<<1);
	return 0;
}

static int tis_fpdlink_add_gpiochip_once(struct tis_fpdlink *priv)
{
	int ret;
	struct device* dev = &priv->deser_client->dev;
	int i;

	if( priv->gpio_chip.gpiodev )
		return 0;

	priv->gpio_chip = tis_fpdlink_gpio_chip;
	priv->gpio_chip.parent = dev;
	priv->gpio_chip.ngpio = priv->n_busses * 8;
	dev_dbg(dev, "ngpio: %d\n", priv->gpio_chip.ngpio);
	ret = gpiochip_add_data(&priv->gpio_chip, priv);
	if (ret < 0)
	{
		dev_err(dev, "Failed to add GPIO chip\n");
		return ret;
	}

	for (i = 0; i < priv->n_busses; ++i)
	{
		if( priv->gpio_type[i] == TIS_FPDLINK_GPIO_TYPE_INITIALIZING )
		{
			priv->gpio_type[i] = TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT;
		}
	}

	return 0;
}

static int tis_fpdlink_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tis_fpdlink *priv;
	const struct fpdlink_reg *reg;
	int i;
	int ret;
	int err;
	unsigned int sts;
	int retry_count;

	dev_info(&client->dev, "Probing TIS_FPDLink, driver ver %s\n", __stringify(TIS_DRIVERS_VERSION));

	priv = devm_kzalloc(&client->dev,
			sizeof(struct tis_fpdlink),
			GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	mutex_init(&priv->i2c_mutex);

	priv->deser_client = client;

	ret = tis_fpdlink_parse_dt(client, priv);
	if (ret)
		return ret;

	priv->deser_regmap = devm_regmap_init_i2c(client, &tis_fpdlink_regmap_config);
	if (IS_ERR(priv->deser_regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->deser_regmap));
		return -ENODEV;
	}

	i2c_set_clientdata(client, priv);

	dev_dbg(&client->dev, "Initializing DESERIALIZER with %d channels\n", priv->n_busses);

	// digital reset
	err = regmap_write(priv->deser_regmap, 0x01, 0x06);
	if (err) {
		dev_err(&client->dev, "i2c write failed, %x = %x\n",
				0x01, 0x06);
		return -EIO;
	}

	err = regmap_read(priv->deser_regmap, 0x04, &sts);
	if (err) {
		dev_err(&client->dev, "i2c read failed\n");
		return -EIO;
	}
	retry_count = 0;
	while (! (sts & (1<<6))) {
		err = regmap_read(priv->deser_regmap, 0x04, &sts);
		if (err) {
			dev_err(&client->dev, "i2c read failed\n");
			return -EIO;
		}
		if (retry_count++ > 20)
		{
			dev_err(&client->dev, "device fails to leave reset!\n");
			return -EIO;
		}
	}

	for (reg = fpdlink_deser_reglist; reg->addr != 0; reg++) {
		dev_dbg(&client->dev, "Writing deser: 0x%x = 0x%x\n", reg->addr, reg->val);
		err = regmap_write(priv->deser_regmap, reg->addr, reg->val);
		if (err) {
			dev_err(&client->dev, "i2c write failed\n");
			return -EIO;
		}
		if (reg->flags & TIS_FPDLINK_FLAG_VERIFY)
		{
			for (retry_count = 0; retry_count < 5; retry_count++)
			{
				unsigned int val;
				int err;
				err = regmap_read(priv->deser_regmap, reg->addr, &val);
				if (err) {
					dev_err(&client->dev, "i2c read failed\n");
					return -EIO;
				}
				if (val != reg->val){
					err = regmap_write(priv->deser_regmap, reg->addr, reg->val);
					if (err) {
						dev_err(&client->dev, "i2c write failed\n");
						return -EIO;
					}
				} else {
					break;
				}
				usleep_range(30000, 31000);
			}
			if (retry_count == 5)
			{
				dev_dbg(&client->dev, "Repeatedly failed to set register!\n");
			}
		}
	}

	for( i = 0; i < priv->n_busses; ++i )
	{
		int gpio_index = priv->coax_power_gpio[i];

		if( gpio_index <= 6 )
		{
			// Enable coax power
			regmap_write(priv->deser_regmap, 0x10 + gpio_index, 0x13); // GPIOx_PIN_CTL
			if (err) {
				dev_err(&client->dev, "i2c write failed\n");
				return -EIO;
			}
		}
	}

	// CSI_CTL
	// Bit 6: Enable CSI Skew-Calibration
	// 5:4  : CSI lane count (00 = 4 lane, 01= 3 lane, 10 = 2 lane, 11 = 1 lane)
	// 0    : CSI_ENABLE
	err = regmap_write(priv->deser_regmap, 0x33, (1<<6) | ((4-priv->deser_lane_count)<<4) | 1);
	if (err) {
		dev_err(&client->dev, "i2c write failed\n");
		return -EIO;
	}

	// RX_PORT_CTL:
	// Enable Port PASS on all enabled receiver ports
	// Enable Port LOCK on all enabled receiver ports
	err = regmap_write(priv->deser_regmap, 0x0c,
		((priv->n_busses == 1) ? 1:3) |
		(3 << 2) |
		(3 << 4));
	if (err) {
		dev_err(&client->dev, "i2c write failed\n");
		return -EIO;
	}

	err = regmap_write(priv->deser_regmap, 0xF8, priv->deser_chan_addr[0] << 1); // I2C_RX0_ID
	if (err) {
		dev_err(&client->dev, "i2c write failed\n");
		return -EIO;
	}
	err = regmap_write(priv->deser_regmap, 0xF9, priv->deser_chan_addr[1] << 1); // I2C_RX1_ID
	if (err) {
		dev_err(&client->dev, "i2c write failed\n");
		return -EIO;
	}

	for (i = 0; i < priv->n_busses; ++i) {

		 // I2C_RX0_ID or I2C_RX1_ID as programmed above
		priv->deser_chan_client[i] = i2c_new_dummy(client->adapter, priv->deser_chan_addr[i]);
		if(!priv->deser_chan_client[i])
		{
			dev_err(&client->dev, "Instantiation of per-channel deserializer i2c device failed\n");
			return -ENODEV;
		}

		priv->deser_chan_regmap[i] = devm_regmap_init_i2c(priv->deser_chan_client[i], &tis_fpdlink_regmap_config);
		if (IS_ERR(priv->deser_chan_regmap[i]))
		{
			dev_err(&priv->deser_chan_client[i]->dev,
				"deser_chan_regmap init failed: %ld\n", PTR_ERR(priv->deser_chan_regmap[i]));
			return -ENODEV;
		}
	}

	for (i = 0; i < priv->n_busses; i++) {
		int j;
		// Set up serializer I2C address mapping
		dev_dbg(&client->dev, "Mapping serializer channel %d to 0x%x\n", i, priv->ser_map_addr[i]);

		err = regmap_write(priv->deser_chan_regmap[i], 0x5c, priv->ser_map_addr[i] << 1);
		if (err)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, 0x5c, priv->ser_map_addr[i] << 1);

		for (j = 0; j < priv->slave_count[i]; j++){
			tis_fpdlink_map_i2c(priv, i, priv->slave_addr[i][j], priv->slave_map_addr[i][j]);
			priv->n_mappings[i]++;
		}

		for (reg = fpdlink_deser_chan_reglist; reg->addr != 0; reg++) {
			err = regmap_write(priv->deser_chan_regmap[i], reg->addr, reg->val);
			if (err) {
				dev_err(&client->dev, "i2c write failed\n");
				return -EIO;
			}
		}
		// CSI_VC_MAP
		dev_dbg(&client->dev, "vc map %i: 0x%x\n", i, priv->vc_map[i]);
		err = regmap_write(priv->deser_chan_regmap[i], 0x72, priv->vc_map[i]);
		if (err) {
			dev_err(&client->dev, "i2c write failed\n");
			return -EIO;
		}

		for( j = 0; j < MAX_BACKCHANNEL_SLOTS; ++j )
		{
			ret = tis_fpdlink_init_deserializer_backchannel(&client->dev, priv->deser_chan_regmap[i], &priv->backchannels[i][j], i);
		}
	}	

	priv->fpdlink_dir = tis_fpdlink_debug_create(client->addr, priv);

	// This is nasty, but we have to give FPD-Link some time to find its lock
	msleep(1000);

	INIT_DELAYED_WORK(&priv->detect_link_work, tis_fpdlink_check_link_func);
	tis_fpdlink_check_link_func(&priv->detect_link_work.work);

	tis_fpdlink_add_gpiochip_once(priv);

	return 0;
}

static int tis_fpdlink_remove(struct i2c_client *client)
{
	struct tis_fpdlink *priv;
	int i;

	dev_dbg(&client->dev, "Removing TIS_FPDLink\n");

	priv = (struct tis_fpdlink *)i2c_get_clientdata(client);

	if (priv->fpdlink_dir)
		debugfs_remove_recursive(priv->fpdlink_dir);

	cancel_delayed_work_sync(&priv->detect_link_work);
	//flush_scheduled_work();


	gpiochip_remove(&priv->gpio_chip);

	for (i = 0; i < priv->n_busses; i++) {
		int gpio_index;

		if (priv->gpio_client[i])
			i2c_unregister_device(priv->gpio_client[i]);

		if (priv->ser_client[i])
			i2c_unregister_device(priv->ser_client[i]);

		if (priv->deser_chan_client[i])
			i2c_unregister_device(priv->deser_chan_client[i]);

		gpio_index = priv->coax_power_gpio[i];

		if( gpio_index <= 6 )
		{
			// Disable Coax Power
			regmap_write(priv->deser_regmap, 0x10 + gpio_index, 0x00); // GPIOx_PIN_CTL
		}
	}
	
	return 0;
}

int tis_fpdlink_configure_serializer_clkout( struct i2c_client* client, int channel, u32 clkout_hz, u32 *actual_clkout_hz )
{
	const u32 REFCLK_HZ = 25000000;

	struct tis_fpdlink *priv = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	u32 actual_hz;

	struct tis_fpdlink_pll_settings pll_settings = {0};
	tis_fpdlink_find_pll_settings(REFCLK_HZ, clkout_hz, &pll_settings);
	priv->clkout_ctrl1[channel] = (pll_settings.hs_clk_div << 5) | pll_settings.m;
	priv->clkout_ctrl2[channel] = pll_settings.n;

	actual_hz = tis_fpdlink_calc_output_clock(REFCLK_HZ, pll_settings.hs_clk_div, pll_settings.m, pll_settings.n);

	dev_info(dev, "Reconfiguring clock, request %d Hz, configure %02x %02x (%d Hz)", clkout_hz, priv->clkout_ctrl1[channel], priv->clkout_ctrl2[channel], actual_hz);

	regmap_write(priv->ser_regmap[channel], 0x06, priv->clkout_ctrl1[channel]);
	regmap_write(priv->ser_regmap[channel], 0x07, priv->clkout_ctrl2[channel]);

	if( actual_clkout_hz )
		*actual_clkout_hz = actual_hz;	
	
	return 0;
}
EXPORT_SYMBOL(tis_fpdlink_configure_serializer_clkout);

int tis_fpdlink_configure_serializer_csi_lanes( struct i2c_client* client, int channel, int num_serializer_csi_lanes )
{
	struct tis_fpdlink *priv = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	priv->mipi_lane_count[channel] = num_serializer_csi_lanes;

	dev_info(dev, "Reconfiguring serializer CSI lanes = %d", num_serializer_csi_lanes);

	return regmap_write(priv->ser_regmap[channel], 0x02, (priv->mipi_lane_count[channel]-1)<<4 | 0x3);
}
EXPORT_SYMBOL(tis_fpdlink_configure_serializer_csi_lanes);

bool tis_fpdlink_is_serializer_connected(struct i2c_client *client, int channel)
{
	struct tis_fpdlink *priv = i2c_get_clientdata(client);

	return (priv->ser_lock[channel] > 0);
}
EXPORT_SYMBOL(tis_fpdlink_is_serializer_connected);

static int regmap_read_retry(struct device *dev, struct regmap *map, unsigned int reg, unsigned int *val);
static int regmap_write_retry(struct device *dev, struct regmap *map, unsigned int reg, unsigned int val);
static int regmap_rmw(struct device *dev, struct regmap *map, unsigned int reg, unsigned int clear_mask, unsigned int set_mask);

static int tis_fpdlink_tca7408_get(struct tis_fpdlink *priv, unsigned int offset)
{
    int ret;
	unsigned int input_status;
    int channel = (offset >> 3);
	struct regmap *gpio_regmap = priv->gpio_regmap[channel];
	struct device *dev = &priv->ser_client[channel]->dev;
    offset = offset & 7;

	ret = regmap_rmw(dev, gpio_regmap, TCA7408_OUTPUT_IMPEDANCE, 0, (1 << offset));
    if (ret < 0)
    {
		dev_err(dev, "%s: regmap_rmw(output_impedance) failed: %d \n", __func__, ret);
		return ret;
    }
	ret = regmap_rmw(dev, gpio_regmap, TCA7408_DIRECTION, (1 << offset), 0);
	if (ret < 0)
    {
		dev_err(dev, "%s: regmap_rmw(direction) failed: %d \n", __func__, ret);
		return ret;
    }

	ret = regmap_read_retry(dev, gpio_regmap, TCA7408_INPUT_STATUS, &input_status);
    if (ret < 0){
		dev_err(dev, "%s: regmap_read_retry(input_status) failed: %d\n", __func__, ret);
        return ret;
    }

    return (input_status >> offset) & 0x1;
}

static int tis_fpdlink_tca7408_set(struct tis_fpdlink *priv, int offset, int val)
{
    int ret;
    int channel = (offset >> 3);
	struct regmap *gpio_regmap = priv->gpio_regmap[channel];
	struct device *dev = &priv->ser_client[channel]->dev;
    offset = offset & 7;

	ret = regmap_rmw(dev, gpio_regmap, TCA7408_OUTPUT_IMPEDANCE, (1 << offset), 0);
    if (ret < 0)
    {
		dev_err(dev, "%s: regmap_rmw(output_impedance) failed: %d \n", __func__, ret);
		return ret;
    }
	ret = regmap_rmw(dev, gpio_regmap, TCA7408_DIRECTION, 0, (1 << offset));
	if (ret < 0)
    {
		dev_err(dev, "%s: regmap_rmw(direction) failed: %d \n", __func__, ret);
		return ret;
    }

	ret = regmap_rmw(dev, gpio_regmap, TCA7408_OUTPUT, val ? 0 : (1 << offset), val ? (1 << offset) : 0);
    if (ret < 0 )
    {
		dev_err(dev, "%s: regmap_rmw(output) failed (%d)\n", __func__, ret);
    }

	return ret;
}

static int tis_fpdlink_tca7408_get_output(struct tis_fpdlink *priv, int offset, bool *status)
{
	int channel = (offset >> 3);
	int gpio_offset = offset & 7;
    struct device *dev = &priv->ser_client[channel]->dev;
	struct regmap *gpio_regmap = priv->gpio_regmap[channel];    
	unsigned int val;	

	int err = regmap_read(gpio_regmap, TCA7408_OUTPUT, &val);
	if( err < 0 )
	{
        dev_err(dev, "%s: Failed to read gpio out\n", __func__);
		return err;		
	}

	*status = (val & (1 << gpio_offset));

	return 0;
}

static int regmap_read_retry(struct device *dev, struct regmap *map, unsigned int reg, unsigned int *val)
{
	int retry = 10;

	while( true )
	{
		int ret = regmap_read(map, reg, val);
		if( ret < 0 && retry-- > 0)
		{
			dev_err(dev, "%s: regmap_read(0x%x) failed (%d), retries = %d\n", __func__, reg, ret, retry);
			usleep_range(20000, 20020);
		}
		else
		{
			return ret;
		}
	}
}

static int regmap_write_retry(struct device *dev, struct regmap *map, unsigned int reg, unsigned int val)
{
	int retry = 10;

	while( true )
	{
		int ret = regmap_write(map, reg, val);
		if( ret < 0 && retry-- > 0)
		{
			dev_err(dev, "%s: regmap_write(0x%x, 0x%x) failed (%d), retries = %d\n", __func__, reg, val, ret, retry);
			usleep_range(20000, 20020);
		}
		else
		{
			return ret;
		}
	}
}

static int regmap_rmw(struct device *dev, struct regmap *map, unsigned int reg, unsigned int clear_mask, unsigned int set_mask)
{
	unsigned int prev_val;
	unsigned int new_val;

	int ret = regmap_read_retry(dev, map, reg, &prev_val);
    if (ret<0)
    {
        dev_err(dev, "%s: regmap_read_retry(0x%x) failed (%d)\n", __func__, reg, ret);
        return ret;
    }

	new_val = (prev_val & ~clear_mask) | set_mask;

	if( new_val != prev_val )
	{
		ret = regmap_write_retry(dev, map, reg, new_val);
        if (ret<0)
        {
            dev_err(dev, "%s: regmap_write_retry(0x%x, 0x%x) failed (%d)\n", __func__, reg, new_val, ret);
        }
	}

	return ret;
}

static int tis_fpdlink_tca6408_set(struct tis_fpdlink *priv, int offset, int val)
{
	int ret;
	int channel = (offset >> 3);
    struct device *dev = &priv->ser_client[channel]->dev;
	struct regmap *gpio_regmap = priv->gpio_regmap[channel];
    offset = offset & 7;

	ret = regmap_rmw(dev, gpio_regmap, 0x03, (1 << offset), 0);
    if (ret<0)
    {
        dev_err(dev, "%s: regmap_rmw(0x03, 0, 0x%x) failed (%d)\n", __func__, (1 << offset), ret);
        return ret;
    }

	ret = regmap_rmw(dev, gpio_regmap, 0x01, val ? 0 : (1 << offset), val ? (1 << offset) : 0);
    if (ret < 0 )
    {
		dev_err(dev, "%s: regmap_rmw(0x01, ...) failed (%d)\n", __func__, ret);
    }

    return ret;
}

static int tis_fpdlink_tca6408_get(struct tis_fpdlink *priv, unsigned int offset)
{
    int ret;
	unsigned int val;
    int channel = (offset >> 3);
	struct device *dev = &priv->ser_client[channel]->dev;
    offset = offset & 7;

	ret = regmap_read_retry(dev, priv->gpio_regmap[channel], 0x0, &val);
    if (ret<0)
    {
        dev_err(dev, "%s: Failed to read from gpio\n", __func__);
        return ret;
    }

    return ((val>>offset) & 1);
}

static int tis_fpdlink_tca6408_get_output(struct tis_fpdlink *priv, int offset, bool *status)
{
	int channel = (offset >> 3);
	int gpio_offset = offset & 7;
    struct device *dev = &priv->ser_client[channel]->dev;
	struct regmap *gpio_regmap = priv->gpio_regmap[channel];    
	unsigned int val;	

	int err = regmap_read(gpio_regmap, 0x01, &val);
	if( err < 0 )
	{
        dev_err(dev, "%s: Failed to read gpio out\n", __func__);
		return err;		
	}

	*status = (val & (1 << gpio_offset));

	return 0;
}

static void tis_fpdlink_warn_gpio_not_decided(struct tis_fpdlink *priv, const char *func, const char *action, int gpio_type)
{
	u64 now = get_jiffies_64();

	u64 dt = now - priv->prev_gpio_warn;

	if (dt > msecs_to_jiffies(1000))
	{
		dev_warn(&priv->deser_client->dev, "%s: gpio_type still undecided (%d), won't %s!", func, gpio_type, action);
		priv->prev_gpio_warn = now;
	}
}

static int tis_fpdlink_direction_output(struct gpio_chip *gpio, unsigned int offset, int val)
{
	unsigned int channel = (offset >> 3);
	struct tis_fpdlink *priv = container_of(gpio, struct tis_fpdlink, gpio_chip);
    struct device *dev = &priv->deser_client->dev;

    if (channel > 1)
    {
		dev_err(dev, "%s: invalid channel %d, offset= %d\n", __func__, channel, offset);
    	return -EINVAL;
    }

	//pr_info("%s: channel %d, offset= %d, val=%d\n", __func__, channel, offset, val);

    switch(priv->gpio_type[channel])
    {
        case TIS_FPDLINK_GPIO_TYPE_INITIALIZING:
            tis_fpdlink_warn_gpio_not_decided(priv, __func__, "toggle", priv->gpio_type[channel]);
            return 0; // Don't fail if we want the gpio_chip creation to go through
        case TIS_FPDLINK_GPIO_TYPE_TCA6408:
            return tis_fpdlink_tca6408_set(priv, offset, val);
        case TIS_FPDLINK_GPIO_TYPE_TCA7408:
            return tis_fpdlink_tca7408_set(priv, offset, val);
        case TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT:
            // we only know the gpio type after the serializer got connected
            tis_fpdlink_warn_gpio_not_decided(priv, __func__, "toggle", priv->gpio_type[channel]);
            return -ENODEV;
        default:
            dev_err(dev, "gpio type not implemented: %d channel = %d offset= %d\n", priv->gpio_type[channel], channel, offset);
            return -EINVAL;
    }
    return 0;
}

static int tis_fpdlink_direction_input(struct gpio_chip *gpio, unsigned int offset)
{
    struct tis_fpdlink *priv = container_of(gpio, struct tis_fpdlink, gpio_chip);
    struct device *dev = &priv->deser_client->dev;
    int ret = 0;
    int channel = (offset >> 3);

    switch(priv->gpio_type[channel])
    {
        case TIS_FPDLINK_GPIO_TYPE_INITIALIZING:
			tis_fpdlink_warn_gpio_not_decided(priv, __func__, "read", priv->gpio_type[channel]);
        	return -ENODEV;
        case TIS_FPDLINK_GPIO_TYPE_TCA6408:
            ret = tis_fpdlink_tca6408_get(priv, offset);
            break;
        case TIS_FPDLINK_GPIO_TYPE_TCA7408:
            ret = tis_fpdlink_tca7408_get(priv, offset);
            break;
        case TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT:
        	// we only know the gpio type after the serializer got connected
			tis_fpdlink_warn_gpio_not_decided(priv, __func__, "read", priv->gpio_type[channel]);
        	return -ENODEV;
        default:
            dev_err(dev, "gpio type '0x%x' not implemented\n", priv->gpio_type[channel]);
            return -EINVAL;
    }

    return ret;
}

static int tis_fpdlink_gpio_get(struct gpio_chip *gpio, unsigned int offset)
{
    struct tis_fpdlink *priv = container_of(gpio, struct tis_fpdlink, gpio_chip);
    struct device *dev = &priv->deser_client->dev;
    int ret = 0;
    int channel = (offset >> 3);

    switch(priv->gpio_type[channel])
    {
        case TIS_FPDLINK_GPIO_TYPE_INITIALIZING:
			tis_fpdlink_warn_gpio_not_decided(priv, __func__, "read", priv->gpio_type[channel]);
        	return 0; // Don't fail if we want the gpio_chip creation to go through
        case TIS_FPDLINK_GPIO_TYPE_TCA6408:
            ret = tis_fpdlink_tca6408_get(priv, offset);
            break;
        case TIS_FPDLINK_GPIO_TYPE_TCA7408:
            ret = tis_fpdlink_tca7408_get(priv, offset);
            break;
        case TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT:
        	// we only know the gpio type after the serializer got connected
			tis_fpdlink_warn_gpio_not_decided(priv, __func__, "read", priv->gpio_type[channel]);
        	return -ENODEV;
        default:
            dev_err(dev, "gpio type not implemented\n");
            return -EINVAL;
    }

    return ret;
}
static void tis_fpdlink_gpio_set(struct gpio_chip *gpio, unsigned int offset, int val)
{
    struct tis_fpdlink *priv = container_of(gpio, struct tis_fpdlink, gpio_chip);
    int channel = (offset >> 3);
    struct device *dev = &priv->deser_client->dev;

	dev_dbg(dev, "%s: channel %d, offset= %d, val= %d\n", __func__, channel, offset, val);

    switch(priv->gpio_type[channel])
    {
        case TIS_FPDLINK_GPIO_TYPE_TCA6408:
            tis_fpdlink_tca6408_set(priv, offset, val);
            break;
        case TIS_FPDLINK_GPIO_TYPE_TCA7408:
            tis_fpdlink_tca7408_set(priv, offset, val);
            break;
        case TIS_FPDLINK_GPIO_TYPE_INITIALIZING:
        case TIS_FPDLINK_GPIO_TYPE_NOT_PRESENT:
        	// we only know the gpio type after the serializer got connected
            tis_fpdlink_warn_gpio_not_decided(priv, __func__, "toggle", priv->gpio_type[channel]);
        	break;
        default:
            dev_err(dev, "gpio type not implemented\n");
            break;
    }
}

int tis_fpdlink_get_gpio_out_status(struct i2c_client *client, int channel, int gpio, bool *status)
{
	struct tis_fpdlink *priv = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	int offset = gpio - priv->gpio_chip.base;

	if( offset < 0 || offset >= priv->gpio_chip.ngpio )
	{
		dev_err(dev, "%s: unexpected offset %d (channel = %d gpio = %d, chip.base = %d)", __func__, offset, channel, gpio, priv->gpio_chip.base);
		return -EINVAL;
	}

    switch(priv->gpio_type[channel])
	{
        case TIS_FPDLINK_GPIO_TYPE_TCA6408:
			return tis_fpdlink_tca6408_get_output(priv, offset, status);
        case TIS_FPDLINK_GPIO_TYPE_TCA7408:
			return tis_fpdlink_tca7408_get_output(priv, offset, status);
		default:
			dev_warn(dev, "%s: Call unexpected, GPIO type not decided", __func__);
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tis_fpdlink_get_gpio_out_status);

static const struct i2c_device_id tis_fpdlink_id[] = {
	{ "tis-fpdlink", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tis_fpdlink_id);

/*
* I2C related structure
*/
static struct i2c_driver tis_fpdlink_i2c_driver = {
	.driver = {
		.name = "tis-fpdlink",
		.owner = THIS_MODULE,
		.of_match_table = tis_fpdlink_of_match,
	},
	.probe = tis_fpdlink_probe,
	.remove = tis_fpdlink_remove,
	.id_table = tis_fpdlink_id,
};

module_i2c_driver(tis_fpdlink_i2c_driver);

MODULE_DESCRIPTION("FDP Link driver for The Imaging Source FPD Link camera connector");
MODULE_AUTHOR("The Imaging Source Europe GmbH");
MODULE_LICENSE("GPL");
