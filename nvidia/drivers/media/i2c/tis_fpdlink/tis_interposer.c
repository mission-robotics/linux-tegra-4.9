/*
 * The Imaging Source interposer board driver
 *
 * Copyright (C) 2019 The Imaging Source Europe GmbH
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>


#include "Si5356-24MHz-Registers.h"
#include "Si5356-27MHz-Registers.h"
#include "Si5356-37MHz-Registers.h"

#define TIS_INTERP_GPIO_TYPE_INITIALIZING           (0)
#define TIS_INTERP_GPIO_TYPE_TCA7408                (1)
#define TIS_INTERP_GPIO_TYPE_TCA6408                (2)
#define TIS_INTERP_GPIO_TYPE_NOT_PRESENT            (3)


struct tis_interposer {
    struct i2c_client *i2c_client;
    int parent;
    int gpio_type;
    u32 clock_khz;

    struct i2c_client *gpio_client;
    struct i2c_client *clock_client;

    struct gpio_chip gpio_chip;
};

static int tis_interposer_direction_output(struct gpio_chip *gpio, unsigned int offset, int val);
static int tis_interposer_direction_input(struct gpio_chip *gpio, unsigned int offset);
static int tis_interposer_gpio_get(struct gpio_chip *gpio, unsigned int offset);
static void tis_interposer_gpio_set(struct gpio_chip *gpio, unsigned int offset, int val);


static const struct gpio_chip tis_interposer_gpio_chip = {
    .label          = "gpio-tis-interposer",
    .owner          = THIS_MODULE,
    .direction_input    = tis_interposer_direction_input,
    .direction_output   = tis_interposer_direction_output,
    .set            = tis_interposer_gpio_set,
    .get            = tis_interposer_gpio_get,
    .base           = -1,
    .ngpio          = 8,
    .can_sleep      = true,
};


static int tis_interposer_probe_dt(struct i2c_client *client,
    struct tis_interposer *interp)
{
	struct device_node *np = client->dev.of_node;
    struct property *prop;
	// struct device_node *adapter_np;
	// struct i2c_adapter *adapter;
    int len;

	if (!np)
		return -ENODEV;

	// adapter_np = of_parse_phandle(np, "i2c-parent", 0);
	// if (!adapter_np) {
	// 	dev_err(&client->dev, "Cannot parse i2c-parent\n");
	// 	return -ENODEV;
	// }
	// adapter = of_find_i2c_adapter_by_node(adapter_np);
	// of_node_put(adapter_np);
	// if (!adapter)
	// 	return -EPROBE_DEFER;

	// interp->parent = i2c_adapter_id(adapter);
	// put_device(&adapter->dev);

    prop = of_find_property(np, "clock_khz", &len);
    if (prop)
    {
        of_property_read_u32(np, "clock_khz", &interp->clock_khz);
    }

    return 0;
}


static int tis_interposer_si5356_set_clock(struct tis_interposer *interp, u32 clock)
{
    int i;
    int ret = 0;
    const Reg_Data *regs;

    switch(clock)
    {
        case 24000:
            regs = Reg_Store_24000;
            break;
        case 27000:
            regs = Reg_Store_27000;
            break;
        case 37000:
            regs = Reg_Store_37000;
            break;
        default:
            dev_err(&interp->i2c_client->dev, "invalid clock setting\n");
            return -EINVAL;
    }

    i2c_smbus_write_byte_data(interp->clock_client, 230, 0x10); // OEB_ALL = 1
    i2c_smbus_write_byte_data(interp->clock_client, 241, 0x65); // DIS_LOL = 1

    for(i=0; i < NUM_REGS_MAX; i++)
    {
        u8 addr, value, mask;
        addr = regs[i].Reg_Addr;
        value = regs[i].Reg_Val;
        mask = regs[i].Reg_Mask;

        if (mask == 0){
            continue;
        }
        if (mask != 0xff)
        {
            u8 tmp;

            tmp = i2c_smbus_read_byte_data(interp->clock_client, addr);
            if (tmp < 0){
                dev_err(&interp->i2c_client->dev, "failed to read from clockchip\n");
                return -EINVAL;
            }
            tmp = tmp & (~mask);
            value = value & mask;
            value = tmp | value;
        }
        dev_dbg(&interp->i2c_client->dev, "0x%x = 0x%x\n", addr, value);
        ret = i2c_smbus_write_byte_data(interp->clock_client, addr, value);
        if (ret < 0)
        {
            dev_err(&interp->i2c_client->dev, "failed to write to clockchip\n");
            return -EINVAL;
        }
    }

    i2c_smbus_write_byte_data(interp->clock_client, 246, 2);
    i2c_smbus_write_byte_data(interp->clock_client, 226, 4);
    i2c_smbus_write_byte_data(interp->clock_client, 226, 0);
    i2c_smbus_write_byte_data(interp->clock_client, 230, 0);

    return 0;
}


static int tis_interposer_tca6408_set(struct tis_interposer *interp, int offset, int val)
{
    s32 ret;
    u8 cfg;
    u8 state;

    ret = i2c_smbus_read_byte_data(interp->gpio_client, 0x3);
    if (ret<0)
    {
        dev_err(&interp->i2c_client->dev, "Failed to read from gpio\n");
        return ret;
    }
    cfg = ret & 0xff;
    if (cfg & (1<<offset))
    {
        cfg &= ~(1<<offset);
        ret = i2c_smbus_write_byte_data(interp->gpio_client, 0x3, cfg);
        if (ret<0)
        {
            dev_err(&interp->i2c_client->dev, "Failed to write to gpio\n");
            return ret;
        }
    }

    ret = i2c_smbus_read_byte_data(interp->gpio_client, 0x1);
    if (ret<0)
    {
        dev_err(&interp->i2c_client->dev, "Failed to read from gpio\n");
        return ret;
    }
    state = ret & 0xff;
    if (val)
    {
        state |= (1<<offset);
    } else {
        state &= ~(1<<offset);
    }
    ret = i2c_smbus_write_byte_data(interp->gpio_client, 0x1, state);

    return ret;
}

static int tis_interposer_tca6408_get(struct tis_interposer *interp, unsigned int offset)
{
    s32 ret;
    u8 state;

    ret = i2c_smbus_read_byte_data(interp->gpio_client, 0x1);
    if (ret<0)
    {
        dev_err(&interp->i2c_client->dev, "Failed to read from gpio\n");
        return ret;
    }
    state = ret & 0xff;
    return ((state>>offset) & 1);
}

static int tis_interposer_direction_output(struct gpio_chip *gpio, unsigned int offset, int val)
{
    struct tis_interposer *interp = container_of(gpio, struct tis_interposer, gpio_chip);
    switch(interp->gpio_type)
    {
        case TIS_INTERP_GPIO_TYPE_INITIALIZING:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not decided yet\n", __func__);
            return 0; // Don't fail if we want the gpio_chip creation to go through

        case TIS_INTERP_GPIO_TYPE_TCA6408:
            tis_interposer_tca6408_set(interp, offset, val);
            break;

        case TIS_INTERP_GPIO_TYPE_NOT_PRESENT:
            dev_err(&interp->i2c_client->dev, "%s: device not present\n", __func__);
            return -ENODEV;

        default:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not implemented\n", __func__);
            return -EINVAL;
    }
    return 0;
}

static int tis_interposer_direction_input(struct gpio_chip *gpio, unsigned int offset)
{
    struct tis_interposer *interp = container_of(gpio, struct tis_interposer, gpio_chip);
    int ret;
    switch(interp->gpio_type)
    {
        case TIS_INTERP_GPIO_TYPE_INITIALIZING:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not decided yet\n", __func__);
            return 0; // Don't fail if we want the gpio_chip creation to go through

        case TIS_INTERP_GPIO_TYPE_TCA6408:
            ret = tis_interposer_tca6408_get(interp, offset);
            break;

        case TIS_INTERP_GPIO_TYPE_NOT_PRESENT:
            dev_err(&interp->i2c_client->dev, "%s: device not present\n", __func__);
            return -ENODEV;

        default:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not implemented\n", __func__);
            return -EINVAL;
    }

    return ret;
}

static int tis_interposer_gpio_get(struct gpio_chip *gpio, unsigned int offset)
{
    struct tis_interposer *interp = container_of(gpio, struct tis_interposer, gpio_chip);
    int ret;

    switch(interp->gpio_type)
    {
        case TIS_INTERP_GPIO_TYPE_INITIALIZING:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not decided yet\n", __func__);
            return 0; // Don't fail if we want the gpio_chip creation to go through

        case TIS_INTERP_GPIO_TYPE_TCA6408:
            ret = tis_interposer_tca6408_get(interp, offset);
            break;

        case TIS_INTERP_GPIO_TYPE_NOT_PRESENT:
            dev_err(&interp->i2c_client->dev, "%s: device not present\n", __func__);
            return -ENODEV;

        default:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not implemented\n", __func__);
            return -EINVAL;
    }

    return ret;
}
static void tis_interposer_gpio_set(struct gpio_chip *gpio, unsigned int offset, int val)
{
    struct tis_interposer *interp = container_of(gpio, struct tis_interposer, gpio_chip);

    switch(interp->gpio_type)
    {
        case TIS_INTERP_GPIO_TYPE_INITIALIZING:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not decided yet\n", __func__);
            break;

        case TIS_INTERP_GPIO_TYPE_TCA6408:
            tis_interposer_tca6408_set(interp, offset, val);
            break;

        case TIS_INTERP_GPIO_TYPE_NOT_PRESENT:
            dev_err(&interp->i2c_client->dev, "%s: device not present\n", __func__);
            break;

        default:
            dev_err(&interp->i2c_client->dev, "%s: gpio type not implemented\n", __func__);
            break;
    }
}


static int tis_interposer_probe_board(struct i2c_adapter *adap,
    struct tis_interposer *interp)
{
    union i2c_smbus_data dummy;
    int ret;
    int detected = 0;


    /*
        TISMIPI_JetsonNano_Adapter_R1_00
        tca6408 GPIO @ 20
        clock chip @ 70
    */
    if (!detected)
    {
        static const u8 gpio_addr = 0x20;
        static const u8 clock_addr = 0x70;

        ret = i2c_smbus_xfer(adap, clock_addr, 0, I2C_SMBUS_READ, 0x0, I2C_SMBUS_BYTE, &dummy);
        if (ret==0)
        {
            dummy.byte = 0x1;
            ret = i2c_smbus_xfer(adap, gpio_addr, 0, I2C_SMBUS_WRITE, 0x1, I2C_SMBUS_BYTE, &dummy);
            ret = i2c_smbus_xfer(adap, gpio_addr, 0, I2C_SMBUS_READ, 0x1, I2C_SMBUS_BYTE, &dummy);
            if (ret == 0)
            {
                dev_info(&interp->i2c_client->dev, "Found tca6408 gpio @20");
                interp->gpio_client = i2c_new_dummy(adap, gpio_addr);
                if (!interp->gpio_client)
                {
                    dev_err(&interp->i2c_client->dev, "Failed to register i2c client\n");
                    return -ENODEV;
                }

                dev_info(&interp->i2c_client->dev, "Found clockchip @70");
                interp->clock_client = i2c_new_dummy(adap, clock_addr);

                interp->gpio_type = TIS_INTERP_GPIO_TYPE_TCA6408;
                detected = 1;
            }
        }
    }

    return ret;
}


static int tis_interposer_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
    struct tis_interposer *interp;
    int ret;

    dev_info(&client->dev, "tis interposer probe, driver ver %s\n", __stringify(TIS_DRIVERS_VERSION));

    interp = devm_kzalloc(&client->dev, sizeof(*interp), GFP_KERNEL);
    if (!interp)
        return -ENOMEM;

    interp->i2c_client = client;
    i2c_set_clientdata(client, interp);

    ret = tis_interposer_probe_dt(client, interp);
    if (ret < 0) {
        dev_err(&client->dev, "Error parsing device tree\n");
        return ret;
    }

    ret = tis_interposer_probe_board(adap, interp);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to probe interposer board -- hardware not connected?");
        dev_info(&client->dev, "Creating dummy gpio_chip anyway\n");

        interp->gpio_chip = tis_interposer_gpio_chip;
        interp->gpio_chip.parent = &client->dev;
        interp->gpio_type = TIS_INTERP_GPIO_TYPE_INITIALIZING;

        ret = gpiochip_add_data(&interp->gpio_chip, interp);
        if (ret < 0) {
            dev_err(&client->dev, "Failed to add GPIO chip");
            return ret;
        }

        interp->gpio_type = TIS_INTERP_GPIO_TYPE_NOT_PRESENT;
        return 0;
    }

    interp->gpio_chip = tis_interposer_gpio_chip;
    interp->gpio_chip.parent = &client->dev;
    ret = gpiochip_add_data(&interp->gpio_chip, interp);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to add GPIO chip");
        return ret;
    }

    if (interp->clock_khz)
    {
        tis_interposer_si5356_set_clock(interp, interp->clock_khz);
    }

    return 0;
}


static int tis_interposer_remove(struct i2c_client *client)
{
    struct tis_interposer *interp = i2c_get_clientdata(client);

    dev_info(&client->dev, "tis-interposer remove\n");

    if (interp->gpio_type)
    {
        gpiochip_remove(&interp->gpio_chip);
    }

    if (interp->gpio_client){
        i2c_unregister_device(interp->gpio_client);
    }

    if (interp->clock_client){
        i2c_unregister_device(interp->clock_client);
    }

	return 0;
}


static const struct i2c_device_id tis_interposer_id[] = {
	{ "tis-interposer", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tis_interposer_id);


static const struct of_device_id tis_interposer_of_match[] = {
	{ .compatible = "tis,tis-interposer", },
	{},
};
MODULE_DEVICE_TABLE(of, tis_interposer_of_match);

static struct i2c_driver tis_interposer_driver = {
	.probe	= tis_interposer_probe,
	.remove	= tis_interposer_remove,
	.id_table = tis_interposer_id,
	.driver	= {
		.name  = "tis-interposer",
        .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tis_interposer_of_match),
	},
};

module_i2c_driver(tis_interposer_driver);

MODULE_DESCRIPTION("The Imaging Source interposer driver");
MODULE_AUTHOR("Arne Caspari <arne.caspari@theimagingsource.com>");
MODULE_LICENSE("GPL");
