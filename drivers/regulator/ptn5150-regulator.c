/*
 * ptn5150-regulator.c - NXP PTN5150 USB-C Regulator Driver
 *
 * Copyright (c) 2021, Connect Tech Inc. All rights reserved.
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

//#define DEBUG

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>

//Registers
#define PTN5150_ID_REG          0x01
#define PTN5150_CTRL_REG        0x02
#define PTN5150_CABLE_INT_REG   0x03
#define PTN5150_CC_STATUS_REG   0x04
#define PTN5150_CON_DET_REG     0x09
#define PTN5150_RESET_REG       0x10
#define PTN5150_INT_MASK_REG    0x18
#define PTN5150_INT_STAT_REG    0x19

#define PTN5150_LIMIT_900MA     900000
#define PTN5150_LIMIT_1500MA    1500000
#define PTN5150_LIMIT_3000MA    3000000

struct ptn5150
{
    u8 addr;                        //I2C address
    struct i2c_client* client;
    struct regmap* regmap;
    struct device* dev;

    u32 default_limit_uA;
    bool optional;                  //When optional is true, do no error if not found

    struct regulator_desc desc;
    struct regulator_dev* reg_dev;
};

static const struct regmap_config ptn5150_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .cache_type = REGCACHE_RBTREE, 
};

static int ptn5150_write_reg(struct ptn5150* ptn5150, u8 reg, u8 val)
{
    if(!ptn5150 || !ptn5150->regmap){
        return -EINVAL;
    }

    dev_dbg(ptn5150->dev, "Write REG: 0x%02X, VAL 0x%02X\n", reg, val);

    return regmap_write(ptn5150->regmap, reg, (u32)val);
}

static int ptn5150_read_reg(struct ptn5150* ptn5150, u8 reg, u8* val)
{
    int ret;
    u32 reg_value;

    if(!ptn5150 || !ptn5150->regmap || !val){
        return -EINVAL;
    }

    ret = regmap_read(ptn5150->regmap, reg, &reg_value);

    if(ret != 0){
        return ret;
    }

    *val = reg_value & 0xFF;

    dev_dbg(ptn5150->dev, "Read REG: 0x%02X, VAL 0x%02X\n", reg, *val);
    
    return 0;
}

static int ptn5150_write_cur_limit(struct ptn5150* ptn5150, u32 limit)
{
    u8 value;   //register value

    if(!ptn5150){
        return -EINVAL;
    }
    
    if(ptn5150_read_reg(ptn5150, PTN5150_CTRL_REG, &value) != 0){
        return -EIO;
    } 

    /*
        bits 3 and 4 are used for current limit. Clear them first to 
        get them to a known state before setting the desired bits
    */
    value &= ~(0x3 << 3);    

    switch(limit){
        case PTN5150_LIMIT_900MA:   //00
        break;   
        case PTN5150_LIMIT_1500MA:  //01
            value |= BIT(3);
        break;
        case PTN5150_LIMIT_3000MA:  //10
            value |= BIT(4);
        break;
        default:
            dev_err(ptn5150->dev, "Invalid Current Limit %u\n", limit);
        return -EINVAL;
    }

    if(ptn5150_write_reg(ptn5150, PTN5150_CTRL_REG, value) != 0){
        return -EIO;
    } 

    return 0;
}

static int ptn5150_set_current_limit(struct regulator_dev* reg_dev,
                                     int min_uA, int max_uA)
{
    struct ptn5150* ptn5150;

    (void)min_uA;

    if(!reg_dev){
        return -EINVAL;
    }

    ptn5150 = rdev_get_drvdata(reg_dev);

    if(!ptn5150){
        return -EINVAL;
    }

    return ptn5150_write_cur_limit(ptn5150, (u32)max_uA);
}

//Read the Current Limit from the device
static int ptn5150_read_cur_limit(struct ptn5150* ptn5150, u32* limit)
{
    u8 value;   //register value

    if(!ptn5150 || !limit){
        return -EINVAL;
    }
    
    if(ptn5150_read_reg(ptn5150, PTN5150_CTRL_REG, &value) != 0){
        return -EIO;
    } 

    //bits 3 and 4 are used for current limit.
    value &= (0x3 << 3);    

    value >>= 3;

    switch(value){
        case 0: //00
            *limit = PTN5150_LIMIT_900MA;
        break;   
        case 1: //01
            *limit = PTN5150_LIMIT_1500MA;
        break;
        case 2: //10
            *limit = PTN5150_LIMIT_3000MA;
        break;
        default:
        return -EFAULT;
    }

    return 0;
}

static int ptn5150_get_current_limit(struct regulator_dev* reg_dev)
{
    struct ptn5150* ptn5150;
    u32 limit;    
    int ret;

    if(!reg_dev){
        return -EINVAL;
    }

    ptn5150 = rdev_get_drvdata(reg_dev);

    if(!ptn5150){
        return -EINVAL;
    }

    ret = ptn5150_read_cur_limit(ptn5150, &limit);
    if(ret != 0){
        return ret;
    }

    return limit;
}


static int ptn5150_remove(struct i2c_client* client)
{
    if(!client){
        return -EINVAL;
    }

    return 0;
}

static int ptn5150_parse_dt(struct ptn5150* ptn5150,
                            struct device_node* node)
{
    u32 value;

    if(of_property_read_u32(node, "regulator-init-microamp", &value) != 0){
        dev_info(ptn5150->dev, "No regulator-init-microamp in dt node, using default 900mA\n");
        ptn5150->default_limit_uA = PTN5150_LIMIT_900MA;
    }
    else{
        switch(value){
            case PTN5150_LIMIT_900MA:   //900mA 
            case PTN5150_LIMIT_1500MA:  //1.5A 
            case PTN5150_LIMIT_3000MA:  //3A 
                ptn5150->default_limit_uA = value;
                dev_info(ptn5150->dev, "Current Limit %uuA\n", value);
            break;
            default:
                dev_err(ptn5150->dev, "Invalid Current Limit %u\n", value);
            return -EINVAL;
        }
    }
    
    /*
        When "optional" is true the device may or may not be present.
        This prevents error messages when some revisions of a product
        use the ptn5150 part and some don't
    */
    ptn5150->optional = of_property_read_bool(node, "cti,optional");

    return 0;
}

//Attempt to read the ID register to see if device is on the bus
static bool ptn5150_detect(struct ptn5150* ptn5150)
{
    u8 value;

    if(!ptn5150){
        return false;
    }

    if(ptn5150_read_reg(ptn5150, PTN5150_ID_REG, &value) != 0){
        return false;
    }

    return true;
}

static struct regulator_ops ptn5150_ops = {

    .set_current_limit = ptn5150_set_current_limit,
    .get_current_limit = ptn5150_get_current_limit,

};

//Register with the regulator subsystem
int ptn5150_register(struct ptn5150* ptn5150)
{

    struct regulator_config cfg = {};

    if(!ptn5150 || !ptn5150->dev){
        return -EINVAL;
    }

    ptn5150->desc.name = devm_kasprintf(ptn5150->dev, GFP_KERNEL, "%s-%d-0x%02x",
                                        ptn5150->dev->driver->name,
                                        i2c_adapter_id(ptn5150->client->adapter),
                                        ptn5150->addr);

    if(!ptn5150->desc.name) {
        dev_err(ptn5150->dev, "Failed to allocate supply name\n");
        return -ENOMEM;
    }

    ptn5150->desc.type = REGULATOR_CURRENT;
    ptn5150->desc.owner = THIS_MODULE;
    ptn5150->desc.ops = &ptn5150_ops;
    ptn5150->desc.n_voltages = 1;
    ptn5150->desc.fixed_uV = 5000000;   //5V

    cfg.of_node = ptn5150->dev->of_node;
    cfg.dev = ptn5150->dev;
    cfg.driver_data = ptn5150;

    ptn5150->reg_dev = devm_regulator_register(ptn5150->dev, &ptn5150->desc, &cfg);

    if(IS_ERR(ptn5150->reg_dev)) {
        dev_err(ptn5150->dev, "Failed to register regulator: %ld\n", PTR_ERR(ptn5150->reg_dev));
        return PTR_ERR(ptn5150->reg_dev);
    }

    dev_dbg(ptn5150->dev, "Regulator Registered %s\n", ptn5150->desc.name);

    return 0;
}

static int ptn5150_probe(struct i2c_client* client,
                         const struct i2c_device_id* id)
{
	
    int ret;
    struct ptn5150* ptn5150;
    
    ptn5150 = devm_kzalloc(&client->dev, sizeof(struct ptn5150), GFP_KERNEL);
    if(!ptn5150){
        return -ENOMEM;
    }

    ptn5150->client = client;
    ptn5150->dev = &client->dev;
    ptn5150->addr = client->addr;

    ret = ptn5150_parse_dt(ptn5150, ptn5150->dev->of_node);
    if(ret != 0){
        dev_err(ptn5150->dev, "Failed to Parse DT\n");
        return ret;
    }
    ptn5150->regmap = devm_regmap_init_i2c(ptn5150->client, &ptn5150_regmap_config);
    if(IS_ERR(ptn5150->regmap)) {
        dev_err(ptn5150->dev, "regmap init failed: %ld\n", PTR_ERR(ptn5150->regmap));
        return -ENOMEM;
    }

    if(!ptn5150_detect(ptn5150)){

        if(!ptn5150->optional){
            dev_err(ptn5150->dev, "PTN5150 USB PD Controller Not Detected\n");
            return -EIO;
        }
        else{
            //This is OK because the device was marked as optional
            dev_dbg(ptn5150->dev, "PTN5150 USB PD Controller Not Detected\n");
            return 0;
        }
    }

    dev_dbg(ptn5150->dev, "PTN5150 USB PD Controller Detected\n");

    //add the private data to the device and I2C client 
    dev_set_drvdata(ptn5150->dev, ptn5150);
    i2c_set_clientdata(client, ptn5150);

    //set the default current limit
    ret = ptn5150_write_cur_limit(ptn5150, ptn5150->default_limit_uA);
    if(ret != 0){
        return ret;
    }

    ret = ptn5150_register(ptn5150);

    return ret;
}


static const struct i2c_device_id ptn5150_id[] = {
    { "ptn5150", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, ptn5150_id);

const struct of_device_id ptn5150_of_match[] = {
    { .compatible = "nxp,ptn5150", },
    { },
};
MODULE_DEVICE_TABLE(of, ptn5150_of_match);

static struct i2c_driver ptn5150_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "ptn5150",
        .of_match_table = ptn5150_of_match,
    },
    .probe = ptn5150_probe,
    .remove = ptn5150_remove,
    .id_table = ptn5150_id,
};

builtin_i2c_driver(ptn5150_i2c_driver);

MODULE_AUTHOR("Parker Newman <pnewman@connecttech.com>");
MODULE_LICENSE("GPL v2");

