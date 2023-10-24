/*
 * Copyright (c) 2021, The Imaging Source Europe GmbH.  All rights reserved.
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


#ifndef __TIS_SENSOR_CONNECTION_INTERNAL_H__
#define __TIS_SENSOR_CONNECTION_INTERNAL_H__

struct tis_sensor_connection
{
    void (*release)(struct tis_sensor_connection *cxn);
    
    int (*check_device_present)(struct tis_sensor_connection *cxn, bool *is_present);
    int (*configure_clkout)(struct tis_sensor_connection *cxn, u32 clkout_hz, u32 *actual_clkout_hz);
    int (*configure_csi_lanes)(struct tis_sensor_connection *cxn, int num_lanes);
    int (*get_gpio_out_status)(struct tis_sensor_connection *cxn, int gpio, bool *status);
};

#endif // __TIS_SENSOR_CONNECTION_INTERNAL_H__