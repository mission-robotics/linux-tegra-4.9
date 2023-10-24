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


#ifndef __TIS_SENSOR_CONNECTION_FPDLINK_H__
#define __TIS_SENSOR_CONNECTION_FPDLINK_H__

#include <linux/of.h>

struct tis_sensor_connection *tis_sensor_connection_fpdlink_create(struct device *dev, struct device_node *fpdlink_node);

#endif // __TIS_SENSOR_CONNECTION_FPDLINK_H__
