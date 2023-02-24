/*
 * A V4L2 driver for OmniVision OV7670 cameras.
 *
 * Copyright 2010 One Laptop Per Child
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#ifndef __IMX219_H
#define __IMX219_H

#include <linux/i2c.h>

int imx219_reg_read(const u16 addr);


#endif
