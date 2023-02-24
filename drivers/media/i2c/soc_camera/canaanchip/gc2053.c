/*
 * Driver for GC2053 CMOS Image Sensor
 *
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>
#include <linux/canaan-camera-module.h>

#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION          KERNEL_VERSION(0, 0x01, 0x01)
#define GC2053_NAME             "gc2053"
#define GC2053_MEDIA_BUS_FMT    MEDIA_BUS_FMT_SRGGB10_1X10

#define MIPI_FREQ_297M          297000000
#define GC2053_XVCLK_FREQ       24000000

#define GC2053_PAGE_SELECT      0xFE

#define GC2053_REG_CHIP_ID_H    0xF0
#define GC2053_REG_CHIP_ID_L    0xF1

#define GC2053_REG_EXP_H        0x03
#define GC2053_REG_EXP_L        0x04

#define GC2053_REG_VTS_H        0x41
#define GC2053_REG_VTS_L        0x42

#define GC2053_REG_CTRL_MODE    0x3E
#define GC2053_MODE_SW_STANDBY  0x11
#define GC2053_MODE_STREAMING   0x91

#define REG_NULL                0xFF

#define GC2053_CHIP_ID          0x2053

#define GC2053_VTS_MAX          0x3FFF
#define GC2053_HTS_MAX          0xFFF

#define GC2053_EXPOSURE_MAX     0x3FFF
#define GC2053_EXPOSURE_MIN     1
#define GC2053_EXPOSURE_STEP    1

#define GC2053_GAIN_MIN         256	//0x40
#define GC2053_GAIN_MAX         16380	//0x2000
#define GC2053_GAIN_STEP        1
#define GC2053_GAIN_DEFAULT     256	//64

#define GC2053_LANES            2

#define SENSOR_ID(_msb, _lsb)   ((_msb) << 8 | (_lsb))

#define GC2053_FLIP_MIRROR_REG  0x17

#define GC_MIRROR_BIT_MASK      BIT(0)
#define GC_FLIP_BIT_MASK        BIT(1)

#define GC2053_NUM_SUPPLIES ARRAY_SIZE(gc2053_supply_names)

#define to_gc2053(sd) container_of(sd, struct gc2053, subdev)

struct regval {
	u8 addr;
	u8 val;
};

struct gc2053_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct gc2053 {
	struct i2c_client   *client;
	struct gpio_desc    *reset_gpio;
	struct gpio_desc    *pwdn_gpio;

	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl    *hblank;
	struct v4l2_ctrl    *vblank;
	struct v4l2_ctrl    *h_flip;
	struct v4l2_ctrl    *v_flip;
	struct mutex        mutex;
	bool            streaming;
	bool			power_on;
	const struct gc2053_mode *cur_mode;
	unsigned int        lane_num;
	unsigned int        cfg_num;
	unsigned int        pixel_rate;

	u32         module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;
	u8			flip;
};

static const struct regval gc2053_start[] = {
	{0xf0, 0x00},		/* mode select streaming on */
	{0x3e, 0x91},
	{REG_NULL, 0x00},
};

static const struct regval gc2053_stop[] = {
	{0xf0, 0x00},		/* mode select streaming off */
	{0x3e, 0x00},
	{REG_NULL, 0x00},
};

/*
 * window_size=800*1080 mipi@2lane
 * mclk=24mhz,mipi_clk=594Mbps
 * pixel_line_total=2200,line_frame_total=1125
 * row_time=29.629us,frame_rate=30fps
 */
static const struct regval gc2053_800x1080_regs_2lane[] = {
	/****system****/
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x00},
	{0xf2, 0x00},
	{0xf3, 0x00},
	{0xf4, 0x36},
	{0xf5, 0xc0},
	{0xf6, 0x44},
	{0xf7, 0x01},
	{0xf8, 0x63},
	{0xf9, 0x40},
	{0xfc, 0x8e},
	/****CISCTL & ANALOG****/
	{0xfe, 0x00},
	{0x87, 0x18},
	{0xee, 0x30},
	{0xd0, 0xb7},
	{0x03, 0x04},
	{0x04, 0x60},
	{0x05, 0x04},
	{0x06, 0x4c},
	{0x07, 0x00},
	{0x08, 0x11},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0b, 0x00},
	{0x0c, 0x02},
	{0x0d, 0x04},
	{0x0e, 0x40},
	{0x12, 0xe2},
	{0x13, 0x16},
	{0x19, 0x0a},
	{0x21, 0x1c},
	{0x28, 0x0a},
	{0x29, 0x24},
	{0x2b, 0x04},
	{0x32, 0xf8},
	{0x37, 0x03},
	{0x39, 0x15},
	{0x43, 0x07},
	{0x44, 0x40},
	{0x46, 0x0b},
	{0x4b, 0x20},
	{0x4e, 0x08},
	{0x55, 0x20},
	{0x66, 0x05},
	{0x67, 0x05},
	{0x77, 0x01},
	{0x78, 0x00},
	{0x7c, 0x93},
	{0x8c, 0x12},
	{0x8d, 0x92},
	{0x90, 0x01},
	{0x9d, 0x10},
	{0xce, 0x7c},
	{0xd2, 0x41},
	{0xd3, 0xdc},
	{0xe6, 0x50},
	/*gain*/
	{0xb6, 0xc0},
	{0xb0, 0x60},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb3, 0x00},
	{0xb4, 0x00},
	{0xb8, 0x01},
	{0xb9, 0x00},
	/*blk*/
	{0x26, 0x30},
	{0xfe, 0x01},
	{0x40, 0x23},
	{0x55, 0x07},
	{0x60, 0x40},
	{0xfe, 0x04},
	{0x14, 0x78},
	{0x15, 0x78},
	{0x16, 0x78},
	{0x17, 0x78},
	/*window*/
	{0xfe, 0x01},
	{0x91, 0x00}, //Out Window Y1[10:8]
	{0x92, 0x02}, //Out Window Y1[7:0]
	{0x93, 0x02}, //Out Window X1[11:8]
	{0x94, 0x2f}, //Out Window X1[7:0]
	{0x95, 0x04}, //Out Window Height[10:8] 1080=0438
	{0x96, 0x38}, //Out Window Height[7:0]  
	{0x97, 0x03}, //Out Window Width[11:8]  1920=0780
	{0x98, 0x20}, //Out Window Width[7:0] 800=0320

	/*ISP*/
	{0xfe, 0x01},
	{0x01, 0x05},
	{0x02, 0x89},
	{0x04, 0x01},
	{0x07, 0xa6},
	{0x08, 0xa9},
	{0x09, 0xa8},
	{0x0a, 0xa7},
	{0x0b, 0xff},
	{0x0c, 0xff},
	{0x0f, 0x00},
	{0x50, 0x1c},
	{0x89, 0x03},
	{0xfe, 0x04},
	{0x28, 0x86},
	{0x29, 0x86},
	{0x2a, 0x86},
	{0x2b, 0x68},
	{0x2c, 0x68},
	{0x2d, 0x68},
	{0x2e, 0x68},
	{0x2f, 0x68},
	{0x30, 0x4f},
	{0x31, 0x68},
	{0x32, 0x67},
	{0x33, 0x66},
	{0x34, 0x66},
	{0x35, 0x66},
	{0x36, 0x66},
	{0x37, 0x66},
	{0x38, 0x62},
	{0x39, 0x62},
	{0x3a, 0x62},
	{0x3b, 0x62},
	{0x3c, 0x62},
	{0x3d, 0x62},
	{0x3e, 0x62},
	{0x3f, 0x62},
	/****DVP & MIPI****/
	{0xfe, 0x01},
	{0x9a, 0x06},
	{0x99, 0x00},
	{0xfe, 0x00},
	{0x7b, 0x2a},
	{0x23, 0x2d},
	{0xfe, 0x03},
	{0x01, 0x27},
	{0x02, 0x56},
	{0x03, 0x8e},
	{0x12, 0x20},
	{0x13, 0x03},
	{0xfe, 0x00},
	{0x3e, 0x81},
	{REG_NULL, 0x00},
};

/*
 * window_size=1920*1080 mipi@2lane
 * mclk=24mhz,mipi_clk=594Mbps
 * pixel_line_total=2200,line_frame_total=1125
 * row_time=29.629us,frame_rate=30fps
 */
static const struct regval gc2053_1920x1080_regs_2lane[] = {
	/****system****/
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x00},
	{0xf2, 0x00},
	{0xf3, 0x00},
	{0xf4, 0x36},
	{0xf5, 0xc0},
	{0xf6, 0x44},
	{0xf7, 0x01},
	{0xf8, 0x63},
	{0xf9, 0x40},
	{0xfc, 0x8e},
	/****CISCTL & ANALOG****/
	{0xfe, 0x00},
	{0x87, 0x18},
	{0xee, 0x30},
	{0xd0, 0xb7},
	{0x03, 0x04},
	{0x04, 0x60},
	{0x05, 0x04},
	{0x06, 0x4c},
	{0x07, 0x00},
	{0x08, 0x11},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0b, 0x00},
	{0x0c, 0x02},
	{0x0d, 0x04},
	{0x0e, 0x40},
	{0x12, 0xe2},
	{0x13, 0x16},
	{0x19, 0x0a},
	{0x21, 0x1c},
	{0x28, 0x0a},
	{0x29, 0x24},
	{0x2b, 0x04},
	{0x32, 0xf8},
	{0x37, 0x03},
	{0x39, 0x15},
	{0x43, 0x07},
	{0x44, 0x40},
	{0x46, 0x0b},
	{0x4b, 0x20},
	{0x4e, 0x08},
	{0x55, 0x20},
	{0x66, 0x05},
	{0x67, 0x05},
	{0x77, 0x01},
	{0x78, 0x00},
	{0x7c, 0x93},
	{0x8c, 0x12},
	{0x8d, 0x92},
	{0x90, 0x01},
	{0x9d, 0x10},
	{0xce, 0x7c},
	{0xd2, 0x41},
	{0xd3, 0xdc},
	{0xe6, 0x50},
	/*gain*/
	{0xb6, 0xc0},
	{0xb0, 0x60},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb3, 0x00},
	{0xb4, 0x00},
	{0xb8, 0x01},
	{0xb9, 0x00},
	/*blk*/
	{0x26, 0x30},
	{0xfe, 0x01},
	{0x40, 0x23},
	{0x55, 0x07},
	{0x60, 0x40},
	{0xfe, 0x04},
	{0x14, 0x78},
	{0x15, 0x78},
	{0x16, 0x78},
	{0x17, 0x78},
	/*window*/
	{0xfe, 0x01},
	{0x92, 0x00},
	{0x94, 0x03},
	{0x95, 0x04},//[10:0]win_out_height-1080
	{0x96, 0x38},
	{0x97, 0x07},//[11:0]win_out_width-1920
	{0x98, 0x80},
	/*ISP*/
	{0xfe, 0x01},
	{0x01, 0x05},
	{0x02, 0x89},
	{0x04, 0x01},
	{0x07, 0xa6},
	{0x08, 0xa9},
	{0x09, 0xa8},
	{0x0a, 0xa7},
	{0x0b, 0xff},
	{0x0c, 0xff},
	{0x0f, 0x00},
	{0x50, 0x1c},
	{0x89, 0x03},
	{0xfe, 0x04},
	{0x28, 0x86},
	{0x29, 0x86},
	{0x2a, 0x86},
	{0x2b, 0x68},
	{0x2c, 0x68},
	{0x2d, 0x68},
	{0x2e, 0x68},
	{0x2f, 0x68},
	{0x30, 0x4f},
	{0x31, 0x68},
	{0x32, 0x67},
	{0x33, 0x66},
	{0x34, 0x66},
	{0x35, 0x66},
	{0x36, 0x66},
	{0x37, 0x66},
	{0x38, 0x62},
	{0x39, 0x62},
	{0x3a, 0x62},
	{0x3b, 0x62},
	{0x3c, 0x62},
	{0x3d, 0x62},
	{0x3e, 0x62},
	{0x3f, 0x62},
	/****DVP & MIPI****/
	{0xfe, 0x01},
	{0x9a, 0x06},
	{0x99, 0x00},
	{0xfe, 0x00},
	{0x7b, 0x2a},
	{0x23, 0x2d},
	{0xfe, 0x03},
	{0x01, 0x27},
	{0x02, 0x56},
	{0x03, 0x8e},
	{0x12, 0x80},
	{0x13, 0x07},
	{0xfe, 0x00},
	{0x3e, 0x81},
	{REG_NULL, 0x00},
};

static const struct gc2053_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x460,
		.hts_def = 0x898,
		.vts_def = 0x465,
		.reg_list = gc2053_1920x1080_regs_2lane,
	},
	{
		.width = 800,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x460,
		.hts_def = 0x898,
		.vts_def = 0x465,
		.reg_list = gc2053_800x1080_regs_2lane,
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_297M
};

/* sensor register write */
static int gc2053_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"gc2053 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int gc2053_write_array(struct i2c_client *client,
				  const struct regval *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = gc2053_write_reg(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

static int reg16_write(struct i2c_client *client, u8 addr, const u16 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[3];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;
	tx[0] = addr;
	tx[1] = data >> 8;
	tx[2] = data & 0xff;
	ret = i2c_transfer(adap, &msg, 1);
	udelay(20);//mdelay(2);

	return ret == 1 ? 0 : -EIO;
}

/* sensor register read */
static int gc2053_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev,
		"gc2053 read reg(0x%x val:0x%x) failed !\n", reg, *val);

	return ret;
}

static int gc2053_get_reso_dist(const struct gc2053_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct gc2053_mode *
gc2053_find_best_fit(struct gc2053 *gc2053, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < gc2053->cfg_num; i++) {
		dist = gc2053_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int gc2053_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2053 *gc2053 = container_of(ctrl->handler,
						 struct gc2053, ctrl_handler);
	struct i2c_client *client = gc2053->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		//max = gc2053->cur_mode->height + ctrl->val - 4;
		max = gc2053->cur_mode->height + ctrl->val - 1;
		__v4l2_ctrl_modify_range(gc2053->exposure,
					 gc2053->exposure->minimum, max,
					 gc2053->exposure->step,
					 gc2053->exposure->default_value);
		break;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	    ret = reg16_write(gc2053->client, GC2053_REG_EXP_H, ctrl->val);
		// ret = gc2053_write_reg(gc2053->client, GC2053_REG_EXP_H,
		// 			   (ctrl->val >> 8) & 0x3f);
		//ret |= gc2053_write_reg(gc2053->client, GC2053_REG_EXP_L),
		// 			   ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
		ret = reg16_write(gc2053->client, 0xb1, ctrl->val);
		//gain = ctrl->val;
		//ret = gc2053_write_reg(gc2053->client, 0xb2, gain & 0xfc);
		//ret |= gc2053_write_reg(gc2053->client, 0xb1, gain >> 8);

		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + gc2053->cur_mode->height;
		ret = gc2053_write_reg(gc2053->client, GC2053_REG_VTS_H, (vts >> 8) & 0x3f);
		ret |= gc2053_write_reg(gc2053->client, GC2053_REG_VTS_L, vts & 0xff);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			gc2053->flip |= GC_MIRROR_BIT_MASK;
		else
			gc2053->flip &= ~GC_MIRROR_BIT_MASK;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			gc2053->flip |= GC_FLIP_BIT_MASK;
		else
			gc2053->flip &= ~GC_FLIP_BIT_MASK;
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops gc2053_ctrl_ops = {
	.s_ctrl = gc2053_set_ctrl,
};

static int gc2053_parse_of(struct gc2053 *gc2053)
{
	struct device *dev = &gc2053->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	gc2053->lane_num = rval;
	if (2 == gc2053->lane_num) {
		gc2053->cur_mode = &supported_modes[0];
		gc2053->cfg_num = ARRAY_SIZE(supported_modes);

		/*pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		gc2053->pixel_rate = MIPI_FREQ_297M * 2U * (gc2053->lane_num) / 10U;
		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 gc2053->lane_num, gc2053->pixel_rate);
	} else {
		dev_info(dev, "gc2053 can not support the lane num(%d)\n", gc2053->lane_num);
	}
	return 0;
}

static int gc2053_initialize_controls(struct gc2053 *gc2053)
{
	const struct gc2053_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc2053->ctrl_handler;
	mode = gc2053->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 10);
	if (ret)
		return ret;
	handler->lock = &gc2053->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
					  0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, gc2053->pixel_rate, 1, gc2053->pixel_rate);

	h_blank = mode->hts_def - mode->width;
	gc2053->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc2053->hblank)
		gc2053->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc2053->vblank = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC2053_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 1;
	gc2053->exposure = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_EXPOSURE, GC2053_EXPOSURE_MIN,
				exposure_max, GC2053_EXPOSURE_STEP,
				mode->exp_def);

	gc2053->anal_gain = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, GC2053_GAIN_MIN,
				GC2053_GAIN_MAX, GC2053_GAIN_STEP,
				GC2053_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_GAIN, GC2053_GAIN_MIN,
				GC2053_GAIN_MAX, GC2053_GAIN_STEP,
				GC2053_GAIN_DEFAULT);
	gc2053->h_flip = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	gc2053->v_flip = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	gc2053->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc2053->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc2053->subdev.ctrl_handler = handler;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc2053_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC2053_XVCLK_FREQ / 1000 / 1000);
}

static int __gc2053_power_on(struct gc2053 *gc2053)
{
	u32 delay_us;

	if (!IS_ERR(gc2053->reset_gpio)) {
		gpiod_set_value_cansleep(gc2053->reset_gpio, 1);
		usleep_range(100, 200);
	}
	if (!IS_ERR(gc2053->pwdn_gpio))
		gpiod_set_value_cansleep(gc2053->pwdn_gpio, 0);

	if (!IS_ERR(gc2053->reset_gpio))
		gpiod_set_value_cansleep(gc2053->reset_gpio, 0);
	usleep_range(3000, 6000);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc2053_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;
}

static void __gc2053_power_off(struct gc2053 *gc2053)
{
	if (!IS_ERR(gc2053->pwdn_gpio))
		gpiod_set_value_cansleep(gc2053->pwdn_gpio, 1);

	if (!IS_ERR(gc2053->reset_gpio))
		gpiod_set_value_cansleep(gc2053->reset_gpio, 1);
}

static int gc2053_check_sensor_id(struct gc2053 *gc2053,
				   struct i2c_client *client)
{
	struct device *dev = &gc2053->client->dev;
	u8 pid = 0, ver = 0;
	u16 id = 0;
	int ret = 0;

	/* Check sensor revision */
	ret = gc2053_read_reg(client, GC2053_REG_CHIP_ID_H, &pid);
	ret |= gc2053_read_reg(client, GC2053_REG_CHIP_ID_L, &ver);
	if (ret) {
		dev_err(&client->dev, "gc2053_read_reg failed (%d)\n", ret);
		return ret;
	}

	id = SENSOR_ID(pid, ver);
	if (id != GC2053_CHIP_ID) {
		dev_err(&client->dev,
				"Sensor detection failed (%04X,%d)\n",
				id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected GC%04x sensor\n", id);
	return 0;
}

static int gc2053_set_flip(struct gc2053 *gc2053, u8 mode)
{
	u8 match_reg = 0;

	gc2053_read_reg(gc2053->client, GC2053_FLIP_MIRROR_REG, &match_reg);

	if (mode == GC_FLIP_BIT_MASK) {
		match_reg |= GC_FLIP_BIT_MASK;
		match_reg &= ~GC_MIRROR_BIT_MASK;
	} else if (mode == GC_MIRROR_BIT_MASK) {
		match_reg |= GC_MIRROR_BIT_MASK;
		match_reg &= ~GC_FLIP_BIT_MASK;
	} else if (mode == (GC_MIRROR_BIT_MASK |
		GC_FLIP_BIT_MASK)) {
		match_reg |= GC_FLIP_BIT_MASK;
		match_reg |= GC_MIRROR_BIT_MASK;
	} else {
		match_reg &= ~GC_FLIP_BIT_MASK;
		match_reg &= ~GC_MIRROR_BIT_MASK;
	}
	return gc2053_write_reg(gc2053->client, GC2053_FLIP_MIRROR_REG, match_reg);
}

static int __gc2053_start_stream(struct gc2053 *gc2053)
{
	int ret;

	ret = gc2053_write_array(gc2053->client, gc2053->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&gc2053->mutex);
	ret = v4l2_ctrl_handler_setup(&gc2053->ctrl_handler);
	mutex_lock(&gc2053->mutex);

	ret = gc2053_set_flip(gc2053, gc2053->flip);
	if (ret)
		return ret;

	ret = gc2053_write_array(gc2053->client, gc2053_start);
	if (ret)
		return ret;

	return ret;
}

static int __gc2053_stop_stream(struct gc2053 *gc2053)
{
	int ret;

	ret = gc2053_write_array(gc2053->client, gc2053_stop);
	if (ret)
		return ret;

	return ret;
}

static long gc2053_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	long ret = 0;

	dev_info(&gc2053->client->dev, "gc2053_ioctl\n");

	switch (cmd) {
	//case:
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long gc2053_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct module_inf *inf;
	struct module_awb_cfg *awb_cfg;
	long ret = 0;

	switch (cmd) {
	case CANAANMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc2053_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case CANAANMODULE_AWB_CFG:
		awb_cfg = kzalloc(sizeof(*awb_cfg), GFP_KERNEL);
		if (!awb_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc2053_ioctl(sd, cmd, awb_cfg);
		if (!ret)
			ret = copy_to_user(up, awb_cfg, sizeof(*awb_cfg));
		kfree(awb_cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#endif

static int gc2053_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	int ret = 0;

	mutex_lock(&gc2053->mutex);
	on = !!on;
	if (on == gc2053->streaming)
		goto unlock_and_return;

	if (on) {
		ret = __gc2053_start_stream(gc2053);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			goto unlock_and_return;
		}
	} else {
		__gc2053_stop_stream(gc2053);
	}

	gc2053->streaming = on;

unlock_and_return:
	mutex_unlock(&gc2053->mutex);
	return 0;
}

static int gc2053_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	const struct gc2053_mode *mode = gc2053->cur_mode;

	mutex_lock(&gc2053->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc2053->mutex);

	return 0;
}

static int gc2053_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	u32 val = 1 << (GC2053_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = (val | V4L2_MBUS_CSI2_CHANNEL_1);

	return 0;
}
static int gc2053_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = GC2053_MEDIA_BUS_FMT;
	return 0;
}

static int gc2053_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc2053 *gc2053 = to_gc2053(sd);

	if (fse->index >= gc2053->cfg_num)
		return -EINVAL;

	if (fse->code != GC2053_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int gc2053_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc2053 *gc2053 = to_gc2053(sd);

	if (fie->index >= gc2053->cfg_num)
		return -EINVAL;

	fie->code = GC2053_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int gc2053_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	const struct gc2053_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc2053->mutex);

	mode = gc2053_find_best_fit(gc2053, fmt);
	dev_info(&gc2053->client->dev, "set_fmt, width:%u, height:%u.\n", mode->width, mode->height);

	fmt->format.code = GC2053_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc2053->mutex);
		return -ENOTTY;
#endif
	} else {
		gc2053->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc2053->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc2053->vblank, vblank_def,
					 GC2053_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc2053->mutex);
	return 0;
}

static int gc2053_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	const struct gc2053_mode *mode = gc2053->cur_mode;

	mutex_lock(&gc2053->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc2053->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC2053_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc2053->mutex);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc2053_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc2053_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc2053->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC2053_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc2053->mutex);
	/* No crop or compose */
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc2053_internal_ops = {
	.open = gc2053_open,
};
#endif

static int gc2053_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	int ret = 0;

	mutex_lock(&gc2053->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc2053->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		gc2053->power_on = true;
	} else {
		gc2053->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc2053->mutex);

	return ret;
}

static const struct v4l2_subdev_core_ops gc2053_core_ops = {
	.s_power = gc2053_s_power,
	.ioctl = gc2053_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc2053_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc2053_video_ops = {
	.s_stream = gc2053_s_stream,
	.g_frame_interval = gc2053_g_frame_interval,
	.g_mbus_config = gc2053_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gc2053_pad_ops = {
	.enum_mbus_code = gc2053_enum_mbus_code,
	.enum_frame_size = gc2053_enum_frame_sizes,
	.enum_frame_interval = gc2053_enum_frame_interval,
	.get_fmt = gc2053_get_fmt,
	.set_fmt = gc2053_set_fmt,
};

static const struct v4l2_subdev_ops gc2053_subdev_ops = {
	.core   = &gc2053_core_ops,
	.video  = &gc2053_video_ops,
	.pad    = &gc2053_pad_ops,
};

static int gc2053_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc2053 *gc2053;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc2053 = devm_kzalloc(dev, sizeof(*gc2053), GFP_KERNEL);
	if (!gc2053)
		return -ENOMEM;

	gc2053->client = client;
	ret = of_property_read_u32(node, CANAANMODULE_CAMERA_MODULE_INDEX,
				   &gc2053->module_index);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_FACING,
				       &gc2053->module_facing);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_NAME,
				       &gc2053->module_name);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_LENS_NAME,
				       &gc2053->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	gc2053->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc2053->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc2053->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc2053->pwdn_gpio))
		dev_info(dev, "Failed to get pwdn-gpios, maybe no used\n");

	if (gpiod_export(gc2053->reset_gpio, 0)) {
		dev_err(dev, "gpiod export gc2053 reset failed.");
	}

	if (gpiod_export(gc2053->pwdn_gpio, 0)) {
		dev_err(dev, "gpiod export gc2053 powerdown failed.");
	}

	ret = gc2053_parse_of(gc2053);
	if (ret != 0)
		return -EINVAL;

	/* 1920 * 1080 by default */
	gc2053->cur_mode = &supported_modes[0];
	gc2053->cfg_num = ARRAY_SIZE(supported_modes);

	mutex_init(&gc2053->mutex);

	sd = &gc2053->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc2053_subdev_ops);
	ret = gc2053_initialize_controls(gc2053);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc2053_power_on(gc2053);
	if (ret)
		goto err_free_handler;

	ret = gc2053_check_sensor_id(gc2053, client);
	if (ret)
		goto err_power_off;

	sd->internal_ops = &gc2053_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
						V4L2_SUBDEV_FL_HAS_EVENTS;

	gc2053->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc2053->pad);
	if (ret < 0)
		goto err_power_off;

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc2053->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc2053->module_index, facing,
		 GC2053_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif

err_power_off:
	__gc2053_power_off(gc2053);
err_free_handler:
	v4l2_ctrl_handler_free(&gc2053->ctrl_handler);

err_destroy_mutex:
	mutex_destroy(&gc2053->mutex);
	return ret;
}

static int gc2053_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2053 *gc2053 = to_gc2053(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc2053->ctrl_handler);
	mutex_destroy(&gc2053->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc2053_power_off(gc2053);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static const struct i2c_device_id gc2053_match_id[] = {
	{ "gc2053", 0 },
	{ },
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc2053_of_match[] = {
	{ .compatible = "galaxycore,gc2053" },
	{},
};
MODULE_DEVICE_TABLE(of, gc2053_of_match);
#endif

static struct i2c_driver gc2053_i2c_driver = {
	.driver = {
		.name = GC2053_NAME,
		.of_match_table = of_match_ptr(gc2053_of_match),
	},
	.probe      = &gc2053_probe,
	.remove     = &gc2053_remove,
	.id_table   = gc2053_match_id,
};

module_i2c_driver(gc2053_i2c_driver);
MODULE_DESCRIPTION("GC2035 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
