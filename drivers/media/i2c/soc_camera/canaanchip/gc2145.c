/*
 * Driver for GC2145 CMOS Image Sensor
 *
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <linux/canaan-camera-module.h>

#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x01, 0x01)
#define GC2145_NAME		"gc2145"
#define GC2145_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SRGGB10_1X10

#define MIPI_FREQ_120M		120000000
#define MIPI_FREQ_240M		240000000

#define GC2145_XVCLK_FREQ	24000000

#define GC2145_REG_CHIP_ID_H	0xF0
#define GC2145_REG_CHIP_ID_L	0xF1

#define GC2145_REG_EXP_SHORT_H	0x01
#define GC2145_REG_EXP_SHORT_L	0x02
#define GC2145_REG_EXP_LONG_H	0x03
#define GC2145_REG_EXP_LONG_L	0x04

#define GC2145_MIRROR_FLIP_REG	0x17
#define MIRROR_MASK		BIT(0)
#define FLIP_MASK		BIT(1)

#define GC2145_CHIP_ID		0x2145

#define GC2145_VTS_MAX		0x3FFF
#define GC2145_HTS_MAX		0xFFF

#define GC2145_EXPOSURE_MAX	0x3FFF
#define GC2145_EXPOSURE_MIN	1
#define GC2145_EXPOSURE_STEP	1

#define GC2145_GAIN_MIN		0x40
#define GC2145_GAIN_MAX		0x2000
#define GC2145_GAIN_STEP	1
#define GC2145_GAIN_DEFAULT	64

#define GC2145_LANES		2

#define REG_NULL			0xFF

#define GC2145219_EXP_LINES_MARGIN 1

#define to_gc2145(sd) container_of(sd, struct gc2145, subdev)

enum {
	LINK_FREQ_120M_INDEX,
	LINK_FREQ_240M_INDEX,
};

struct gc2145_reg {
	u8 addr;
	u8 val;
};

struct gain_reg_config {
	u32 value;
	u16 analog_gain;
	u16 col_gain;
	u16 analog_sw;
	u16 ram_width;
};

struct gc2145_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 link_freq_index;
	const struct gc2145_reg *reg_list;
};

struct gc2145 {
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;

	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl    *hblank;
	struct v4l2_ctrl    *vblank;
	struct v4l2_ctrl    *h_flip;
	struct v4l2_ctrl    *v_flip;
	struct v4l2_ctrl    *link_freq;
	struct v4l2_ctrl    *pixel_rate;

	struct mutex    lock;
	bool		    streaming;
	bool		    power_on;
	unsigned int    cfg_num;
	const struct gc2145_mode *cur_mode;

	u32		module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;

	bool			  has_init_exp;
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_120M,
	MIPI_FREQ_240M,
};

static const struct gc2145_reg gc2145_start[] = {
	{0xfe, 0x03},
	{0x10, 0x91},/* mode select streaming on */
	{0xfe, 0x00},
	{REG_NULL, 0x00},
};

static const struct gc2145_reg gc2145_stop[] = {
	{0xfe, 0x03},
	{0x10, 0x81},/* mode select streaming off */
	{0xfe, 0x00},
	{REG_NULL, 0x00},
};

/*
 * window size=1600*1200 mipi@2lane
 * mclk=24M mipi_clk=480Mbps
 * pixel_line_total=1920  line_frame_total=1250
 * row_time=40us frame_rate=20fps
 */
static const struct gc2145_reg gc2145_1600x1200_mipi_raw10_settings[] = {
	{0xfe,0xf0},
	{0xfe,0xf0},
	{0xfe,0xf0},
	{0xfc,0x06},
	{0xf6,0x00},
	{0xf7,0x1d},
	{0xf8,0x84},
	{0xfa,0x00},
	{0xf9,0x8e},
	{0xf2,0x00},

	/*** Analog & Cisctl ***/
	{0xfe,0x00},
	{0x03,0x04},//Exposure[12:8]
	{0x04,0xe2},//Exposure[7:0]
	{0x05,0x01},//hb[11:8]
	{0x06,0x56},//hb[7:0]
	{0x07,0x00},//vb[12:8]
	{0x08,0x14},//vb[7:0]

	{0x09,0x00},//row_start[10:8]
	{0x0a,0x00},//row_start[7:0]
	{0x0b,0x00},//col_start[10:8]
	{0x0c,0x00},//col_start[7:1]
	{0x0d,0x04},//win_height[10:8] 04c0=1216
	{0x0e,0xc0},//win_height[7:0]
	{0x0f,0x06},//win_width[10:8] 0652=1618
	{0x10,0x52},//win_width[7:0]
	{0x12,0x2e},
	{0x17,0x13},
	{0x18,0x22},
	{0x19,0x0e},
	{0x1a,0x01},
	{0x1b,0x4b},
	{0x1c,0x07},
	{0x1d,0x10},
	{0x1e,0x88},
	{0x1f,0x78},
	{0x20,0x03},
	{0x21,0x40},
	{0x22,0xa0},
	{0x24,0x16},
	{0x25,0x01},
	{0x26,0x10},
	{0x2d,0x60},
	{0x30,0x01},
	{0x31,0x90},
	{0x33,0x06},
	{0x34,0x01},

	/*** ISP reg ***/
	{0x80,0x06},
	{0x81,0x00},
	{0x82,0x30},
	{0x83,0x00},
	{0x84,0x19},
	{0x86,0x02},
	{0x88,0x03},
	{0x89,0x03},
	{0x85,0x30},
	{0x8a,0x00},
	{0x8b,0x00},
	{0xb0,0x10},
	{0xb1,0x10},
	{0xc3,0x00},
	{0xc4,0x80},
	{0xc5,0x90},
	{0xc6,0x38},
	{0xc7,0x40},
	{0xec,0x06},
	{0xed,0x04},
	{0xee,0x60},
	{0xef,0x90},
	{0xb6,0x00}, //close aec
	{0x90,0x01},
	{0x91,0x00},//out_win_y1[10:8]
	{0x92,0x01},//out_win_y1[7:0]
	{0x93,0x00},//out_win_x1[10:8]
	{0x94,0x01},//out_win_x1[7:0]
	{0x95,0x04},//out_win_height[10:8] 1200=04b0
	{0x96,0xb0},//out_win_height[7:0]
	{0x97,0x06},//out_win_width[10:8] 1600=0640
	{0x98,0x40},//out_win_width[7:0]

	/*** BLK ***/
	{0x40,0x42},
	{0x41,0x00},
	{0x43,0x54},
	{0x5e,0x00},
	{0x5f,0x00},
	{0x60,0x00},
	{0x61,0x00},
	{0x62,0x00},
	{0x63,0x00},
	{0x64,0x00},
	{0x65,0x00},
	{0x66,0x20},
	{0x67,0x20},
	{0x68,0x20},
	{0x69,0x20},
	{0x76,0x00},
	{0x6a,0x08},
	{0x6b,0x08},
	{0x6c,0x08},
	{0x6d,0x08},
	{0x6e,0x08},
	{0x6f,0x08},
	{0x70,0x08},
	{0x71,0x08},
	{0x76,0x00},
	{0x72,0xf0},
	{0x7e,0x3c},
	{0x7f,0x00},
	{0xfe,0x02},
	{0x48,0x15},
	{0x49,0x00},
	{0x4b,0x0b},
	{0xfe,0x00},

	/*** dark sun ***/
	{0xfe,0x00},
	{0x18,0x22},
	{0xfe,0x02},
	{0x40,0xbf},
	{0x46,0xcf},
	{0xfe,0x00},

	/***MIPI*****/
	{0xfe,0x03},
	{0x01,0x87},//[7]:clk lane_p2s_sel
	{0x02,0x22},
	{0x03,0x10},//[4]:
	{0x04,0x90},
	{0x05,0x01},
	{0x06,0x88},
	{0x11,0x2b},
	{0x12,0xd0},
	{0x13,0x07},
	{0x15,0x10},
	{0x17,0xf1},
	{0xfe,0x00},
	{REG_NULL, 0x00},
};

static const struct gc2145_reg gc2145_1280x960_mipi_raw10_settings[] = {
	{0xfe, 0xf0},
	{0xfe, 0xf0},
	{0xfe, 0xf0},
	{0xfc, 0x06},
	{0xf6, 0x00},
	{0xf7, 0x1d},
	{0xf8, 0x84},
	{0xfa, 0x00},
	{0xf9, 0x8e},
	{0xf2, 0x00},

    /*** Analog & Cisctl ***/
	{0xfe, 0x00},
	{0x03, 0x04},
	{0x04, 0xe2},
	{0x05, 0x01},
	{0x06, 0x56},
	{0x07, 0x00},
	{0x08, 0x14},
	{0x09, 0x00},//row_start[10:8]
	{0x0a, 0x00},//row_start[7:0]
	{0x0b, 0x00},//col_start[10:8]
	{0x0c, 0x00},//col_start[7:1]
	{0x0d, 0x04},//win_height[10:8] 04c0=1216
	{0x0e, 0xc0},//win_height[7:0]
	{0x0f, 0x06},//win_width[10:8] 0652=1618
	{0x10, 0x52},//win_width[7:0]

	{0x12, 0x2e},
	{0x17, 0x13}, //mirror
	{0x18, 0x22},
	{0x19, 0x0e},
	{0x1a, 0x01},
	{0x1b, 0x4b},
	{0x1c, 0x07},
	{0x1d, 0x10},
	{0x1e, 0x88},
	{0x1f, 0x78},
	{0x20, 0x03},
	{0x21, 0x40},
	{0x22, 0xa0},
	{0x24, 0x16},
	{0x25, 0x01},
	{0x26, 0x10},
	{0x2d, 0x60},
	{0x30, 0x01},
	{0x31, 0x90},
	{0x33, 0x06},
	{0x34, 0x01},

    /*** ISP reg ***/
	{0xfe, 0x00},
	{0x80, 0x06},
	{0x81, 0x00},
	{0x82, 0x30},
	{0x83, 0x00},
	{0x84, 0x19},
	{0x86, 0x02},
	{0x88, 0x03},
	{0x89, 0x03},
	{0x85, 0x30},
	{0x8a, 0x00},
	{0x8b, 0x00},
	{0xb0, 0x10},
	{0xb1, 0x10},
	{0xc3, 0x00},
	{0xc4, 0x80},
	{0xc5, 0x90},
	{0xc6, 0x38},
	{0xc7, 0x40},
	{0xec, 0x06},
	{0xed, 0x04},
	{0xee, 0x60},
	{0xef, 0x90},
	{0xb6, 0x00}, //close aec
	{0x90, 0x01},//Crop enable
	{0x91, 0x00},//out_win_y1[10:8]
	{0x92, 0x01},//out_win_y1[7:0]
	{0x93, 0x00},//out_win_x1[10:8]
	{0x94, 0x01},//out_win_x1[7:0]
	{0x95, 0x03},//out_win_height[10:8]
	{0x96, 0xc0},//out_win_height[7:0] //960
	{0x97, 0x05},//out_win_width[10:8]
	{0x98, 0x00},//out_win_width[7:0] //1280

	/*** BLK ***/
	{0x40, 0x42},
	{0x41, 0x00},
	{0x43, 0x54},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x00},
	{0x63, 0x00},
	{0x64, 0x00},
	{0x65, 0x00},
	{0x66, 0x20},
	{0x67, 0x20},
	{0x68, 0x20},
	{0x69, 0x20},
	{0x76, 0x00},
	{0x6a, 0x08},
	{0x6b, 0x08},
	{0x6c, 0x08},
	{0x6d, 0x08},
	{0x6e, 0x08},
	{0x6f, 0x08},
	{0x70, 0x08},
	{0x71, 0x08},
	{0x76, 0x00},
	{0x72, 0xf0},
	{0x7e, 0x3c},
	{0x7f, 0x00},
	{0xfe, 0x02},
	{0x48, 0x15},
	{0x49, 0x00},
	{0x4b, 0x0b},
	{0xfe, 0x00},

    /*** dark sun ***/
	{0xfe, 0x00},
	{0x18, 0x22},
	{0xfe, 0x02},
	{0x40, 0xbf},
	{0x46, 0xcf},
	{0xfe, 0x00},

    /***MIPI*****/
	{0xfe, 0x03},
	{0x01, 0x87},
	{0x02, 0x22},
	{0x03, 0x10},
	{0x04, 0x90},
	{0x05, 0x01},
	{0x06, 0x88},
	{0x11, 0x2b},
	{0x12, 0x40},
	{0x13, 0x06},
	{0x15, 0x10},
	{0x17, 0xf1},
	{0xfe, 0x00},
	{REG_NULL, 0x00},
};

static const struct gc2145_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 960,
		.max_fps = {
			.numerator = 10000,
			.denominator = 200000,
		},
		.exp_def = 0x460,
		.hts_def = 0x780,
		.vts_def = 0x4e2,
		.link_freq_index = LINK_FREQ_240M_INDEX,
		.reg_list = gc2145_1280x960_mipi_raw10_settings,
	},
	{
		.width = 1600,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 200000,
		},
		.exp_def = 0x460,
		.hts_def = 0x780,
		.vts_def = 0x4e2,
		.link_freq_index = LINK_FREQ_240M_INDEX,
		.reg_list = gc2145_1600x1200_mipi_raw10_settings,
	},
};

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
static u64 to_pixel_rate(u32 index)
{
	u64 pixel_rate = link_freq_menu_items[index] * 2 * GC2145_LANES;

	do_div(pixel_rate, 10);

	return pixel_rate;
}

/* sensor register write */
static int gc2145_write_reg(struct i2c_client *client, u8 reg, u8 val)
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

static int gc2145_write_array(struct i2c_client *client,
				  const struct gc2145_reg *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = gc2145_write_reg(client, regs[i].addr, regs[i].val);
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
static int gc2145_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"gc2145 read reg(0x%x val:0x%x) failed !\n", reg, *val);

	return ret;
}

static const struct gain_reg_config gain_reg_configs[] = {
	{  64, 0x0000, 0x0100, 0x6807, 0x00f8},
	{  75, 0x0010, 0x010c, 0x6807, 0x00f8},
	{  90, 0x0020, 0x011b, 0x6c08, 0x00f9},
	{ 105, 0x0030, 0x012c, 0x6c0a, 0x00fa},
	{ 122, 0x0040, 0x013f, 0x7c0b, 0x00fb},
	{ 142, 0x0050, 0x0216, 0x7c0d, 0x00fe},
	{ 167, 0x0060, 0x0235, 0x7c0e, 0x00ff},
	{ 193, 0x0070, 0x0316, 0x7c10, 0x0801},
	{ 223, 0x0080, 0x0402, 0x7c12, 0x0802},
	{ 257, 0x0090, 0x0431, 0x7c13, 0x0803},
	{ 299, 0x00a0, 0x0532, 0x7c15, 0x0805},
	{ 346, 0x00b0, 0x0635, 0x7c17, 0x0807},
	{ 397, 0x00c0, 0x0804, 0x7c18, 0x0808},
	{ 444, 0x005a, 0x0919, 0x7c17, 0x0807},
	{ 523, 0x0083, 0x0b0f, 0x7c17, 0x0807},
	{ 607, 0x0093, 0x0d12, 0x7c19, 0x0809},
	{ 700, 0x0084, 0x1000, 0x7c1b, 0x080c},
	{ 817, 0x0094, 0x123a, 0x7c1e, 0x080f},
	{1131, 0x005d, 0x1a02, 0x7c23, 0x0814},
	{1142, 0x009b, 0x1b20, 0x7c25, 0x0816},
	{1334, 0x008c, 0x200f, 0x7c27, 0x0818},
	{1568, 0x009c, 0x2607, 0x7c2a, 0x081b},
	{2195, 0x00b6, 0x3621, 0x7c32, 0x0823},
	{2637, 0x00ad, 0x373a, 0x7c36, 0x0827},
	{3121, 0x00bd, 0x3d02, 0x7c3a, 0x082b},
};

static int gc2145_set_gain(struct gc2145 *gc2145, u32 gain)
{
	int ret = 0;
	ret = gc2145_write_reg(gc2145->client, 0xb1, gain >> 4);
	return ret;
}

static int gc2145_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2145 *gc2145 = container_of(ctrl->handler,
					     struct gc2145, ctrl_handler);
	s64 max;
	int ret = 0;

	dev_dbg(&gc2145->client->dev, "%s enter, id:0x%X val:%d.\n", __func__, ctrl->id, ctrl->val);

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc2145->cur_mode->height + ctrl->val - GC2145219_EXP_LINES_MARGIN;
		__v4l2_ctrl_modify_range(gc2145->exposure,
					 gc2145->exposure->minimum, max,
					 gc2145->exposure->step,
					 gc2145->exposure->default_value);
		break;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	    ret = reg16_write(gc2145->client,  GC2145_REG_EXP_LONG_H, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
		ret = gc2145_write_reg(gc2145->client,  0xb1, (ctrl->val +256)>>4);
		break;
	case V4L2_CID_VBLANK:
		/* The exposure goes up and reduces the frame rate, no need to write vb */
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		dev_warn(&gc2145->client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops gc2145_ctrl_ops = {
	.s_ctrl = gc2145_set_ctrl,
};

static int gc2145_initialize_controls(struct gc2145 *gc2145)
{
	const struct gc2145_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc2145->ctrl_handler;
	mode = gc2145->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 10);
	if (ret)
		return ret;
	handler->lock = &gc2145->lock;

	gc2145->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq_menu_items) - 1, 0,
						   link_freq_menu_items);

	gc2145->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
						   0, to_pixel_rate(LINK_FREQ_240M_INDEX),
						   1, to_pixel_rate(LINK_FREQ_120M_INDEX));

	h_blank = mode->hts_def - mode->width;
	gc2145->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (gc2145->hblank)
		gc2145->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc2145->vblank = v4l2_ctrl_new_std(handler, &gc2145_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   GC2145_VTS_MAX - mode->height,
					   1, vblank_def);

	exposure_max = mode->vts_def - GC2145219_EXP_LINES_MARGIN;
	gc2145->exposure = v4l2_ctrl_new_std(handler, &gc2145_ctrl_ops,
					     V4L2_CID_EXPOSURE, GC2145_EXPOSURE_MIN,
					     exposure_max, GC2145_EXPOSURE_STEP,
					     mode->exp_def);

	gc2145->anal_gain = v4l2_ctrl_new_std(handler, &gc2145_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, GC2145_GAIN_MIN,
					      GC2145_GAIN_MAX, GC2145_GAIN_STEP,
					      GC2145_GAIN_DEFAULT);


	v4l2_ctrl_new_std(handler, &gc2145_ctrl_ops,
					      V4L2_CID_GAIN, GC2145_GAIN_MIN,
					      GC2145_GAIN_MAX, GC2145_GAIN_STEP,
					      GC2145_GAIN_DEFAULT);

	gc2145->h_flip = v4l2_ctrl_new_std(handler, &gc2145_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	gc2145->v_flip = v4l2_ctrl_new_std(handler, &gc2145_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc2145->client->dev, "Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc2145->subdev.ctrl_handler = handler;
	gc2145->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc2145_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC2145_XVCLK_FREQ / 1000 / 1000);
}

static int __gc2145_power_on(struct gc2145 *gc2145)
{
	u32 delay_us;

	if (!IS_ERR(gc2145->pwdn_gpio)) {
		gpiod_set_value_cansleep(gc2145->pwdn_gpio, 0);//exit powerdown mode
		usleep_range(500, 800);
    }

	if (!IS_ERR(gc2145->reset_gpio)) {
		gpiod_set_value_cansleep(gc2145->reset_gpio, 0);//exit reset mode
		usleep_range(3000, 6000);
	}
	if (!IS_ERR(gc2145->reset_gpio)) {
		gpiod_set_value_cansleep(gc2145->reset_gpio, 1);//enter reset mode
		usleep_range(3000, 6000);
    }

	if (!IS_ERR(gc2145->reset_gpio))
		gpiod_set_value_cansleep(gc2145->reset_gpio, 0);//exit reset mode

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc2145_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;
}


static void __gc2145_power_off(struct gc2145 *gc2145)
{
	if (!IS_ERR(gc2145->reset_gpio))
		gpiod_set_value_cansleep(gc2145->reset_gpio, 1);
	if (!IS_ERR(gc2145->pwdn_gpio))
		gpiod_set_value_cansleep(gc2145->pwdn_gpio, 0);
}

static int gc2145_check_sensor_id(struct gc2145 *gc2145)
{
	struct device *dev = &gc2145->client->dev;
	u8 id_h = 0, id_l = 0;
	u16 id = 0;
	int ret = 0;

	ret = gc2145_read_reg(gc2145->client, GC2145_REG_CHIP_ID_H, &id_h);
	ret |= gc2145_read_reg(gc2145->client, GC2145_REG_CHIP_ID_L, &id_l);
	if (ret) {
		dev_err(dev, "Failed to read sensor id, (%d)\n", ret);
		return ret;
	}

	id = id_h << 8 | id_l;
	if (id != GC2145_CHIP_ID) {
		dev_err(dev, "sensor id: %04X mismatched\n", id);
		return -ENODEV;
	}

	dev_info(dev, "Detected GC2145 sensor\n");
	return 0;
}

static long gc2145_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	long ret = 0;

	dev_info(&gc2145->client->dev, "gc2145_ioctl\n");

	switch (cmd) {
	//case:
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static int __gc2145_start_stream(struct gc2145 *gc2145)
{
	int ret;

	ret = gc2145_write_array(gc2145->client, gc2145->cur_mode->reg_list);
	if (ret)
		return ret;
	usleep_range(2000, 5000);

	/* Apply customized control from user */
	mutex_unlock(&gc2145->lock);
	ret = v4l2_ctrl_handler_setup(&gc2145->ctrl_handler);
	mutex_lock(&gc2145->lock);

	ret = gc2145_write_array(gc2145->client, gc2145_start);
	if (ret)
		return ret;

	return ret;
}

static int __gc2145_stop_stream(struct gc2145 *gc2145)
{
	int ret;

	ret = gc2145_write_array(gc2145->client, gc2145_stop);
	if (ret)
		return ret;

	return ret;
}

#ifdef CONFIG_COMPAT
static long gc2145_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = gc2145_ioctl(sd, cmd, inf);
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

		ret = gc2145_ioctl(sd, cmd, awb_cfg);
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

static int gc2145_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	int ret = 0;
	dev_info(&gc2145->client->dev, "%s enter\n",__func__);

	mutex_lock(&gc2145->lock);
	on = !!on;
	if (on == gc2145->streaming)
		goto unlock_and_return;

	if (on) {
		ret = __gc2145_start_stream(gc2145);
		if (ret) {
			dev_err(&gc2145->client->dev, "Failed to start gc2145 stream\n");
			goto unlock_and_return;
		}
	} else {
		__gc2145_stop_stream(gc2145);
	}

	gc2145->streaming = on;

unlock_and_return:
	mutex_unlock(&gc2145->lock);
	dev_info(&gc2145->client->dev, "%s exit\n",__func__);

	return 0;
}

static int gc2145_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	const struct gc2145_mode *mode = gc2145->cur_mode;

	mutex_lock(&gc2145->lock);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc2145->lock);

	return 0;
}

static int gc2145_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2;
	config->flags = 1 << (GC2145_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int gc2145_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = GC2145_MEDIA_BUS_FMT;
	return 0;
}

static int gc2145_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc2145 *gc2145 = to_gc2145(sd);

	if (fse->index >= gc2145->cfg_num)
		return -EINVAL;

	if (fse->code != GC2145_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int gc2145_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc2145 *gc2145 = to_gc2145(sd);

	if (fie->index >= gc2145->cfg_num)
		return -EINVAL;

	fie->code = GC2145_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int gc2145_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	const struct gc2145_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc2145->lock);

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	dev_info(&gc2145->client->dev, "set_fmt, width:%u, height:%u.\n", mode->width, mode->height);

	fmt->format.code = GC2145_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc2145->lock);
		return -ENOTTY;
#endif
	} else {
		gc2145->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(gc2145->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(gc2145->pixel_rate,
					 to_pixel_rate(mode->link_freq_index));
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc2145->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc2145->vblank, vblank_def,
					 GC2145_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc2145->lock);
	return 0;
}

static int gc2145_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	const struct gc2145_mode *mode = gc2145->cur_mode;

	mutex_lock(&gc2145->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc2145->lock);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC2145_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc2145->lock);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc2145_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc2145_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc2145->lock);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC2145_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&gc2145->lock);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc2145_internal_ops = {
	.open = gc2145_open,
};
#endif

static int gc2145_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc2145 *gc2145 = to_gc2145(sd);
	int ret = 0;

	mutex_lock(&gc2145->lock);

	if (gc2145->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		gc2145->power_on = true;
	} else {
		gc2145->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc2145->lock);

	return ret;
}

static const struct v4l2_subdev_core_ops gc2145_core_ops = {
	.s_power = gc2145_s_power,
	.ioctl = gc2145_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc2145_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc2145_video_ops = {
	.s_stream = gc2145_s_stream,
	.g_frame_interval = gc2145_g_frame_interval,
	.g_mbus_config = gc2145_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gc2145_pad_ops = {
	.enum_mbus_code = gc2145_enum_mbus_code,
	.enum_frame_size = gc2145_enum_frame_sizes,
	.enum_frame_interval = gc2145_enum_frame_interval,
	.get_fmt = gc2145_get_fmt,
	.set_fmt = gc2145_set_fmt,
};

static const struct v4l2_subdev_ops gc2145_subdev_ops = {
	.core   = &gc2145_core_ops,
	.video  = &gc2145_video_ops,
	.pad    = &gc2145_pad_ops,
};

static int gc2145_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc2145 *gc2145;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	gc2145 = devm_kzalloc(dev, sizeof(*gc2145), GFP_KERNEL);
	if (!gc2145)
		return -ENOMEM;

	gc2145->client = client;
	ret = of_property_read_u32(node, CANAANMODULE_CAMERA_MODULE_INDEX,
				   &gc2145->module_index);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_FACING,
				       &gc2145->module_facing);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_NAME,
				       &gc2145->module_name);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_LENS_NAME,
				       &gc2145->len_name);

	if (ret) {
		dev_err(dev, "Failed to get module information\n");
		return -EINVAL;
	}

	gc2145->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);//reset pin low actvie, set logical value.
	if (IS_ERR(gc2145->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc2145->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_HIGH);//pwdn pin high active, set logical value.
	if (IS_ERR(gc2145->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	if (gpiod_export(gc2145->reset_gpio, 0)) {
		dev_err(dev, "gpiod export gc2145 reset failed.");
	}

	if (gpiod_export(gc2145->pwdn_gpio, 0)) {
		dev_err(dev, "gpiod export gc2145 powerdown failed.");
	}

	mutex_init(&gc2145->lock);

	/* set default mode */
	gc2145->cur_mode = &supported_modes[0];
	gc2145->cfg_num = ARRAY_SIZE(supported_modes);

	sd = &gc2145->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc2145_subdev_ops);
	ret = gc2145_initialize_controls(gc2145);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc2145_power_on(gc2145);
	if (ret)
		goto err_free_handler;

	ret = gc2145_check_sensor_id(gc2145);
	if (ret)
		goto err_power_off;

	sd->internal_ops = &gc2145_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	gc2145->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc2145->pad);
	if (ret < 0)
		goto err_power_off;

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc2145->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc2145->module_index, facing,
		 GC2145_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "Failed to register v4l2 async subdev\n");
		goto err_clean_entity;
	}

	return 0;

err_clean_entity:
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__gc2145_power_off(gc2145);
err_free_handler:
	v4l2_ctrl_handler_free(&gc2145->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc2145->lock);

	return ret;
}

static int gc2145_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2145 *gc2145 = to_gc2145(sd);

	v4l2_async_unregister_subdev(sd);
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc2145->ctrl_handler);
	mutex_destroy(&gc2145->lock);

	__gc2145_power_off(gc2145);
	return 0;
}

static const struct i2c_device_id gc2145_match_id[] = {
	{ "gc2145", 0 },
	{ },
};

static const struct of_device_id gc2145_of_match[] = {
	{ .compatible = "galaxycore,gc2145" },
	{},
};
MODULE_DEVICE_TABLE(of, gc2145_of_match);

static struct i2c_driver gc2145_i2c_driver = {
	.driver = {
		.name = GC2145_NAME,
		.of_match_table = of_match_ptr(gc2145_of_match),
	},
	.probe      = &gc2145_probe,
	.remove     = &gc2145_remove,
	.id_table   = gc2145_match_id,
};

module_i2c_driver(gc2145_i2c_driver);
MODULE_DESCRIPTION("Galaxycore GC2145 Image Sensor driver");
MODULE_LICENSE("GPL v2");
