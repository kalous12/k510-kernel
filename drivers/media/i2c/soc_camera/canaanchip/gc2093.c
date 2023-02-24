/*
 * Driver for GC2093 CMOS Image Sensor
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
#define GC2093_NAME		"gc2093"
#define GC2093_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SRGGB10_1X10

#define MIPI_FREQ_297M		297000000
#define MIPI_FREQ_396M		396000000

#define GC2093_XVCLK_FREQ	24000000

#define GC2093_REG_CHIP_ID_H	0x03F0
#define GC2093_REG_CHIP_ID_L	0x03F1

#define GC2093_REG_EXP_SHORT_H	0x0001
#define GC2093_REG_EXP_SHORT_L	0x0002
#define GC2093_REG_EXP_LONG_H	0x0003
#define GC2093_REG_EXP_LONG_L	0x0004

#define GC2093_MIRROR_FLIP_REG	0x0017
#define MIRROR_MASK		BIT(0)
#define FLIP_MASK		BIT(1)

#define GC2093_REG_CTRL_MODE	0x003E
#define GC2093_MODE_SW_STANDBY	0x11
#define GC2093_MODE_STREAMING	0x91

#define GC2093_CHIP_ID		0x2093

#define GC2093_VTS_MAX		0x3FFF
#define GC2093_HTS_MAX		0xFFF

#define GC2093_EXPOSURE_MAX	0x3FFF
#define GC2093_EXPOSURE_MIN	1
#define GC2093_EXPOSURE_STEP	1

#define GC2093_GAIN_MIN		0x40
#define GC2093_GAIN_MAX		0x2000
#define GC2093_GAIN_STEP	1
#define GC2093_GAIN_DEFAULT	64

#define GC2093_LANES		2

#define REG_NULL			0xFFFF

#define to_gc2093(sd) container_of(sd, struct gc2093, subdev)

enum {
	LINK_FREQ_297M_INDEX,
	LINK_FREQ_396M_INDEX,
};

struct gc2093_reg {
	u16 addr;
	u8 val;
};

struct gain_reg_config {
	u32 value;
	u16 analog_gain;
	u16 col_gain;
	u16 analog_sw;
	u16 ram_width;
};

struct gc2093_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 link_freq_index;
	const struct gc2093_reg *reg_list;
};

struct gc2093 {
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
	const struct gc2093_mode *cur_mode;

	u32		module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;

	bool			  has_init_exp;
	u8			flip;
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_297M,
	MIPI_FREQ_396M,
};

/*
 * window size=800*1080 mipi@2lane
 * mclk=24M mipi_clk=792Mbps
 * pixel_line_total=2640  line_frame_total=1125
 * row_time=29.62us frame_rate=30fps
 */
static const struct gc2093_reg gc2093_800x1080_linear_30fps_settings[] = {
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x11},
	{0x03f8, 0x30},
	{0x03f9, 0x42},
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0003, 0x04}, //shutter time[13:8]
	{0x0004, 0x60}, //shutter time[7:0]
	{0x0005, 0x05}, //Line length[11:8]
	{0x0006, 0x8e}, //Line length[7:0]
	{0x0007, 0x00}, //VB[13:8]
	{0x0008, 0x11}, //VB[7:0]
	{0x0009, 0x00}, //Row Start[10:8]
	{0x000a, 0x02}, //Row Start[7:0]
	{0x000b, 0x00}, //Col Start[10:8]
	{0x000c, 0x04}, //Col Start[7:0]
	{0x000d, 0x04}, //Window height[10:8] 1088
	{0x000e, 0x40}, //Window height[7:0]
	{0x000f, 0x07}, //Window width[11:8] 1932
	{0x0010, 0x8c}, //Window width[7:0]
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x04}, //FrameLength[13:8] 0465
	{0x0042, 0x65}, //FrameLength[7:0]
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x40},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x30},
	{0x004a, 0x01},
	{0x004b, 0x28},
	{0x0055, 0x30},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x68},
	{0x00b3, 0x00},
	{0x00b8, 0x01},
	{0x00b9, 0x00},
	{0x00b1, 0x01},
	{0x00b2, 0x00},
	/*isp*/
	{0x0101, 0x0c},
	{0x0102, 0x89},
	{0x0104, 0x01},
	{0x0107, 0xa6},
	{0x0108, 0xa9},
	{0x0109, 0xa8},
	{0x010a, 0xa7},
	{0x010b, 0xff},
	{0x010c, 0xff},
	{0x010f, 0x00},
	{0x0158, 0x00},
	{0x0428, 0x86},
	{0x0429, 0x86},
	{0x042a, 0x86},
	{0x042b, 0x68},
	{0x042c, 0x68},
	{0x042d, 0x68},
	{0x042e, 0x68},
	{0x042f, 0x68},
	{0x0430, 0x4f},
	{0x0431, 0x68},
	{0x0432, 0x67},
	{0x0433, 0x66},
	{0x0434, 0x66},
	{0x0435, 0x66},
	{0x0436, 0x66},
	{0x0437, 0x66},
	{0x0438, 0x62},
	{0x0439, 0x62},
	{0x043a, 0x62},
	{0x043b, 0x62},
	{0x043c, 0x62},
	{0x043d, 0x62},
	{0x043e, 0x62},
	{0x043f, 0x62},
	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x04},
	{0x0122, 0x65},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},
	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x0f},
	{0x014b, 0x00},
	{0x0155, 0x07},
	{0x0414, 0x78},
	{0x0415, 0x78},
	{0x0416, 0x78},
	{0x0417, 0x78},
	{0x04e0, 0x18},
	/*window*/
	{0x0191, 0x00}, //Out Window Y1[10:8]
	{0x0192, 0x02}, //Out Window Y1[7:0]
	{0x0193, 0x02}, //Out Window X1[11:8]
	{0x0194, 0x2f}, //Out Window X1[7:0]
	{0x0195, 0x04}, //Out Window Height[10:8] 1080=0438
	{0x0196, 0x38}, //Out Window Height[7:0]
	{0x0197, 0x03}, //Out Window Width[11:8]  1920=0780
	{0x0198, 0x20}, //Out Window Width[7:0] 800=0320
	/****DVP & MIPI****/
	{0x019a, 0x06},
	{0x007b, 0x2a},
	{0x0023, 0x2d},
	{0x0201, 0x27},
	{0x0202, 0x56},
	{0x0203, 0xce},
	{0x0212, 0x20},
	{0x0213, 0x03},
	{0x0215, 0x10},

	{REG_NULL, 0x00},
};

/*
 * window size=1920*1080 mipi@2lane
 * mclk=24M mipi_clk=792Mbps
 * pixel_line_total=  line_frame_total=1125
 * row_time=29.62us frame_rate=30fps
 */
static const struct gc2093_reg gc2093_1080p_linear_30fps_settings[] = {
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x11},
	{0x03f8, 0x30},
	{0x03f9, 0x42},
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0003, 0x04}, //shutter time[13:8]
	{0x0004, 0x60}, //shutter time[7:0]
	{0x0005, 0x05}, //Line length[11:8]
	{0x0006, 0x8e}, //Line length[7:0]
	{0x0007, 0x00}, //VB[13:8]
	{0x0008, 0x11}, //VB[7:0]
	{0x0009, 0x00}, //Row Start[10:8]
	{0x000a, 0x02}, //Row Start[7:0]
	{0x000b, 0x00}, //Col Start[10:8]
	{0x000c, 0x04}, //Col Start[7:0]
	{0x000d, 0x04}, //Window height[10:8] 1088
	{0x000e, 0x40}, //Window height[7:0]
	{0x000f, 0x07}, //Window width[11:8] 1932
	{0x0010, 0x8c}, //Window width[7:0]
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x04}, //FrameLength[13:8] 0465
	{0x0042, 0x65}, //FrameLength[7:0]
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x40},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x30},
	{0x004a, 0x01},
	{0x004b, 0x28},
	{0x0055, 0x30},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x68},
	{0x00b3, 0x00},
	{0x00b8, 0x01},
	{0x00b9, 0x00},
	{0x00b1, 0x01},
	{0x00b2, 0x00},
	/*isp*/
	{0x0101, 0x0c},
	{0x0102, 0x89},
	{0x0104, 0x01},
	{0x0107, 0xa6},
	{0x0108, 0xa9},
	{0x0109, 0xa8},
	{0x010a, 0xa7},
	{0x010b, 0xff},
	{0x010c, 0xff},
	{0x010f, 0x00},
	{0x0158, 0x00},
	{0x0428, 0x86},
	{0x0429, 0x86},
	{0x042a, 0x86},
	{0x042b, 0x68},
	{0x042c, 0x68},
	{0x042d, 0x68},
	{0x042e, 0x68},
	{0x042f, 0x68},
	{0x0430, 0x4f},
	{0x0431, 0x68},
	{0x0432, 0x67},
	{0x0433, 0x66},
	{0x0434, 0x66},
	{0x0435, 0x66},
	{0x0436, 0x66},
	{0x0437, 0x66},
	{0x0438, 0x62},
	{0x0439, 0x62},
	{0x043a, 0x62},
	{0x043b, 0x62},
	{0x043c, 0x62},
	{0x043d, 0x62},
	{0x043e, 0x62},
	{0x043f, 0x62},
	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x04},
	{0x0122, 0x65},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},
	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x0f},
	{0x014b, 0x00},
	{0x0155, 0x07},
	{0x0414, 0x78},
	{0x0415, 0x78},
	{0x0416, 0x78},
	{0x0417, 0x78},
	{0x04e0, 0x18},
	/*window*/
	{0x0192, 0x02}, //Out Window Y1[7:0]
	{0x0194, 0x03}, //Out Window X1[7:0]
	{0x0195, 0x04}, //Out Window Height[10:8]
	{0x0196, 0x38}, //Out Window Height[7:0]
	{0x0197, 0x07}, //Out Window Width[11:8]
	{0x0198, 0x80}, //Out Window Width[7:0]
	/****DVP & MIPI****/
	{0x019a, 0x06},
	{0x007b, 0x2a},
	{0x0023, 0x2d},
	{0x0201, 0x27},
	{0x0202, 0x56},
	{0x0203, 0xce},
	{0x0212, 0x80},
	{0x0213, 0x07},
	{0x0215, 0x10},
	{REG_NULL, 0x00},
};

/*
 * window size=1920*1080 mipi@2lane
 * mclk=24M mipi_clk=792Mbps
 * pixel_line_total=2640  line_frame_total=1250
 * row_time=13.3333us frame_rate=60fps
 */
static const struct gc2093_reg gc2093_1080p_linear_60fps_settings[] = {
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x01},
	{0x03f8, 0x63},
	{0x03f9, 0x40},
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x04}, //shutter time[13:8]
	{0x0004, 0x02}, //shutter time[7:0]
	{0x0005, 0x02}, //Line length[11:8]
	{0x0006, 0x94}, //Line length[7:0]
	{0x0007, 0x00}, //VB[13:8]
	{0x0008, 0x11}, //VB[7:0]
	{0x0009, 0x00}, //Row Start[10:8]
	{0x000a, 0x02}, //Row Start[7:0]
	{0x000b, 0x00}, //Col Start[10:8]
	{0x000c, 0x04}, //Col Start[7:0]
	{0x000d, 0x04}, //Window height[10:8] 1088
	{0x000e, 0x40}, //Window height[7:0]
	{0x000f, 0x07}, //Window width[11:8] 1932
	{0x0010, 0x8c}, //Window width[7:0]
	{0x0013, 0x15},
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x04}, //FrameLength[13:8] 09c4=2500
	{0x0042, 0xe2}, //FrameLength[7:0]
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x28},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x28},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x68},//0x60
	/*isp*/
	{0x0101, 0x0c},
	{0x0102, 0x89},
	{0x0104, 0x01},
	{0x010f, 0x00},
	{0x0158, 0x00},
	{/*dark sun*/},
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x04},
	{0x0122, 0xd8},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},
	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x0f},
	{0x014b, 0x00},
	{0x0155, 0x07},
	{0x0414, 0x78},
	{0x0415, 0x78},
	{0x0416, 0x78},
	{0x0417, 0x78},
	{0x0454, 0x78},
	{0x0455, 0x78},
	{0x0456, 0x78},
	{0x0457, 0x78},
	{0x04e0, 0x18},
	/*window*/
	{0x0192, 0x02}, //Out Window Y1[7:0]
	{0x0194, 0x03}, //Out Window X1[7:0]
	{0x0195, 0x04}, //Out Window Height[10:8]
	{0x0196, 0x38}, //Out Window Height[7:0]
	{0x0197, 0x07}, //Out Window Width[11:8]
	{0x0198, 0x80}, //Out Window Width[7:0]
	/****DVP & MIPI****/
	{0x019a, 0x06},
	{0x007b, 0x2a},
	{0x0023, 0x2d},
	{0x0201, 0x27},
	{0x0202, 0x56},
	{0x0203, 0xce},
	{0x0212, 0x80},
	{0x0213, 0x07},
	{0x0215, 0x10},
	{REG_NULL, 0x00},
};

/*
 * window size=1920*1080 mipi@2lane
 * mclk=24M mipi_clk=792Mbps
 * pixel_line_total=2640  line_frame_total=1250
 * row_time=13.3333us frame_rate=30fps
 */
static const struct gc2093_reg gc2093_1080p_hdr_30fps_settings[] = {
	/****system****/
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0B},
	{0x03f7, 0x01},
	{0x03f8, 0x63},
	{0x03f9, 0x40},
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x04}, //shutter time[13:8]
	{0x0004, 0x02}, //shutter time[7:0]
	{0x0005, 0x02}, //Line length[11:8]
	{0x0006, 0x94}, //Line length[7:0]
	{0x0007, 0x00}, //VB[13:8]
	{0x0008, 0x11}, //VB[7:0]
	{0x0009, 0x00}, //Row Start[10:8]
	{0x000a, 0x02}, //Row Start[7:0]
	{0x000b, 0x00}, //Col Start[10:8]
	{0x000c, 0x04}, //Col Start[7:0]
	{0x000d, 0x04}, //Window height[10:8] 1088
	{0x000e, 0x40}, //Window height[7:0]
	{0x000f, 0x07}, //Window width[11:8] 1932
	{0x0010, 0x8c}, //Window width[7:0]
	{0x0013, 0x15},
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x04}, //FrameLength[13:8] 09c4=2500 04e2=1250
	{0x0042, 0xe2}, //FrameLength[7:0]
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x28},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x28},
	{0x0066, 0x3f},
	{0x0068, 0x3f},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x68},//0x60
	/*isp*/
	{0x0101, 0x0c},
	{0x0102, 0x89},
	{0x0104, 0x01},
	{0x010e, 0x01},
	{0x010f, 0x00},
	{0x0158, 0x00},
	{/*dark sun*/},
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x01},
	{0x0121, 0x04},
	{0x0122, 0xd8},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},
	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x0f},
	{0x014b, 0x00},
	{0x0155, 0x07},
	{0x0414, 0x78},
	{0x0415, 0x78},
	{0x0416, 0x78},
	{0x0417, 0x78},
	{0x0454, 0x78},
	{0x0455, 0x78},
	{0x0456, 0x78},
	{0x0457, 0x78},
	{0x04e0, 0x18},
	/*window*/
	{0x0192, 0x02}, //Out Window Y1[7:0]
	{0x0194, 0x03}, //Out Window X1[7:0]
	{0x0195, 0x04}, //Out Window Height[10:8]
	{0x0196, 0x38}, //Out Window Height[7:0]
	{0x0197, 0x07}, //Out Window Width[11:8]
	{0x0198, 0x80}, //Out Window Width[7:0]
	/****DVP & MIPI****/
	{0x019a, 0x06},
	{0x007b, 0x2a},
	{0x0023, 0x2d},
	{0x0201, 0x27},
	{0x0202, 0x5f},
	{0x0203, 0xce},
	{0x0212, 0x80},
	{0x0213, 0x07},
	{0x0215, 0x10},
	/****HDR EN****/
	{0x0027, 0x71},
	{0x0215, 0x92},
	{0x024d, 0x01},
	//{0x001a, 0x9c}, //exp_exp2_th_mode
	//{0x005a, 0x00}, //exp_exp2_th
	//{0x005b, 0x49}, //exp_exp2_th
	{REG_NULL, 0x00},
};

static const struct gc2093_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x460,
		.hts_def = 0x898,
		.vts_def = 0x465,//0x9c4,//0x465,
		.link_freq_index = LINK_FREQ_396M_INDEX,
		.reg_list = gc2093_1080p_linear_30fps_settings,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x460,
		.hts_def = 0xa50,
		.vts_def = 0x4e2,
		.link_freq_index = LINK_FREQ_396M_INDEX,
		.reg_list = gc2093_1080p_linear_60fps_settings,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x460,
		.hts_def = 0xa50,
		.vts_def = 0x4e2,
		.link_freq_index = LINK_FREQ_396M_INDEX,
		.reg_list = gc2093_1080p_hdr_30fps_settings,
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
		.vts_def = 0x465,//0x9c4,//0x465,
		.link_freq_index = LINK_FREQ_396M_INDEX,
		.reg_list = gc2093_800x1080_linear_30fps_settings,
	},
};

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
static u64 to_pixel_rate(u32 index)
{
	u64 pixel_rate = link_freq_menu_items[index] * 2 * GC2093_LANES;

	do_div(pixel_rate, 10);

	return pixel_rate;
}

/* sensor register write */
static int gc2093_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev, "write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int gc2093_write_array(struct i2c_client *client,
				  const struct gc2093_reg *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = gc2093_write_reg(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

static int reg16_write(struct i2c_client *client, const u16 addr, const u16 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[4];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 4;
	msg.flags = 0;
	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data >> 8;
	tx[3] = data & 0xff;
	ret = i2c_transfer(adap, &msg, 1);
	udelay(20);//mdelay(2);

	return ret == 1 ? 0 : -EIO;
}

/* sensor register read */
static int gc2093_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	u8 buf[2] = {reg >> 8, reg & 0xff};
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}, {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev, "read reg(0x%x val:0x%x) failed !\n", reg, *val);

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

static int gc2093_set_gain(struct gc2093 *gc2093, u32 gain)
{
	int ret, i = 0;
	//u16 pre_gain = 0;

	//for (i = 0; i < ARRAY_SIZE(gain_reg_configs) - 1; i++)
	//	if ((gain_reg_configs[i].value <= gain) && (gain < gain_reg_configs[i+1].value))
	//		break;

	//ret = gc2093_write_reg(gc2093->client, 0x00b4, gain_reg_configs[i].analog_gain >> 8);
	//ret |= gc2093_write_reg(gc2093->client, 0x00b3, gain_reg_configs[i].analog_gain & 0xff);
	//ret |= gc2093_write_reg(gc2093->client, 0x00b8, gain_reg_configs[i].col_gain >> 8);
	//ret |= gc2093_write_reg(gc2093->client, 0x00b9, gain_reg_configs[i].col_gain & 0xff);
	//ret |= gc2093_write_reg(gc2093->client, 0x00ce, gain_reg_configs[i].analog_sw >> 8);
	//ret |= gc2093_write_reg(gc2093->client, 0x00c2, gain_reg_configs[i].analog_sw & 0xff);
	//ret |= gc2093_write_reg(gc2093->client, 0x00cf, gain_reg_configs[i].ram_width >> 8);
	//ret |= gc2093_write_reg(gc2093->client, 0x00d9, gain_reg_configs[i].ram_width & 0xff);

	//pre_gain = 64 * gain / gain_reg_configs[i].value;

	ret |= gc2093_write_reg(gc2093->client, 0x00b1, (gain >> 8));
	ret |= gc2093_write_reg(gc2093->client, 0x00b2, (gain & 0xfc));

	return ret;
}

static int gc2093_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2093 *gc2093 = container_of(ctrl->handler,
					     struct gc2093, ctrl_handler);
	s64 max;
	int ret = 0;
	dev_dbg(&gc2093->client->dev, "%s enter, id:0x%X val:%d.\n", __func__, ctrl->id, ctrl->val);

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc2093->cur_mode->height + ctrl->val - 1;
		__v4l2_ctrl_modify_range(gc2093->exposure,
					 gc2093->exposure->minimum, max,
					 gc2093->exposure->step,
					 gc2093->exposure->default_value);
		break;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		//ret = gc2093_write_reg(gc2093->client, GC2093_REG_EXP_LONG_H,
		//		       (ctrl->val >> 8) & 0x3f);
		//ret |= gc2093_write_reg(gc2093->client, GC2093_REG_EXP_LONG_L,
		//			ctrl->val & 0xff);
		ret = reg16_write(gc2093->client,  GC2093_REG_EXP_LONG_H, ctrl->val);
		//dev_info(&gc2093->client->dev, "V4L2_CID_EXPOSURE, val=%d\n", ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
		//gc2093_set_gain(gc2093, ctrl->val);
		ret = reg16_write(gc2093->client, 0x00b1, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		/* The exposure goes up and reduces the frame rate, no need to write vb */
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			gc2093->flip |= MIRROR_MASK;
		else
			gc2093->flip &= ~MIRROR_MASK;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			gc2093->flip |= FLIP_MASK;
		else
			gc2093->flip &= ~FLIP_MASK;
		break;
	default:
		dev_warn(&gc2093->client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops gc2093_ctrl_ops = {
	.s_ctrl = gc2093_set_ctrl,
};

static int gc2093_initialize_controls(struct gc2093 *gc2093)
{
	const struct gc2093_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc2093->ctrl_handler;
	mode = gc2093->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 10);
	if (ret)
		return ret;
	handler->lock = &gc2093->lock;

	gc2093->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq_menu_items) - 1, 0,
						   link_freq_menu_items);

	gc2093->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
					       0, to_pixel_rate(LINK_FREQ_396M_INDEX),
					       1, to_pixel_rate(LINK_FREQ_297M_INDEX));

	h_blank = mode->hts_def - mode->width;
	gc2093->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (gc2093->hblank)
		gc2093->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc2093->vblank = v4l2_ctrl_new_std(handler, &gc2093_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   GC2093_VTS_MAX - mode->height,
					   1, vblank_def);

	exposure_max = mode->vts_def - 1;
	gc2093->exposure = v4l2_ctrl_new_std(handler, &gc2093_ctrl_ops,
					     V4L2_CID_EXPOSURE, GC2093_EXPOSURE_MIN,
					     exposure_max, GC2093_EXPOSURE_STEP,
					     mode->exp_def);

	gc2093->anal_gain = v4l2_ctrl_new_std(handler, &gc2093_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, GC2093_GAIN_MIN,
					      GC2093_GAIN_MAX, GC2093_GAIN_STEP,
					      GC2093_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &gc2093_ctrl_ops,
					      V4L2_CID_GAIN, GC2093_GAIN_MIN,
					      GC2093_GAIN_MAX, GC2093_GAIN_STEP,
					      GC2093_GAIN_DEFAULT);

	gc2093->h_flip = v4l2_ctrl_new_std(handler, &gc2093_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	gc2093->v_flip = v4l2_ctrl_new_std(handler, &gc2093_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	gc2093->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc2093->client->dev, "Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc2093->subdev.ctrl_handler = handler;
	gc2093->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc2093_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC2093_XVCLK_FREQ / 1000 / 1000);
}

static int __gc2093_power_on(struct gc2093 *gc2093)
{
	u32 delay_us;

	if (!IS_ERR(gc2093->reset_gpio)) {
		gpiod_set_value_cansleep(gc2093->reset_gpio, 1);
		usleep_range(100, 200);
	}
	if (!IS_ERR(gc2093->pwdn_gpio))
		gpiod_set_value_cansleep(gc2093->pwdn_gpio, 0);

	if (!IS_ERR(gc2093->reset_gpio))
		gpiod_set_value_cansleep(gc2093->reset_gpio, 0);
	usleep_range(3000, 6000);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc2093_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;
}


static void __gc2093_power_off(struct gc2093 *gc2093)
{
	if (!IS_ERR(gc2093->reset_gpio))
		gpiod_set_value_cansleep(gc2093->reset_gpio, 1);
	if (!IS_ERR(gc2093->pwdn_gpio))
		gpiod_set_value_cansleep(gc2093->pwdn_gpio, 0);
}

static int gc2093_check_sensor_id(struct gc2093 *gc2093)
{
	struct device *dev = &gc2093->client->dev;
	u8 id_h = 0, id_l = 0;
	u16 id = 0;
	int ret = 0;

	ret = gc2093_read_reg(gc2093->client, GC2093_REG_CHIP_ID_H, &id_h);
	ret |= gc2093_read_reg(gc2093->client, GC2093_REG_CHIP_ID_L, &id_l);
	if (ret) {
		dev_err(dev, "Failed to read sensor id, (%d)\n", ret);
		return ret;
	}

	id = id_h << 8 | id_l;
	if (id != GC2093_CHIP_ID) {
		dev_err(dev, "sensor id: %04X mismatched\n", id);
		return -ENODEV;
	}

	dev_info(dev, "Detected GC2093 sensor\n");
	return 0;
}

static long gc2093_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	long ret = 0;

	dev_info(&gc2093->client->dev, "gc2093_ioctl\n");

	switch (cmd) {
	//case:
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static int gc2093_set_flip(struct gc2093 *gc2093, u8 mode)
{
	return gc2093_write_reg(gc2093->client,  GC2093_MIRROR_FLIP_REG, mode);
}

static int __gc2093_start_stream(struct gc2093 *gc2093)
{
	int ret;
	dev_info(&gc2093->client->dev, "%s enter\n",__func__);

	ret = gc2093_write_array(gc2093->client, gc2093->cur_mode->reg_list);
	if (ret)
		return ret;

	/* Apply customized control from user */
	mutex_unlock(&gc2093->lock);
	v4l2_ctrl_handler_setup(&gc2093->ctrl_handler);
	mutex_lock(&gc2093->lock);

	ret = gc2093_set_flip(gc2093, gc2093->flip);
	if (ret)
		return ret;

	return gc2093_write_reg(gc2093->client, GC2093_REG_CTRL_MODE,
				GC2093_MODE_STREAMING);
}

static int __gc2093_stop_stream(struct gc2093 *gc2093)
{
	gc2093->has_init_exp = false;
	return gc2093_write_reg(gc2093->client, GC2093_REG_CTRL_MODE,
				GC2093_MODE_SW_STANDBY);
}

#ifdef CONFIG_COMPAT
static long gc2093_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = gc2093_ioctl(sd, cmd, inf);
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

		ret = gc2093_ioctl(sd, cmd, awb_cfg);
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

static int gc2093_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	int ret = 0;

	mutex_lock(&gc2093->lock);
	on = !!on;
	if (on == gc2093->streaming)
		goto unlock_and_return;

	if (on) {
		ret = __gc2093_start_stream(gc2093);
		if (ret) {
			dev_err(&gc2093->client->dev, "Failed to start gc2093 stream\n");
			goto unlock_and_return;
		}
	} else {
		__gc2093_stop_stream(gc2093);
	}

	gc2093->streaming = on;

unlock_and_return:
	mutex_unlock(&gc2093->lock);
	return 0;
}

static int gc2093_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	const struct gc2093_mode *mode = gc2093->cur_mode;

	mutex_lock(&gc2093->lock);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc2093->lock);

	return 0;
}

static int gc2093_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	u32 val = 1 << (GC2093_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = (val | V4L2_MBUS_CSI2_CHANNEL_1);

	return 0;
}

static int gc2093_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = GC2093_MEDIA_BUS_FMT;
	return 0;
}

static int gc2093_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc2093 *gc2093 = to_gc2093(sd);

	if (fse->index >= gc2093->cfg_num)
		return -EINVAL;

	if (fse->code != GC2093_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int gc2093_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc2093 *gc2093 = to_gc2093(sd);

	if (fie->index >= gc2093->cfg_num)
		return -EINVAL;

	fie->code = GC2093_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int gc2093_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	const struct gc2093_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc2093->lock);

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	dev_info(&gc2093->client->dev, "set_fmt, width:%u, height:%u.\n", mode->width, mode->height);

	fmt->format.code = GC2093_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc2093->lock);
		return -ENOTTY;
#endif
	} else {
		gc2093->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(gc2093->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(gc2093->pixel_rate,
					 to_pixel_rate(mode->link_freq_index));
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc2093->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc2093->vblank, vblank_def,
					 GC2093_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc2093->lock);
	return 0;
}

static int gc2093_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	const struct gc2093_mode *mode = gc2093->cur_mode;

	mutex_lock(&gc2093->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc2093->lock);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC2093_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc2093->lock);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc2093_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc2093_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc2093->lock);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC2093_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&gc2093->lock);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc2093_internal_ops = {
	.open = gc2093_open,
};
#endif

static int gc2093_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc2093 *gc2093 = to_gc2093(sd);
	int ret = 0;

	mutex_lock(&gc2093->lock);

	if (gc2093->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		gc2093->power_on = true;
	} else {
		gc2093->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc2093->lock);

	return ret;
}

static const struct v4l2_subdev_core_ops gc2093_core_ops = {
	.s_power = gc2093_s_power,
	.ioctl = gc2093_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc2093_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc2093_video_ops = {
	.s_stream = gc2093_s_stream,
	.g_frame_interval = gc2093_g_frame_interval,
	.g_mbus_config = gc2093_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gc2093_pad_ops = {
	.enum_mbus_code = gc2093_enum_mbus_code,
	.enum_frame_size = gc2093_enum_frame_sizes,
	.enum_frame_interval = gc2093_enum_frame_interval,
	.get_fmt = gc2093_get_fmt,
	.set_fmt = gc2093_set_fmt,
};

static const struct v4l2_subdev_ops gc2093_subdev_ops = {
	.core   = &gc2093_core_ops,
	.video  = &gc2093_video_ops,
	.pad    = &gc2093_pad_ops,
};

static int gc2093_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc2093 *gc2093;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	gc2093 = devm_kzalloc(dev, sizeof(*gc2093), GFP_KERNEL);
	if (!gc2093)
		return -ENOMEM;

	gc2093->client = client;
	ret = of_property_read_u32(node, CANAANMODULE_CAMERA_MODULE_INDEX,
				   &gc2093->module_index);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_FACING,
				       &gc2093->module_facing);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_NAME,
				       &gc2093->module_name);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_LENS_NAME,
				       &gc2093->len_name);

	if (ret) {
		dev_err(dev, "Failed to get module information\n");
		return -EINVAL;
	}

	gc2093->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc2093->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc2093->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(gc2093->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");


	if (gpiod_export(gc2093->reset_gpio, 0)) {
		dev_err(dev, "gpiod export gc2093 reset failed.");
	}

	if (gpiod_export(gc2093->pwdn_gpio, 0)) {
		dev_err(dev, "gpiod export gc2093 powerdown failed.");
	}

	mutex_init(&gc2093->lock);

	/* set default mode */
	gc2093->cur_mode = &supported_modes[0];
	gc2093->cfg_num = ARRAY_SIZE(supported_modes);

	sd = &gc2093->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc2093_subdev_ops);
	ret = gc2093_initialize_controls(gc2093);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc2093_power_on(gc2093);
	if (ret)
		goto err_free_handler;

	ret = gc2093_check_sensor_id(gc2093);
	if (ret)
		goto err_power_off;

	sd->internal_ops = &gc2093_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	gc2093->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc2093->pad);
	if (ret < 0)
		goto err_power_off;

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc2093->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc2093->module_index, facing,
		 GC2093_NAME, dev_name(sd->dev));

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
	__gc2093_power_off(gc2093);
err_free_handler:
	v4l2_ctrl_handler_free(&gc2093->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc2093->lock);

	return ret;
}

static int gc2093_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2093 *gc2093 = to_gc2093(sd);

	v4l2_async_unregister_subdev(sd);
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc2093->ctrl_handler);
	mutex_destroy(&gc2093->lock);

	__gc2093_power_off(gc2093);
	return 0;
}

static const struct i2c_device_id gc2093_match_id[] = {
	{ "gc2093", 0 },
	{ },
};

static const struct of_device_id gc2093_of_match[] = {
	{ .compatible = "galaxycore,gc2093" },
	{},
};
MODULE_DEVICE_TABLE(of, gc2093_of_match);

static struct i2c_driver gc2093_i2c_driver = {
	.driver = {
		.name = GC2093_NAME,
		.of_match_table = of_match_ptr(gc2093_of_match),
	},
	.probe      = &gc2093_probe,
	.remove     = &gc2093_remove,
	.id_table   = gc2093_match_id,
};

module_i2c_driver(gc2093_i2c_driver);
MODULE_DESCRIPTION("Galaxycore GC2093 Image Sensor driver");
MODULE_LICENSE("GPL v2");
