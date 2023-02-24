/*
 * Driver for IMX385 CMOS Image Sensor from Sony
 *
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 * Copyright (C) 2014, Andrew Chew <achew@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * V0.0X01.0X01 add enum_frame_interval function.
 * V0.0X01.0X02 add function g_mbus_config.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/canaan-camera-module.h>
#include <linux/k510isp.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
// gpio
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/bug.h>
#include <media/dvb_math.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)

/* IMX385 supported geometry */
#define IMX385_TABLE_END				0xffff
#define IMX385_ANALOGUE_GAIN_MIN		0
#define IMX385_ANALOGUE_GAIN_MAX		4095
#define IMX385_ANALOGUE_GAIN_DEFAULT	0

/* In dB*256 */
#define IMX385_DIGITAL_GAIN_MIN			0
#define IMX385_DIGITAL_GAIN_MAX			4095
#define IMX385_DIGITAL_GAIN_DEFAULT		0

#define IMX385_DIGITAL_EXPOSURE_MIN		0
#define IMX385_DIGITAL_EXPOSURE_MAX		1125
#define IMX385_DIGITAL_EXPOSURE_DEFAULT	560

#define IMX385_VTS_MAX			0x7fff

/*
 * Constants for sensor reset delay
 */
#define IMX385_RESET_DELAY1			(2000)
#define IMX385_RESET_DELAY2			(2200)

#define IMX385_EXP_LINES_MARGIN			4

#define IMX385_NAME						"IMX385"

#define IMX385_LANES					2

static const s64 link_freq_menu_items[] = {
	456000000,
};

struct imx385_reg {
	u16 addr;
	u8 val;
};

struct imx385_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 bpp;
	u32 hdr_mode;
	const struct imx385_reg *reg_list;
};

/* MCLK:24MHz  1920x1080  30fps   MIPI LANE2 */
static const struct imx385_reg imx385_init_tab_1920_1080_30fps[] = {
	{0x3000,0x01},
	{0x3001,0x00},
	{0x3002,0x01},
	{0x3004,0x10},
	{0x3005,0x01},
	{0x3007,0x03},
	{0x3009,0x02},
	{0x300a,0x00},
	{0x300b,0x00},
	{0x3012,0x2c},
	{0x3013,0x01},
	{0x3014,0x80},
	{0x3015,0x00},
	{0x3016,0x08},
	{0x3018,0x65},
	{0x3019,0x04},
	{0x301a,0x00},
	{0x301b,0x30},
	{0x301c,0x11},
	{0x3020,0x02},
	{0x3021,0x00},
	{0x3022,0x00},
	{0x303a,0x0c},
	{0x303b,0x00},
	{0x303c,0x00},
	{0x303d,0x00},
	{0x303e,0x49},
	{0x303f,0x04},
	{0x3044,0x01},
	{0x3046,0x30},
	{0x3047,0x38},
	{0x3049,0x0a},
	{0x3054,0x66},
	{0x305c,0x18},
	{0x305d,0x10},
	{0x305e,0x20},
	{0x305f,0x10},
	{0x310b,0x07},
	{0x3110,0x12},
	{0x31ed,0x38},
	{0x3338,0xd4},
	{0x3339,0x40},
	{0x333a,0x10},
	{0x333b,0x00},
	{0x333c,0xd4},
	{0x333d,0x40},
	{0x333e,0x10},
	{0x333f,0x00},
	{0x3344,0x00},
	{0x3346,0x01},
	{0x3357,0x49},
	{0x3358,0x04},
	{0x336b,0x3f},
	{0x336c,0x1f},
	{0x337d,0x0c},
	{0x337e,0x0c},
	{0x337f,0x01},
	{0x3380,0x40},
	{0x3381,0x4a},
	{0x3382,0x67},
	{0x3383,0x1F},
	{0x3384,0x3f},
	{0x3385,0x27},
	{0x3386,0x1F},
	{0x3387,0x17},
	{0x3388,0x77},
	{0x3389,0x27},
	{0x338d,0x67},
	{0x338e,0x03},
	{0x3000,0x00},//
	{0x3002,0x00},
	{IMX385_TABLE_END, 0x00}
};

static const struct imx385_reg imx385_wdr2f_12p5fps_12bit_2lane[] = {
	{0x3000,0x01},
	{0x3001,0x00},
	{0x3002,0x01},
	{0x3005,0x01},
	{0x3007,0x13},
	{0x3009,0x02},
	{0x300a,0x00},
	{0x300b,0x00},
	{0x300c,0x11},
	{0x3012,0x2c},
	{0x3013,0x01},
	{0x3014,0x80},
	{0x3015,0x00},
	{0x3016,0x08},
	{0x3018,0x65},
	{0x3019,0x04},
	{0x301a,0x00},
	{0x301b,0xa0},
	{0x301c,0x14},
	{0x3020,0x03},  //SHS1 = 0x04 = 4, SEF1max =  RHS1 - 3 -1  = 37 - 4 = 33, SEF1 = 37 - 3 -1 = 33
	{0x3021,0x00},  //0x00
	{0x3022,0x00},  //0x00
	{0x3023,0x89},  //SHS2 = 0x89 = 137, LEFmax = 2250 - 37 - 4 = 2209, LEF = 2250 - 137 -1 = 2122
	{0x3024,0x00},  //0x00
	{0x3025,0x00},
	{0x302c,0x25}, //RHS1 = 0x25 = 37, VBP1 = (RHS1 - 1)/2 = 18
	{0x302d,0x00},
	{0x302e,0x00},
	{0x303a,0x0c},
	{0x303b,0x00},
	{0x303c,0x00},
	{0x303d,0x00},
	{0x303e,0x49},
	{0x303f,0x04},
	{0x3043,0x05},
	{0x3044,0x01},
	{0x3046,0x30},
	{0x3047,0x38},
	{0x3049,0x0a},
	{0x3054,0x66},
	{0x305c,0x18},
	{0x305d,0x10},
	{0x305e,0x20},
	{0x305f,0x10},
	{0x3108,0x11},
	{0x3109,0x01},
	{0x310A,0x00},
	{0x310b,0x07},
	{0x3110,0x12},
	{0x31ed,0x38},
	{0x3338,0xd4},
	{0x3339,0x40},
	{0x333a,0x10},
	{0x333b,0x00},
	{0x333c,0xd4},
	{0x333d,0x40},
	{0x333e,0x10},
	{0x333f,0x00},
	{0x3344,0x00},
	{0x3346,0x01},
	{0x3354,0x00},
	{0x3357,0xb2},	//PIC_SIZE_V = 0x8b8 = 2232 , = (1097 +VBP1)*2 = (1097 +19) * 2 = 2232 (0x8b8)
	{0x3358,0x08},
	{0x336b,0x3f},
	{0x336c,0x1f},
	{0x337d,0x0c},
	{0x337e,0x0c},
	{0x337f,0x01},
	{0x3380,0x40},
	{0x3381,0x4a},
	{0x3382,0x67},
	{0x3383,0x1f},
	{0x3384,0x3f},
	{0x3385,0x27},
	{0x3386,0x1F},
	{0x3387,0x17},
	{0x3388,0x77},
	{0x3389,0x27},
	{0x338d,0x67},
	{0x338e,0x03},
	{0x3000,0x00},
	{0x3002,0x00},
	{0x308c,0x00},
	{IMX385_TABLE_END, 0x00}
};

static const struct imx385_reg imx385_wdr3f_12p5fps_12bit_2lane[] = {
	{0x3000,0x01},
	{0x3001,0x00},
	{0x3002,0x01},
	{0x3005,0x01},
	{0x3007,0x03},
	{0x3009,0x02},	//FRSEL: CSI-2 2lane, 2 lane
	{0x300a,0x00},
	{0x300b,0x00},
	{0x300c,0x21},	// WDMODE: 1, DOL mode; WDSEL: DOL 3frame
	{0x3012,0x2c},
	{0x3013,0x01},
	{0x3014,0x80},
	{0x3015,0x00},
	{0x3016,0x08},
	{0x3018,0x65},
	{0x3019,0x04},
	{0x301a,0x00},
	{0x301b,0xa0},
	{0x301c,0x14},
	{0x3020,0x0d},	// SHS1 = 10, SEF1max = 46 - 5 = 41; SEF1 = 46 - 13 - 1 = 32
	{0x3021,0x00},
	{0x3022,0x00},
	{0x3023,0x36},   //SHS2 = 54, SEF2max = 59 - 46 - 5 = 8; SEF2 = 59 - 54 - 1 = 4
	{0x3024,0x00},
	{0x3025,0x00},
	{0x3026,0x93},  //SHS3 = 2451, LEFmax = 4500 - 59 - 4 -1 = 4436, LEF = 4500 - 2451 -1 = 2048
	{0x3027,0x09},
	{0x3028,0x00},
	{0x302c,0x2e},	//RHS1 = 0x2e = 46, VBP1 = (RHS1 - 1)/3 = 15
	{0x302d,0x00},
	{0x302e,0x00},
	{0x302f,0x3b},	 //RHS2 = 0x3b = 59, VBP2 = (RHS2 - 2)/3 = 19 <20,  VBP2 - VBP1 = 19 - 15 = 4 <6
	{0x3030,0x00},
	{0x3031,0x00},
	{0x303a,0x0c},
	{0x303b,0x00},
	{0x303c,0x00},
	{0x303d,0x00},
	{0x303e,0x49},
	{0x303f,0x04},
	{0x3043,0x05},
	{0x3044,0x01},	//OPORTSEL, ODBIT, 12bit, CSI-2
	{0x3046,0x30},
	{0x3047,0x38},
	{0x3049,0x0a},
	{0x3054,0x66},
	{0x305c,0x18},
	{0x305d,0x10},
	{0x305e,0x20},
	{0x305f,0x10},
	{0x3108,0x23},
	{0x3109,0x03},
	{0x310A,0x00},
	{0x310b,0x07},
	{0x3110,0x12},
	{0x31ed,0x38},
	{0x3338,0xd4},
	{0x3339,0x40},
	{0x333a,0x10},
	{0x333b,0x00},
	{0x333c,0xd4},
	{0x333d,0x40},
	{0x333e,0x10},
	{0x333f,0x00},
	{0x3344,0x00},
	{0x3346,0x01},
	{0x3354,0x00},
	{0x3357,0x44},	//PIC_SIZE_V = 3396
	{0x3358,0x0d},
	{0x336b,0x3f},
	{0x336c,0x1f},
	{0x337d,0x0c},
	{0x337e,0x0c},
	{0x337f,0x01},
	{0x3380,0x40},
	{0x3381,0x4a},
	{0x3382,0x67},
	{0x3383,0x1f},
	{0x3384,0x3f},
	{0x3385,0x27},
	{0x3386,0x1F},
	{0x3387,0x17},
	{0x3388,0x77},
	{0x3389,0x27},
	{0x338d,0x67},
	{0x338e,0x03},
	{0x3000,0x00},
	{0x3002,0x00},
	{0x308c,0x00},
	{IMX385_TABLE_END, 0x00}
};

static const struct imx385_reg start[] = {
	{0x3000, 0x00},		/* mode select streaming on */
	{IMX385_TABLE_END, 0x00}
};

static const struct imx385_reg stop[] = {
	{0x3000, 0x01},		/* mode select streaming off */
	{IMX385_TABLE_END, 0x00}
};

enum {
	TEST_PATTERN_DISABLED,
	TEST_PATTERN_MULTI_PIXELS,
	TEST_PATTERN_SEQUENCE1,
	TEST_PATTERN_HORIZONTAL_COLOR_BAR,
	TEST_PATTERN_VERTICAL_COLOR_BAR,
	TEST_PATTERN_SEQUENCE2,
	TEST_PATTERN_GRADATION_PATTERN1,
	TEST_PATTERN_GRADATION_PATTERN2,
	TEST_PATTERN_000_555_TOGGLE_PATTERN,
	TEST_PATTERN_4PIXEL_PATTERN,
	TEST_PATTERN_HORIZONTAL1_ROW,
	TEST_PATTERN_VERTICAL1_COLUMN,
	TEST_PATTERN_1ROW_AND_1_COLUMN,
	TEST_PATTERN_STRIPE_PATTERN,
	TEST_PATTERN_CHECKER_PATTERN,
	TEST_PATTERN_1_PIXEL_PATTERN,
	TEST_PATTERN_HORIZONTAL_2PIXEL_PATTERN,
	TEST_PATTERN_MAX,
};

static const char *const tp_qmenu[] = {
	"Disabled",
	"Multiple Pixels Pattern",
	"Sequence Pattern 1",
	"Horizontal Color-bar Chart",
	"Vertical Color-bar Chart",
	"Sequence Pattern 2",
	"Gradation Pattern 1",
	"Gradation Pattern 2",
	"000h/555h Toggle Pattern",
	"4-pixel Pattern",
	"Horizontal 1 Row Pattern",
	"Vertical 1 Column Pattern",
	"1 Row and 1 Column Pattern",
	"Stripe Pattern of the Arbitrary Value",
	"Checker pattern of the Arbitrary Value",
	"1-pixel Pattern",
	"Horizontal 2-pixel Pattern",
};

#define SIZEOF_I2C_TRANSBUF 32

struct imx385 {
	struct i2c_client	*client;

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct clk *clk;
	struct v4l2_rect crop_rect;
	int hflip;
	int vflip;
	u8 analogue_gain;
	u16 digital_gain;	/* bits 11:0 */
	u16 exposure_time;

	u16 test_pattern;
	u16 test_pattern_pgthru;
	u16 test_pattern_colorwidth;
	u16 test_pattern_pg_mode;
	u16 test_pattern_pghpos;
	u16 test_pattern_pgvpos;
	u16 test_pattern_pghpstep;
	u16 test_pattern_pgvpstep;
	u16 test_pattern_pghpnum;
	u16 test_pattern_pgvpnum;
	u16 test_pattern_pghprm;
	u16 test_pattern_pgdata1;
	u16 test_pattern_pgdata2;

	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	const struct imx385_mode *cur_mode;
	u32 cfg_num;
	u16 cur_vts;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	struct gpio_desc *imx385_powerdown;
	struct gpio_desc *imx385_reset;
	struct mutex lock; /* mutex lock for operations */
	bool			power_on;
	bool		    streaming;
	bool			has_init_exp;
	struct cnmodule_hdr_ae_cfg init_hdrae_exp;
};

/**
 * @brief
 *
 */
static const struct imx385_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB12_1X12,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 0x0898 - IMX385_EXP_LINES_MARGIN,
		.vts_def = 0x0465,
		.hdr_mode = NO_HDR,
		.reg_list = imx385_init_tab_1920_1080_30fps,
		.bpp = 12,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB12_1X12,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 0x0898 - IMX385_EXP_LINES_MARGIN,
		.vts_def = 0x0465,
		.hdr_mode = HDR_X2,
		.reg_list = imx385_wdr2f_12p5fps_12bit_2lane,
		.bpp = 12,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB12_1X12,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 0x0898 - IMX385_EXP_LINES_MARGIN,
		.vts_def = 0x0465,
		.hdr_mode = HDR_X3,
		.reg_list = imx385_wdr3f_12p5fps_12bit_2lane,
		.bpp = 12,
	},
};

/**
 * @brief
 *
 * @param client
 * @return struct imx385*
 */
static struct imx385 *to_imx385(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx385, subdev);
}

/**
 * @brief
 *
 * @param client
 * @param addr
 * @param data
 * @return int
 */
static int reg_write(struct i2c_client *client, const u16 addr, const u8 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[3];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;
	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data;
	ret = i2c_transfer(adap, &msg, 1);
	mdelay(2);

	return ret == 1 ? 0 : -EIO;
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
	mdelay(2);

	return ret == 1 ? 0 : -EIO;
}

static int imx385_set_2frame_dol_hdr_et(struct i2c_client *client, const u16 l_exp_time)
{
	int ret;
	u16 efl, shs2;
	u16 sef1, shs1;
	u16 rhs1 = 37;
	u16 fsc = 2250;	//1125 x 2
	u16 efl_max = 2209;	// FSC - ((RHS1+3)+1) = 2250 - 37 - 4 = 2209
	u16 sef1_max = 33;		// RHS1 - £¨SHS1_min +1£©= 37 - 3 -1 = 33;
	u16 exp_time;

	efl = l_exp_time;
	if (efl > efl_max)
		efl = efl_max;

	sef1 = efl >> 5;	//1/32,  SEF1 = EFL>>6;	//1/64
	if (sef1 > sef1_max)
		sef1 = sef1_max;
	else if (sef1 < 1)
		sef1 = 1;

	shs2 = fsc - efl -1;
	shs1 = rhs1 - sef1 -1;
	//long ET
	exp_time = ((shs2 & 0xff) <<8 ) + ((shs2 & 0xff00) >> 8);
	ret = reg16_write(client, 0x3023, exp_time);
	//short ET
	exp_time = ((shs1 & 0xff) << 8) + ((shs1 & 0xff00) >> 8);
	ret |= reg16_write(client, 0x3020, exp_time);

	return ret;
}

static int imx385_set_3frame_dol_hdr_et(struct i2c_client *client, const u16 l_exp_time)
{
	int ret;
	u16 efl, shs3;
	u16 sef1, shs1;
	u16 sef2, shs2;
	u16 rhs1 = 46;
	u16 rhs2 = 59;
	u16 fsc = 4500;	//1125 x 4
	u16 efl_max = 4436;	// FSC - ((RHS2+4)+1) = 4500 - 59 - 5 = 4436
	u16 sef1_max = 38;		// RHS1 - £¨SHS1_min +1£©= 46 - 4 -1 = 41;
	u16 sef2_max = 8;		// RHS2 - (SHS2_min +1) = 59 - (RHS1 + 4 +1£©= 59 - 46 - 5 = 8;
	u16 exp_time;

	efl = l_exp_time;
	if (efl > efl_max)
		efl = efl_max;

	sef1 = efl >> 5;	//1/32,  SEF1 = EFL>>6;	//1/64
	if (sef1 > sef1_max)
		sef1 = sef1_max;
	else if (sef1 < 1)
		sef1 = 1;

	sef2 = efl >> 9;	//1/512
	if (sef2 > sef2_max)
		sef2 = sef2_max;
	else if (sef2 < 1)
		sef2 = 1;

	shs3 = fsc - efl -1;
	shs1 = rhs1 - sef1 -1;
	shs2 = rhs2 - sef2 -1;
	//long ET
	exp_time = ((shs3 & 0xff) << 8) + ((shs3 &0xff00) >> 8);
	ret = reg16_write(client, 0x3026, exp_time);
	//short ET
	exp_time = ((shs1 & 0xff) << 8) + ((shs1 & 0xff00) >> 8);
	ret |= reg16_write(client, 0x3020, exp_time);
	//the shortest ET
	exp_time = ((shs2 & 0xff) << 8) + ((shs2 & 0xff00) >> 8);
	ret |= reg16_write(client, 0x3023, exp_time);

	return ret;
}

static int reg_read(struct i2c_client *client, const u16 addr)
{
	u8 buf[2] = {addr >> 8, addr & 0xff};
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
	if (ret < 0) {
		dev_warn(&client->dev, "Reading register %x from %x failed\n",
				addr, client->addr);
		return ret;
	}

	return buf[0];
}
/**
 * @brief
 *
 * @param client
 * @param table
 * @return int
 */
static int reg_write_table(struct i2c_client *client,
		const struct imx385_reg table[])
{
	const struct imx385_reg *reg;
	int ret;

	for (reg = table; reg->addr != IMX385_TABLE_END; reg++) {
		ret = reg_write(client, reg->addr, reg->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}
/**
 * @brief
 *
 * @param client
 * @param table
 * @return int
 */
static int reg_read_table(struct i2c_client *client,
		const struct imx385_reg table[])
{
	const struct imx385_reg *reg;

	for (reg = table; reg->addr != IMX385_TABLE_END; reg++) {
		dev_info(&client->dev,"%s:addr(0x%x),val(0x%x)\n",__func__,reg->addr,reg_read(client,reg->addr));
	}

	return 0;
}

static int __imx385_power_on(struct imx385 *imx385)
{
	dev_dbg(&imx385->client->dev, "__imx385_power_on\n");

	if (!IS_ERR(imx385->imx385_reset)) {
		gpiod_set_value_cansleep(imx385->imx385_reset, 1);
		usleep_range(IMX385_RESET_DELAY1, IMX385_RESET_DELAY2);
	}

	if (!IS_ERR(imx385->imx385_reset))
		gpiod_set_value_cansleep(imx385->imx385_reset, 0);

	usleep_range(IMX385_RESET_DELAY1, IMX385_RESET_DELAY2);
	return 0;
}

static void __imx385_power_off(struct imx385 *imx385)
{
	dev_dbg(&imx385->client->dev, "__imx385_power_off\n");

	//if (!IS_ERR(imx385->pwdn_gpio))
	//	gpiod_set_value_cansleep(imx385->pwdn_gpio, 1);

	if (!IS_ERR(imx385->imx385_reset))
		gpiod_set_value_cansleep(imx385->imx385_reset, 1);
}

/**
 * @brief
 *
 */
/* V4L2 subdev core operations */
static int imx385_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *imx385 = to_imx385(client);
	int ret = 0;

	mutex_lock(&imx385->lock);

	/* If the power state is not modified - no work to do. */
	if (imx385->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		__imx385_power_on(imx385);
		imx385->power_on = true;
	} else {
		__imx385_power_off(imx385);
		imx385->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx385->lock);

	return ret;
}

/* V4L2 ctrl operations */
static int imx385_s_ctrl_test_pattern(struct v4l2_ctrl *ctrl)
{
	struct imx385 *priv =
		container_of(ctrl->handler, struct imx385, ctrl_handler);

	switch (ctrl->val) {
		case TEST_PATTERN_DISABLED:
			priv->test_pattern = 0x0000;
			break;
		case TEST_PATTERN_MULTI_PIXELS:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pghpos = 0;
			priv->test_pattern_pgvpos = 0;
			priv->test_pattern_pghpstep = 1;
			priv->test_pattern_pgvpstep = 1;
			priv->test_pattern_pghpnum = 1;
			priv->test_pattern_pgvpnum = 1;
			break;
		case TEST_PATTERN_SEQUENCE1:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 1;
			break;
		case TEST_PATTERN_HORIZONTAL_COLOR_BAR:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 2;
			priv->test_pattern_colorwidth = 0;
			break;
		case TEST_PATTERN_VERTICAL_COLOR_BAR:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 3;
			priv->test_pattern_colorwidth = 0;
			break;
		case TEST_PATTERN_SEQUENCE2:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 4;
			break;
		case TEST_PATTERN_GRADATION_PATTERN1:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 5;
			break;
		case TEST_PATTERN_GRADATION_PATTERN2:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 6;
			break;
		case TEST_PATTERN_000_555_TOGGLE_PATTERN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 7;
			break;
		case TEST_PATTERN_4PIXEL_PATTERN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 8;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pghpos = 0;
			priv->test_pattern_pgvpos = 0;
			priv->test_pattern_pgthru = 1;
			break;
		case TEST_PATTERN_HORIZONTAL1_ROW:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 9;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pgvpos = 0;
			priv->test_pattern_pgthru = 1;
			break;
		case TEST_PATTERN_VERTICAL1_COLUMN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0xa;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pghpos = 0;
			priv->test_pattern_pgthru = 1;
			break;
		case TEST_PATTERN_1ROW_AND_1_COLUMN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0xb;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pghpos = 0;
			priv->test_pattern_pgthru = 1;
			break;
		case TEST_PATTERN_STRIPE_PATTERN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0xc;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			break;
		case TEST_PATTERN_CHECKER_PATTERN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0xd;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			break;
		case TEST_PATTERN_1_PIXEL_PATTERN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0xe;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pghpos = 0;
			priv->test_pattern_pgvpos = 0;
			priv->test_pattern_pgthru = 1;
			break;
		case TEST_PATTERN_HORIZONTAL_2PIXEL_PATTERN:
			priv->test_pattern = 0x0001;
			priv->test_pattern_pg_mode = 0xf;
			priv->test_pattern_pgdata1 = 0xf;
			priv->test_pattern_pgdata2 = 0xf;
			priv->test_pattern_pghpos = 0;
			priv->test_pattern_pgvpos = 0;
			priv->test_pattern_pghprm = 1;
			priv->test_pattern_pgthru = 1;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}
/**
 * @brief
 *
 * @param sd
 * @param fi
 * @return int
 */
static int imx385_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *priv = to_imx385(client);
	const struct imx385_mode *mode = priv->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

/**
 * @brief
 *
 * @param ctrl
 * @return int
 */


static int IMX385_GaindB[481] =
{
	0,   3,   5,   8,   10,  13,  15,  17,  19,  22,
	24,  26,  28,  30,  32,  33,  35,  37,  39,  40,
	42,  44,  45,  47,  49,  50,  52,  53,  55,  56,
	57,  59,  60,  62,  63,  64,  65,  67,  68,  69,
	70,  72,  73,  74,  75,  76,  77,  78,  80,  81,
	82,  83,  84,  85,  86,  87,  88,  89,  90,  91,
	92,  93,  94,  95,  95,  96,  97,  98,  99,  100,
	101, 102, 102, 103, 104, 105, 106, 106, 107, 108,
	109, 110, 110, 111, 112, 113, 113, 114, 115, 116,
	116, 117, 118, 118, 119, 120, 120, 121, 122, 122,
	123, 124, 124, 125, 126, 126, 127, 128, 128, 129,
	129, 130, 131, 131, 132, 132, 133, 134, 134, 135,
	135, 136, 136, 137, 138, 138, 139, 139, 140, 140,
	141, 141, 142, 142, 143, 144, 144, 145, 145, 146,
	146, 147, 147, 148, 148, 149, 149, 150, 150, 151,
	151, 151, 152, 152, 153, 153, 154, 154, 155, 155,
	156, 156, 157, 157, 157, 158, 158, 159, 159, 160,
	160, 160, 161, 161, 162, 162, 163, 163, 163, 164,
	164, 165, 165, 165, 166, 166, 167, 167, 167, 168,
	168, 169, 169, 169, 170, 170, 171, 171, 171, 172,
	172, 172, 173, 173, 174, 174, 174, 175, 175, 175,
	176, 176, 176, 177, 177, 178, 178, 178, 179, 179,
	179, 180, 180, 180, 181, 181, 181, 182, 182, 182,
	183, 183, 183, 184, 184, 184, 185, 185, 185, 186,
	186, 186, 187, 187, 187, 187, 188, 188, 188, 189,
	189, 189, 190, 190, 190, 191, 191, 191, 191, 192,
	192, 192, 193, 193, 193, 194, 194, 194, 194, 195,
	195, 195, 196, 196, 196, 196, 197, 197, 197, 198,
	198, 198, 198, 199, 199, 199, 199, 200, 200, 200,
	201, 201, 201, 201, 202, 202, 202, 202, 203, 203,
	203, 203, 204, 204, 204, 204, 205, 205, 205, 206,
	206, 206, 206, 207, 207, 207, 207, 208, 208, 208,
	208, 209, 209, 209, 209, 210, 210, 210, 210, 210,
	211, 211, 211, 211, 212, 212, 212, 212, 213, 213,
	213, 213, 214, 214, 214, 214, 214, 215, 215, 215,
	215, 216, 216, 216, 216, 217, 217, 217, 217, 217,
	218, 218, 218, 218, 219, 219, 219, 219, 219, 220,
	220, 220, 220, 220, 221, 221, 221, 221, 222, 222,
	222, 222, 222, 223, 223, 223, 223, 223, 224, 224,
	224, 224, 224, 225, 225, 225, 225, 225, 226, 226,
	226, 226, 226, 227, 227, 227, 227, 227, 228, 228,
	228, 228, 228, 229, 229, 229, 229, 229, 230, 230,
	230, 230, 230, 231, 231, 231, 231, 231, 232, 232,
	232, 232, 232, 232, 233, 233, 233, 233, 233, 234,
	234, 234, 234, 234, 234, 235, 235, 235, 235, 235,
	236, 236, 236, 236, 236, 236, 237, 237, 237, 237,
	237, 238, 238, 238, 238, 238, 238, 239, 239, 239,
	239, 239, 239, 240, 240, 240, 240, 240, 240, 241,
	241,
};

static int imx385_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx385 *priv =
		container_of(ctrl->handler, struct imx385, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	u16 d_gain;
	u16 exp_time;
	int ret;

	dev_dbg(&client->dev,"%s:ctrl->id(0x%x),ctrl->val(%d)\n",__func__,ctrl->id,ctrl->val);

	switch (ctrl->id) {
		case V4L2_CID_HFLIP:
			priv->hflip = ctrl->val;
			break;

		case V4L2_CID_VFLIP:
			priv->vflip = ctrl->val;
			break;

		case V4L2_CID_ANALOGUE_GAIN:
		case V4L2_CID_GAIN:
			priv->digital_gain = (ctrl->val + 7) >> 3;
			if (priv->digital_gain <= 32) {
				priv->digital_gain = 0;
			} else {
				priv->digital_gain -= 32;
				if (priv->digital_gain > 480) {
					priv->digital_gain = 480;
				}
			}

			priv->digital_gain = IMX385_GaindB[priv->digital_gain];
			d_gain = ((priv->digital_gain & 0xff) <<8) + ((priv->digital_gain & 0x300) >> 8);
			ret = reg16_write(client, 0x3014, d_gain);
			return ret;

		case V4L2_CID_EXPOSURE:
			if(priv->cur_mode->hdr_mode == NO_HDR) {
				priv->exposure_time = 1124 - ctrl->val;
				exp_time = ((priv->exposure_time & 0xff) << 8) + ((priv->exposure_time & 0xff00) >> 8);
				ret = reg16_write(client, 0x3020, exp_time);
				return ret;
			} else if(priv->cur_mode->hdr_mode == HDR_X2) {
				ret = imx385_set_2frame_dol_hdr_et(client,ctrl->val);
				return ret;
			} else {
				ret = imx385_set_3frame_dol_hdr_et(client,ctrl->val);
				return ret;
			}

		case V4L2_CID_TEST_PATTERN:
			return imx385_s_ctrl_test_pattern(ctrl);

		case V4L2_CID_VBLANK:
			if (ctrl->val < priv->cur_mode->vts_def)
				ctrl->val = priv->cur_mode->vts_def;
			if ((ctrl->val - IMX385_EXP_LINES_MARGIN) != priv->cur_vts)
				priv->cur_vts = ctrl->val - IMX385_EXP_LINES_MARGIN;
			ret = reg_write(client, 0x3019, ((priv->cur_vts >> 8) & 0xff));
			ret |= reg_write(client, 0x3018, (priv->cur_vts & 0xff));
			return ret;

		default:
			return -EINVAL;
	}
	/* If enabled, apply settings immediately */
	//reg = reg_read(client, 0x3000);
	//if ((reg & 0x01) != 0x01)
	//	imx385_s_stream(&priv->subdev, 1);

	return 0;
}
/**
 * @brief
 *
 * @param sd
 * @param cfg
 * @param code
 * @return int
 */
static int imx385_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *imx385 = to_imx385(client);

	if (code->index != 0)
		return -EINVAL;
	code->code = imx385->cur_mode->bus_fmt;

	return 0;
}
/**
 * @brief
 *
 * @param mode
 * @param framefmt
 * @return int
 */
static int imx385_get_reso_dist(const struct imx385_mode *mode,
		struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		abs(mode->height - framefmt->height);
}
/**
 * @brief
 *
 * @param fmt
 * @return const struct imx385_mode*
 */
static const struct imx385_mode *imx385_find_best_fit(
		struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = imx385_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static void imx385_change_mode(struct imx385 *imx385, const struct imx385_mode *mode)
{
	imx385->cur_mode = mode;
	imx385->cur_vts = imx385->cur_mode->vts_def;
	dev_dbg(&imx385->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d\n",
			mode->width, mode->height, mode->hdr_mode);
}

/**
 * @brief
 *
 * @param sd
 * @param cfg
 * @param fmt
 * @return int
 */
static int imx385_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *imx385 = to_imx385(client);
	const struct imx385_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&imx385->lock);

	mode = imx385_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	dev_info(&imx385->client->dev, "%s, width:%u, height:%u, hdr_mode:%u.\n", \
		__func__, mode->width, mode->height, mode->hdr_mode);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
	} else {
		imx385_change_mode(imx385, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx385->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx385->vblank, vblank_def,
					 IMX385_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&imx385->lock);

	return 0;
}

/**
 * @brief
 *
 * @param sd
 * @param cfg
 * @param fmt
 * @return int
 */
static int imx385_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *imx385 = to_imx385(client);
	const struct imx385_mode *mode = imx385->cur_mode;

	mutex_lock(&imx385->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx385->lock);

	dev_info(&imx385->client->dev, "%s, width:%u, height:%u, hdr_mode:%u.\n", \
		__func__, fmt->format.width, fmt->format.height, mode->hdr_mode);

	return 0;
}

static int imx385_set_hdrae_2frame(struct imx385 *imx385,
		struct cnmodule_hdr_ae_cfg *hdr_ae_cfg)
{
	struct i2c_client *client = imx385->client;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;
	int ret = 0;

	if (!imx385->has_init_exp && !imx385->streaming) {
		imx385->init_hdrae_exp = *hdr_ae_cfg;
		imx385->has_init_exp = true;
		dev_dbg(&client->dev, "imx335 is not streaming, save hdr ae!\n");
		return ret;
	}

	l_exp_time = hdr_ae_cfg->long_exp_val;
	m_exp_time = hdr_ae_cfg->middle_exp_val;
	s_exp_time = hdr_ae_cfg->short_exp_val;
	l_a_gain = hdr_ae_cfg->long_gain_val;
	m_a_gain = hdr_ae_cfg->middle_gain_val;
	s_a_gain = hdr_ae_cfg->short_gain_val;

	dev_info(&client->dev,
			"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
			l_exp_time, l_a_gain, m_exp_time, m_a_gain, s_exp_time, s_a_gain);

	if (imx385->cur_mode->hdr_mode == HDR_X2) {
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
	}

	return ret;
}

static int imx385_set_hdrae_3frame(struct imx385 *imx385,
		struct cnmodule_hdr_ae_cfg *hdr_ae_cfg)
{
	struct i2c_client *client = imx385->client;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;
	int ret = 0;

	if (!imx385->has_init_exp && !imx385->streaming) {
		imx385->init_hdrae_exp = *hdr_ae_cfg;
		imx385->has_init_exp = true;
		dev_dbg(&client->dev, "imx335 is not streaming, save hdr ae!\n");
		return ret;
	}

	l_exp_time = hdr_ae_cfg->long_exp_val;
	m_exp_time = hdr_ae_cfg->middle_exp_val;
	s_exp_time = hdr_ae_cfg->short_exp_val;
	l_a_gain = hdr_ae_cfg->long_gain_val;
	m_a_gain = hdr_ae_cfg->middle_gain_val;
	s_a_gain = hdr_ae_cfg->short_gain_val;

	dev_info(&client->dev,
			"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
			l_exp_time, l_a_gain, m_exp_time, m_a_gain, s_exp_time, s_a_gain);

	return ret;
}

/**
 * @brief
 *
 * @param sd
 * @param cmd
 * @param arg
 * @return long
 */
static long imx385_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *imx385 = to_imx385(client);

	struct cnmodule_hdr_cfg *hdr_cfg;
	u32 i, h, w;
	long ret = 0;

	switch (cmd) {
		case CNMODULE_HDR_AE_EXP_CFG:
			dev_info(&imx385->client->dev, "set hdr ae exp.\n");
			if (imx385->cur_mode->hdr_mode == HDR_X2)
				ret = imx385_set_hdrae_2frame(imx385, arg);
			else if (imx385->cur_mode->hdr_mode == HDR_X3)
				ret = imx385_set_hdrae_3frame(imx385, arg);
			break;

		case CNMODULE_GET_MODULE_INFO:
			break;
		case CNMODULE_GET_HDR_CFG:
			dev_info(&imx385->client->dev, "get hdr cfg.\n");
			hdr_cfg = (struct cnmodule_hdr_cfg *)arg;
			hdr_cfg->hdr_mode = imx385->cur_mode->hdr_mode;
			break;
		case CNMODULE_SET_HDR_CFG:
			hdr_cfg = (struct cnmodule_hdr_cfg *)arg;
			w = imx385->cur_mode->width;
			h = imx385->cur_mode->height;
			for (i = 0; i < imx385->cfg_num; i++) {
				if (w == supported_modes[i].width &&
						h == supported_modes[i].height &&
						supported_modes[i].hdr_mode == hdr_cfg->hdr_mode) {
					imx385_change_mode(imx385, &supported_modes[i]);
					break;
				}
			}
			dev_info(&imx385->client->dev, "set hdr mode:%d %ux%u\n",
						hdr_cfg->hdr_mode, w, h);

			if (i == imx385->cfg_num) {
				dev_err(&imx385->client->dev,
						"not find hdr mode:%d %dx%d config\n",
						hdr_cfg->hdr_mode, w, h);
				ret = -EINVAL;
			} else {
				w = imx385->cur_mode->hts_def - imx385->cur_mode->width;
				h = imx385->cur_mode->vts_def - imx385->cur_mode->height;
				__v4l2_ctrl_modify_range(imx385->hblank, w, w, 1, w);
				__v4l2_ctrl_modify_range(imx385->vblank, h,
						IMX385_VTS_MAX - imx385->cur_mode->height,
						1, h);
			}
			break;

		default:
			ret = -ENOIOCTLCMD;
			break;
	}

	return ret;
}

static int __imx385_start_stream(struct imx385 *imx385)
{
	int ret;
	u8 reg = 0x00;
	dev_info(&imx385->client->dev, "%s enter\n",__func__);

	ret = reg_write_table(imx385->client, imx385->cur_mode->reg_list);
	if (ret)
		return ret;

	if (imx385->has_init_exp && imx385->cur_mode->hdr_mode != NO_HDR) {
		ret = imx385_ioctl(&imx385->subdev, CNMODULE_HDR_AE_EXP_CFG,
				&imx385->init_hdrae_exp);
		if (ret) {
			dev_err(&imx385->client->dev,
					"init exp fail in hdr mode\n");
			return ret;
		}
	}

	if (imx385->vflip)
		reg |= 0x1;
	if (imx385->hflip)
		reg |= 0x2;

	ret = reg_write(imx385->client, 0x3007, reg);
	if (ret) {
		dev_err(&imx385->client->dev,"%s:reg(0x3007) failed,ret(%d)\n", __func__, ret);
		return ret;
	}

	ret = reg_write(imx385->client, 0x300a, 0xf0);
	ret |= reg_write(imx385->client, 0x300b, 0x00);
	ret |= reg_write(imx385->client, 0x300e, 0x01);
	ret |= reg_write(imx385->client, 0x3089, 0xff);
	ret |= reg_write(imx385->client, 0x308c, 0x00);
	if (ret) {
		dev_err(&imx385->client->dev,"%s:ret(%d)\n",__func__,ret);
		return ret;
	}
	imx385->cur_vts = imx385->cur_mode->vts_def - IMX385_EXP_LINES_MARGIN;

	return reg_write_table(imx385->client, start);
}

static int __imx385_stop_stream(struct imx385 *imx385)
{
	imx385->has_init_exp = false;
	return reg_write_table(imx385->client, stop);
}

static int imx385_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *imx385 = to_imx385(client);
	int ret = 0;

	mutex_lock(&imx385->lock);
	on = !!on;
	if (on == imx385->streaming)
		goto unlock_and_return;

	if (on) {
		ret = __imx385_start_stream(imx385);
		if (ret) {
			dev_err(&imx385->client->dev, "Failed to start imx385 stream\n");
			goto unlock_and_return;
		}
	} else {
		__imx385_stop_stream(imx385);
	}

	imx385->streaming = on;

unlock_and_return:
	mutex_unlock(&imx385->lock);
	return 0;
}

/**
 * @brief
 *
 */
#ifdef CONFIG_COMPAT
static long imx385_compat_ioctl32(struct v4l2_subdev *sd,
		unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct module_inf *inf;
	struct module_awb_cfg *cfg;
	long ret;

	switch (cmd) {
		case CANAANMODULE_GET_MODULE_INFO:
			inf = kzalloc(sizeof(*inf), GFP_KERNEL);
			if (!inf) {
				ret = -ENOMEM;
				return ret;
			}

			ret = imx385_ioctl(sd, cmd, inf);
			if (!ret)
				ret = copy_to_user(up, inf, sizeof(*inf));
			kfree(inf);
			break;
		case CANAANMODULE_AWB_CFG:
			cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
			if (!cfg) {
				ret = -ENOMEM;
				return ret;
			}

			ret = copy_from_user(cfg, up, sizeof(*cfg));
			if (!ret)
				ret = imx385_ioctl(sd, cmd, cfg);
			kfree(cfg);
			break;
		default:
			ret = -ENOIOCTLCMD;
			break;
	}

	return ret;
}
#endif
/**
 * @brief
 *
 * @param sd
 * @param cfg
 * @param fie
 * @return int
 */
static int imx385_enum_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *priv = to_imx385(client);

	if (fie->index >= priv->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;

	return 0;
}

/**
 * @brief
 *
 * @param sd
 * @param config
 * @return int
 */
static int imx385_g_mbus_config(struct v4l2_subdev *sd,
		struct v4l2_mbus_config *config)
{
	u32 val = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *priv = to_imx385(client);
	const struct imx385_mode *mode = priv->cur_mode;

	val = 1 << (IMX385_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;
	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

/* Various V4L2 operations tables */
static struct v4l2_subdev_video_ops imx385_subdev_video_ops = {
	.s_stream = imx385_s_stream,
	.g_frame_interval = imx385_g_frame_interval,
	.g_mbus_config = imx385_g_mbus_config,
};

static struct v4l2_subdev_core_ops imx385_subdev_core_ops = {
	.s_power = imx385_s_power,
	.ioctl = imx385_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx385_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_pad_ops imx385_subdev_pad_ops = {
	.enum_mbus_code = imx385_enum_mbus_code,
	.enum_frame_interval = imx385_enum_frame_interval,
	.set_fmt = imx385_set_fmt,
	.get_fmt = imx385_get_fmt,
};

static struct v4l2_subdev_ops imx385_subdev_ops = {
	.core = &imx385_subdev_core_ops,
	.video = &imx385_subdev_video_ops,
	.pad = &imx385_subdev_pad_ops,
};

static const struct v4l2_ctrl_ops imx385_ctrl_ops = {
	.s_ctrl = imx385_s_ctrl,
};
/**
 * @brief
 *
 * @param client
 * @return int
 */
static int imx385_video_probe(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	u16 model_id;
	u32 lot_id;
	u16 chip_id;
	u16 data_format;
	int ret;

	ret = imx385_s_power(subdev, 1);
	if (ret < 0)
		return ret;

	/* Check and show model, lot, and chip ID. */
	ret = reg_read(client, 0x339a);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (high byte)\n");
		goto done;
	}
	model_id = ret << 8;
	//?
	ret = reg_read(client, 0x3399);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (low byte)\n");
		goto done;
	}
	model_id |= ret;

	ret = reg_read(client, 0x339C);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (mid byte)\n");
		goto done;
	}
	lot_id = ret << 8;

	ret = reg_read(client, 0x339B);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (low byte)\n");
		goto done;
	}
	lot_id |= ret;
	//?
	ret = reg_read(client, 0x339E);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Chip ID (high byte)\n");
		goto done;
	}
	chip_id = ret;

	ret = reg_read(client, 0x33A2);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Data format (high byte)\n");
		goto done;
	}
	data_format = ret<<8;

	ret = reg_read(client, 0x33A1);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Data format (low byte)\n");
		goto done;
	}
	data_format |= ret;

	if (model_id != 0x0385) {
		dev_err(&client->dev, "Model ID: %x not supported!\n",
				model_id);
		ret = -ENODEV;
		goto done;
	}
	dev_info(&client->dev,
			"Model ID 0x%04x, Lot ID 0x%06x, Chip ID 0x%04x,data_format 0x%x\n",
			model_id, lot_id, chip_id,data_format);
done:
	imx385_s_power(subdev, 0);
	return ret;
}
/**
 * @brief
 *
 * @param sd
 * @return int
 */
static int imx385_ctrls_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx385 *priv = to_imx385(client);
	const struct imx385_mode *mode = priv->cur_mode;
	s64 pixel_rate, h_blank, v_blank;
	int ret;
	u32 fps = 0;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 10);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx385_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx385_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* exposure */
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx385_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN,
			IMX385_ANALOGUE_GAIN_MIN,
			IMX385_ANALOGUE_GAIN_MAX,
			1, IMX385_ANALOGUE_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx385_ctrl_ops,
			V4L2_CID_GAIN,
			IMX385_DIGITAL_GAIN_MIN,
			IMX385_DIGITAL_GAIN_MAX, 1,
			IMX385_DIGITAL_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx385_ctrl_ops,
			V4L2_CID_EXPOSURE,
			IMX385_DIGITAL_EXPOSURE_MIN,
			IMX385_DIGITAL_EXPOSURE_MAX, 1,
			IMX385_DIGITAL_EXPOSURE_DEFAULT);

	/* blank */
	h_blank = mode->hts_def - mode->width;
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_HBLANK,
			h_blank, h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_VBLANK,
			v_blank, v_blank, 1, v_blank);

	/* freq */
	v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			0, 0, link_freq_menu_items);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
			mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE,
			0, pixel_rate, 1, pixel_rate);

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &imx385_ctrl_ops,
			V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(tp_qmenu) - 1, 0, 0, tp_qmenu);

	priv->subdev.ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
				priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}
	priv->has_init_exp = false;

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret < 0) {
		dev_err(&client->dev, "Error %d setting default controls\n",
				ret);
		goto error;
	}

	return 0;
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}
/**
 * @brief
 *
 * @param client
 * @param did
 * @return int
 */
static int imx385_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
	struct imx385 *priv;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
			DRIVER_VERSION >> 16,
			(DRIVER_VERSION & 0xff00) >> 8,
			DRIVER_VERSION & 0x00ff);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
				"I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}
	priv = devm_kzalloc(&client->dev, sizeof(struct imx385), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	ret = of_property_read_u32(node, CANAANMODULE_CAMERA_MODULE_INDEX,
			&priv->module_index);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_FACING,
			&priv->module_facing);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_MODULE_NAME,
			&priv->module_name);
	ret |= of_property_read_string(node, CANAANMODULE_CAMERA_LENS_NAME,
			&priv->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	//priv->imx385_powerdown = devm_gpiod_get(dev, "powerdown-gpios", GPIOD_OUT_HIGH);
	//if(IS_ERR(priv->imx385_powerdown))
	//{
	//	dev_err(dev, "get imx385_powerdown err !\n");
	//    return -EINVAL;
	//}

	priv->imx385_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if(IS_ERR(priv->imx385_reset))
	{
		ret = PTR_ERR(priv->imx385_reset);
		dev_err(dev, "%s:Reset imx385 GPIO not setup in DT ret(%d)\n",__func__,ret);
		return ret;
	}

	__imx385_power_on(priv);


	/* 1920 * 1080 by default */
	priv->cur_mode = &supported_modes[0];
	priv->cfg_num = ARRAY_SIZE(supported_modes);

	priv->crop_rect.left = 0x0; //0x2A8
	priv->crop_rect.top = 0x0; //0x2b4;
	priv->crop_rect.width = priv->cur_mode->width;
	priv->crop_rect.height = priv->cur_mode->height;

	v4l2_i2c_subdev_init(&priv->subdev, client, &imx385_subdev_ops);
	ret = imx385_ctrls_init(&priv->subdev);
	if (ret < 0)
		return ret;
	ret = imx385_video_probe(client);
	if (ret < 0)
		return ret;

	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&priv->subdev.entity, 1, &priv->pad);
	if (ret < 0)
		return ret;

	sd = &priv->subdev;
	memset(facing, 0, sizeof(facing));
	if (strcmp(priv->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
			priv->module_index, facing,
			IMX385_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0)
		return ret;

	return ret;
}
/**
 * @brief
 *
 * @param client
 * @return int
 */
static int imx385_remove(struct i2c_client *client)
{
	struct imx385 *priv = to_imx385(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	media_entity_cleanup(&priv->subdev.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return 0;
}

static const struct i2c_device_id imx385_id[] = {
	{"imx385", 0},
	{}
};

static const struct of_device_id imx385_of_match[] = {
	{ .compatible = "sony,imx385" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx385_of_match);

MODULE_DEVICE_TABLE(i2c, imx385_id);
static struct i2c_driver imx385_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx385_of_match),
		.name = IMX385_NAME,
	},
	.probe = imx385_probe,
	.remove = imx385_remove,
	.id_table = imx385_id,
};

module_i2c_driver(imx385_i2c_driver);
MODULE_DESCRIPTION("Sony imx385 Camera driver");
MODULE_LICENSE("GPL v2");
