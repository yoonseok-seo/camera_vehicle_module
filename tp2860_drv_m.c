// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/of_gpio.h>  
/* min/typical/max system clock (xclk) frequencies */
#define TP_XCLK_MIN  6000000
#define TP_XCLK_MAX 54000000 *3

#define m_1920 0


#define APPL_USE_PWR  1

enum tp_mode_id {
	TP_MODE = 0,
	TP_NUM_MODES,
};

enum tp_frame_rate {
	TP_FPS=0,
	TP_NUM_FRAMERATES,
};

enum tp_format_mux {
	TP_FMT_MUX_YUV422 = 0,
	TP_FMT_MUX_RGB,
	TP_FMT_MUX_DITHER,
	TP_FMT_MUX_RAW_DPC,
	TP_FMT_MUX_SNR_RAW,
	TP_FMT_MUX_RAW_CIP,
};

struct tp_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct tp_pixfmt tp_formats[] = {
	{ MEDIA_BUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG, },
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_BE, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_COLORSPACE_SRGB, },
};

/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
static unsigned int virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
		 "MIPI CSI-2 virtual channel (0..3), default 0");

static const int tp_framerates[] = {
	[TP_FPS] = 60,
};

/* regulator supplies */
static const char * const tp_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};




#define TP_NUM_SUPPLIES ARRAY_SIZE(tp_supply_name)

/*
 * Image size under 1280 * 960 are SUBSAMPLING
 * Image size upper 1280 * 960 are SCALING
 */
enum tp_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct tp_mode_info {
	enum tp_mode_id id;
	enum tp_downsize_mode dn_mode;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
};



struct tp_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *light_freq;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

struct tp_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to tpcam */
	u32 xclk_freq;

	struct regulator_bulk_data supplies[TP_NUM_SUPPLIES];

	int pwr_gpio;
  int reset_gpio;

	bool   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count; 

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;

	const struct tp_mode_info *current_mode;
	const struct tp_mode_info *last_mode;
	enum tp_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct tp_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	bool pending_mode_change;
	bool streaming;
};

int tp_g_pwr_gpio;
int tp_g_reset_gpio;
struct tp_dev *tp_g_sensor;



static const struct tp_mode_info
tp_mode_data[TP_NUM_MODES] = {
#if m_1920
	{TP_MODE, SCALING,
	 1920, 2500, 1080, 1120,
	},
#else 	
	{TP_MODE, SCALING,
//	 720, 1892, 240, 740,
	 1280, 1892, 720, 740,
	},
#endif 	
};


static inline struct tp_dev *to_tp_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tp_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct tp_dev,
			     ctrls.handler)->sd;
}

/*
 * After trying the various combinations, reading various
 * documentations spread around the net, and from the various
 * feedback, the clock tree is probably as follows:
 *
 *   +--------------+
 *   |  Ext. Clock  |
 *   +-+------------+
 *     |  +----------+
 *     +->|   PLL1   | - reg 0x3036, for the multiplier
 *        +-+--------+ - reg 0x3037, bits 0-3 for the pre-divider
 *          |  +--------------+
 *          +->| System Clock |  - reg 0x3035, bits 4-7
 *             +-+------------+
 *               |  +--------------+
 *               +->| MIPI Divider | - reg 0x3035, bits 0-3
 *               |  +-+------------+
 *               |    +----------------> MIPI SCLK
 *               |    +  +-----+
 *               |    +->| / 2 |-------> MIPI BIT CLK
 *               |       +-----+
 *               |  +--------------+
 *               +->| PLL Root Div | - reg 0x3037, bit 4
 *                  +-+------------+
 *                    |  +---------+
 *                    +->| Bit Div | - reg 0x3035, bits 0-3
 *                       +-+-------+
 *                         |  +-------------+
 *                         +->| SCLK Div    | - reg 0x3108, bits 0-1
 *                         |  +-+-----------+
 *                         |    +---------------> SCLK
 *                         |  +-------------+
 *                         +->| SCLK 2X Div | - reg 0x3108, bits 2-3
 *                         |  +-+-----------+
 *                         |    +---------------> SCLK 2X
 *                         |  +-------------+
 *                         +->| PCLK Div    | - reg 0x3108, bits 4-5
 *                            ++------------+
 *                             +  +-----------+
 *                             +->|   P_DIV   | - reg 0x3035, bits 0-3
 *                                +-----+-----+
 *                                       +------------> PCLK
 *
 * This is deviating from the datasheet at least for the register
 * 0x3108, since it's said here that the PCLK would be clocked from
 * the PLL.
 *
 * There seems to be also (unverified) constraints:
 *  - the PLL pre-divider output rate should be in the 4-27MHz range
 *  - the PLL multiplier output rate should be in the 500-1000MHz range
 *  - PCLK >= SCLK * 2 in YUV, >= SCLK in Raw or JPEG
 *
 * In the two latter cases, these constraints are met since our
 * factors are hardcoded. If we were to change that, we would need to
 * take this into account. The only varying parts are the PLL
 * multiplier and the system clock divider, which are shared between
 * all these clocks so won't cause any issue.
 */

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 3 in the vendor kernels.
 */
#define TP_PLL_PREDIV	3

#define TP_PLL_MULT_MIN	4
#define TP_PLL_MULT_MAX	252

/*
 * This is supposed to be ranging from 1 to 16, but the value is
 * always set to either 1 or 2 in the vendor kernels.
 */
#define TP_SYSDIV_MIN	1
#define TP_SYSDIV_MAX	16

/*
 * Hardcode these values for scaler and non-scaler modes.
 * FIXME: to be re-calcualted for 1 data lanes setups
 */
#define TP_MIPI_DIV_PCLK	2
#define TP_MIPI_DIV_SCLK	1

/*
 * This is supposed to be ranging from 1 to 2, but the value is always
 * set to 2 in the vendor kernels.
 */
#define TP_PLL_ROOT_DIV			2
#define TP_PLL_CTRL3_PLL_ROOT_DIV_2		BIT(4)

/*
 * We only supports 8-bit formats at the moment
 */
#define TP_BIT_DIV				2
#define TP_PLL_CTRL0_MIPI_MODE_8BIT		0x08

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 2 in the vendor kernels.
 */
#define TP_SCLK_ROOT_DIV	2

/*
 * This is hardcoded so that the consistency is maintained between SCLK and
 * SCLK 2x.
 */
#define TP_SCLK2X_ROOT_DIV (TP_SCLK_ROOT_DIV / 2)

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 1 in the vendor kernels.
 */
#define TP_PCLK_ROOT_DIV			1
#define TP_PLL_SYS_ROOT_DIVIDER_BYPASS	0x00





#define tp2860_DEFAULT_SLAVE_ID ( 0x88 >> 1)  // 0x8a

#define CVBS_960H 0 //1->960H 0->720H


enum
{		VIN1=0,
    VIN2=1,
    VIN3=2,
    VIN4=3,
};
enum
{    STD_TVI,
    STD_HDA, //AHD
};
enum
{    PAL,
    NTSC,
    HD25,
    HD30,
    FHD25,
    FHD30,
    HD50,
    HD60,
    QHD25,  //only support with 2lane mode
    QHD30,	//only support with 2lane mode
    FHD50,	//only support with 2lane mode
    FHD60,	//only support with 2lane mode
};
enum
{    MIPI_2LANE,
    MIPI_1LANE,
};

//static int tp2860_write_reg(struct tp2860_dev *sensor, u8 reg, u8 val)
static int tp2860_write_reg(u8 reg, u8 val)
{
//	struct i2c_client *client = sensor->i2c_client;
	struct i2c_client *client = tp_g_sensor->i2c_client; //sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;

	msg.addr = tp2860_DEFAULT_SLAVE_ID;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}


//static int tp2860_read_reg(tp2860_dev *sensor, u8 reg, u8 *val)
u8 tp2860_read_reg(u8 reg)
{
	struct i2c_client *client = tp_g_sensor->i2c_client; //sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;


	buf[0] = reg;

	msg[0].addr = tp2860_DEFAULT_SLAVE_ID;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = 1;//sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x\n",
			__func__, reg);
		return ret;
	}

	return buf[0];
	return 0;
}


void tp2860_mipi_out(unsigned char fmt, unsigned char std, unsigned char lane)
{	
	u8 tmp;
	//mipi setting
	tp2860_write_reg(0x40, 0x08); //select MIPI page
	
	
	
	tp2860_write_reg(0x02, 0x7d);
	tp2860_write_reg(0x03, 0x75);
	tp2860_write_reg(0x04, 0x75);
	tp2860_write_reg(0x13, 0xef);	
	tp2860_write_reg(0x20, 0x00);	
	tp2860_write_reg(0x23, 0x9e);	
	
	if(MIPI_1LANE == lane)
	
{		tp2860_write_reg(0x21, 0x11);
		
		if(FHD30 == fmt || FHD25 == fmt)
		
{			if(STD_HDA == std)
			{	
	    		tp2860_write_reg(0x14, 0x07);	
					tp2860_write_reg(0x15, 0x05);	
			}
			else
			
{	    		tp2860_write_reg(0x14, 0x00);	
					tp2860_write_reg(0x15, 0x02);			
			}			
			tp2860_write_reg(0x2a, 0x08);
			tp2860_write_reg(0x2b, 0x06);
			tp2860_write_reg(0x2c, 0x11);
			tp2860_write_reg(0x2e, 0x0a);				
		}
		else if(HD30 == fmt || HD25 == fmt)
   {	
			if(STD_HDA == std)
      {
      		tp2860_write_reg(0x14, 0x47);	
					tp2860_write_reg(0x15, 0x09);	
			}
			else
     {
		    		tp2860_write_reg(0x14, 0x00);	
					tp2860_write_reg(0x15, 0x12);			
			}					
			tp2860_write_reg(0x2a, 0x04);
			tp2860_write_reg(0x2b, 0x03);
			tp2860_write_reg(0x2c, 0x09);
			tp2860_write_reg(0x2e, 0x02);					
		}
		else if(NTSC == fmt || PAL == fmt)
   {	   
	    tp2860_write_reg(0x14, 0x51);	
			tp2860_write_reg(0x15, 0x07);	
			
			tp2860_write_reg(0x2a, 0x02);
			tp2860_write_reg(0x2b, 0x01);
			tp2860_write_reg(0x2c, 0x05);
			tp2860_write_reg(0x2e, 0x02);							
		}
		else if(HD60 == fmt || HD50 == fmt)
    {
	   		tp2860_write_reg(0x14, 0x00);	
			tp2860_write_reg(0x15, 0x02);			
	
			tp2860_write_reg(0x2a, 0x08);
			tp2860_write_reg(0x2b, 0x06);
			tp2860_write_reg(0x2c, 0x11);
			tp2860_write_reg(0x2e, 0x0a);				
		}	
	}	    
	else //2lane
  {	  
	    tp2860_write_reg(0x21, 0x12);
			if(FHD30 == fmt || FHD25 == fmt)
      {				
      	if(STD_HDA == std)
				{	

			    tp2860_write_reg(0x14, 0x40);			
					tp2860_write_reg(0x15, 0x05);	
				}
				else
        {
				    tp2860_write_reg(0x14, 0x41);
					tp2860_write_reg(0x15, 0x02);			
				}			
				tp2860_write_reg(0x2a, 0x04);
				tp2860_write_reg(0x2b, 0x03);
				tp2860_write_reg(0x2c, 0x09);
				tp2860_write_reg(0x2e, 0x02);				
			}
			else if(HD30 == fmt || HD25 == fmt)
      {				
        if(STD_HDA == std)
        {
				          tp2860_write_reg(0x14, 0x50);				
					        tp2860_write_reg(0x15, 0x09);	
 	      }
				else
        {
 				    tp2860_write_reg(0x14, 0x41);
					  tp2860_write_reg(0x15, 0x12);			
				}			
				tp2860_write_reg(0x2a, 0x02);
				tp2860_write_reg(0x2b, 0x01);
				tp2860_write_reg(0x2c, 0x05);
				tp2860_write_reg(0x2e, 0x02);					
			}
			else if(NTSC == fmt || PAL == fmt)
     {			  
	      tp2860_write_reg(0x14, 0x62);
				tp2860_write_reg(0x15, 0x07);	
			
				tp2860_write_reg(0x2a, 0x02);
				tp2860_write_reg(0x2b, 0x00);
				tp2860_write_reg(0x2c, 0x03);
				tp2860_write_reg(0x2e, 0x02);						
			}
			else if(HD60 == fmt || HD50 == fmt)
      {		    
	      tp2860_write_reg(0x14, 0x41);
				tp2860_write_reg(0x15, 0x02);			
			
				tp2860_write_reg(0x2a, 0x04);
				tp2860_write_reg(0x2b, 0x03);
				tp2860_write_reg(0x2c, 0x09);
				tp2860_write_reg(0x2e, 0x02);				
			}			
			else if(QHD30 == fmt || QHD25 == fmt || FHD60 == fmt || FHD50 == fmt)
      {			  
	      tp2860_write_reg(0x14, 0x00);
				tp2860_write_reg(0x15, 0x01);			
								
				tp2860_write_reg(0x2a, 0x08);
				tp2860_write_reg(0x2b, 0x06);
				tp2860_write_reg(0x2c, 0x11);
				tp2860_write_reg(0x2e, 0x0a);					
			}			
	}


	tmp = tp2860_read_reg(0x14); //PLL reset
	tp2860_write_reg(0x14, 0x80|tmp);
	tp2860_write_reg(0x14, tmp);
	
	/* Enable MIPI CSI2 output */
	tp2860_write_reg(0x28, 0x02);	 //stream off 
	//tp2860_write_reg(0x28, 0x00);	 //stream on
	tp2860_write_reg(0x40, 0x00); //back to decoder page
	
}




#if 1
void tp2860_dum_reg(void)
{
	u8 tmp;
	int index;
	
	
	#if 1
//	  tp2860_write_reg(0x40, 0x00); //select decoder page
//	  pr_err(" %s  index = 0x%02x   value = 0x%02x \n",__func__,0x01, tp2860_read_reg(0x01));
		
	for(index=0; index < 0xff; index++)
	{
		pr_err(" index = 0x%02x   value = 0x%02x \n",index, tp2860_read_reg(index));
	}
	
	
	  return ;
	
#endif	
	


	


	tp2860_write_reg(0x40, 0x08); //select MIPI page
	
	tp2860_write_reg(0x28, 0x02);	 //stream off 

//	tp2860_write_reg(0x22, 0x80);
//	tp2860_write_reg(0x22, 0xa0);
	//tp2860_write_reg(0x22, 0x80);
	tmp = tp2860_read_reg(0x22);
	tp2860_write_reg(0x22, tmp|0x80);
//	tp2860_write_reg(0x23, 0x1e);
//	pr_err("%s  index = 0x%02x   value = 0x%02x \n",__func__, 0x22, tp2860_read_reg(0x22)); //tp_ena

//	pr_err(" index = 0x%02x   value = 0x%02x \n",0x23, tp2860_read_reg(0x23));
		/* Enable MIPI CSI2 output */
//	tp2860_write_reg(0x28, 0x02);	 //stream off 
//	tp2860_write_reg(0x28, 0x00);	 //stream on
//	tp2860_write_reg(0x40, 0x00); //back to decoder page
	
	return;
	
	pr_err("decoder page \n");
	tp2860_write_reg(0x40, 0x00); //select decoder page
	for(index=0; index < 0xff; index++)
	{
		pr_err(" index = 0x%02x   value = 0x%02x \n",index, tp2860_read_reg(index));
	}

	pr_err("mipi page \n");
	tp2860_write_reg(0x40, 0x08); //select MIPI page

	for(index=0; index < 0xff; index++)
	{
		pr_err(" index = 0x%02x   value = 0x%02x \n",index, tp2860_read_reg(index));
	}
  return;

}	
#endif	
	

void tp2860_readid(void)
{
	u8 tmp;
	
	
	tmp = tp2860_read_reg(0x00);
	pr_err("%s   chip id %d\n",__func__, tmp);
	
}



void mipi_stream_on(void)
{
	tp2860_write_reg(0x40, 0x08); //select MIPI page
		/* Enable MIPI CSI2 output */
//	tp2860_write_reg(0x28, 0x02);	 //stream off 
	tp2860_write_reg(0x28, 0x00);	 //stream on
	tp2860_write_reg(0x40, 0x00); //back to decoder page

}

void mipi_stream_off(void)
{
	tp2860_write_reg(0x40, 0x08); //select MIPI page
		/* Enable MIPI CSI2 output */
	tp2860_write_reg(0x28, 0x02);	 //stream off 
	//tp2860_write_reg(0x28, 0x00);	 //stream on
	tp2860_write_reg(0x40, 0x00); //back to decoder page

}
/////////////////////////////////
//ch: video channel
//fmt: PAL/NTSC/HD25/HD30
//std: STD_TVI/STD_HDA
//lane: MIPI_2LANE/MIPI_1LANE
//sample: tp2860_sensor_init(VIN1,HD30,STD_TVI,MIPI_2LANE); //video is TVI 720p30 from Vin1
////////////////////////////////
void tp2860_sensor_init(unsigned char ch,unsigned char fmt,unsigned char std, unsigned char lane)

{	  
	u8 tmp;
	
	
	
	pr_err("%s  start \n",__func__);
	
	tp2860_write_reg(0x40, 0x00); //select decoder page
	tp2860_write_reg(0x06, 0x12); //default value	
	tp2860_write_reg(0x42, 0x00);	//common setting for all format	
	tp2860_write_reg(0x4e, 0x00); //common setting for MIPI output
	tp2860_write_reg(0x54, 0x00); //common setting for MIPI output
	tp2860_write_reg(0x41, ch);		//video MUX select
	
	
	 
	
	if(PAL == fmt)
	
{
	#if CVBS_960H	 
		tp2860_write_reg(0x02, 0x47);
		tp2860_write_reg(0x0c, 0x13); 
		tp2860_write_reg(0x0d, 0x51);  

		tp2860_write_reg(0x15, 0x13);
		tp2860_write_reg(0x16, 0x76); 
		tp2860_write_reg(0x17, 0x80); 
		tp2860_write_reg(0x18, 0x17);
		tp2860_write_reg(0x19, 0x20);
		tp2860_write_reg(0x1a, 0x17);				
		tp2860_write_reg(0x1c, 0x09);
		tp2860_write_reg(0x1d, 0x48);
	
		tp2860_write_reg(0x20, 0x48);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x37);
		tp2860_write_reg(0x23, 0x3f);

		tp2860_write_reg(0x2b, 0x70);  
		tp2860_write_reg(0x2c, 0x2a); 
		tp2860_write_reg(0x2d, 0x64);
		tp2860_write_reg(0x2e, 0x56);

		tp2860_write_reg(0x30, 0x7a);  
		tp2860_write_reg(0x31, 0x4a); 
		tp2860_write_reg(0x32, 0x4d);
		tp2860_write_reg(0x33, 0xf0);	
		
		tp2860_write_reg(0x35, 0x65); 
		tp2860_write_reg(0x38, 0x00);				
		tp2860_write_reg(0x39, 0x04); 
			
#else //PAL 720H

		tp2860_write_reg(0x02, 0x47);
		tp2860_write_reg(0x06, 0x32);		
		tp2860_write_reg(0x0c, 0x13); 
		tp2860_write_reg(0x0d, 0x51);  

		tp2860_write_reg(0x15, 0x03);
		tp2860_write_reg(0x16, 0xf0); 
		tp2860_write_reg(0x17, 0xa0); 
		tp2860_write_reg(0x18, 0x17);
		tp2860_write_reg(0x19, 0x20);
		tp2860_write_reg(0x1a, 0x15);				
		tp2860_write_reg(0x1c, 0x06);
		tp2860_write_reg(0x1d, 0xc0);
	
		tp2860_write_reg(0x20, 0x48);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x37);
		tp2860_write_reg(0x23, 0x3f);

		tp2860_write_reg(0x2b, 0x70);  
		tp2860_write_reg(0x2c, 0x2a); 
		tp2860_write_reg(0x2d, 0x4b);
		tp2860_write_reg(0x2e, 0x56);

		tp2860_write_reg(0x30, 0x7a);  
		tp2860_write_reg(0x31, 0x4a); 
		tp2860_write_reg(0x32, 0x4d);
		tp2860_write_reg(0x33, 0xfb);	
		
		tp2860_write_reg(0x35, 0x65); 
		tp2860_write_reg(0x38, 0x00);				
		tp2860_write_reg(0x39, 0x04); 	
#endif			
	}
	else if(NTSC == fmt)
	
{
	#if CVBS_960H		
		tp2860_write_reg(0x02, 0x47);
		tp2860_write_reg(0x0c, 0x13); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x13);
		tp2860_write_reg(0x16, 0x60); 

//		tp2860_write_reg(0x17, 0x80); 
//		tp2860_write_reg(0x18, 0x12);
//		tp2860_write_reg(0x19, 0xf0);
//		tp2860_write_reg(0x1a, 0x07);				

		tp2860_write_reg(0x17, 0xa0); 
		tp2860_write_reg(0x18, 0x11);
		tp2860_write_reg(0x19, 0xf1);
		tp2860_write_reg(0x1a, 0x05);				


		tp2860_write_reg(0x1c, 0x09);
		tp2860_write_reg(0x1d, 0x38);
	
		tp2860_write_reg(0x20, 0x40);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x2b, 0x70);  
		tp2860_write_reg(0x2c, 0x2a); 
		tp2860_write_reg(0x2d, 0x68);
		tp2860_write_reg(0x2e, 0x57);

		tp2860_write_reg(0x30, 0x62);  
		tp2860_write_reg(0x31, 0xbb); 
		tp2860_write_reg(0x32, 0x96);
		tp2860_write_reg(0x33, 0xc0);
		
		tp2860_write_reg(0x35, 0x65); 
		tp2860_write_reg(0x38, 0x00);			
		tp2860_write_reg(0x39, 0x04); 
		
#else	//NTSC 720H
		
		tp2860_write_reg(0x02, 0x47);
		tp2860_write_reg(0x0c, 0x13); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x03);
		tp2860_write_reg(0x16, 0xd6); 
		tp2860_write_reg(0x17, 0xa0); 
		tp2860_write_reg(0x18, 0x12);
		tp2860_write_reg(0x19, 0xf0);
		tp2860_write_reg(0x1a, 0x05);				
		tp2860_write_reg(0x1c, 0x06);
		tp2860_write_reg(0x1d, 0xb4);
	
		tp2860_write_reg(0x20, 0x40);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x2b, 0x70);  
		tp2860_write_reg(0x2c, 0x2a); 
		tp2860_write_reg(0x2d, 0x4b);
		tp2860_write_reg(0x2e, 0x57);

		tp2860_write_reg(0x30, 0x62);  
		tp2860_write_reg(0x31, 0xbb); 
		tp2860_write_reg(0x32, 0x96);
		tp2860_write_reg(0x33, 0xcb);
		
		tp2860_write_reg(0x35, 0x65); 
		tp2860_write_reg(0x38, 0x00);			
		tp2860_write_reg(0x39, 0x04); 			
#endif		
	}
	else if(HD25 == fmt)
{		
	  tp2860_write_reg(0x02, 0x42);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x13); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x13);
		tp2860_write_reg(0x16, 0x15); 
		tp2860_write_reg(0x17, 0x00); 
		tp2860_write_reg(0x18, 0x19);
		tp2860_write_reg(0x19, 0xd0);
		tp2860_write_reg(0x1a, 0x25);			
		tp2860_write_reg(0x1c, 0x07);  //1280*720, 25fps
		tp2860_write_reg(0x1d, 0xbc);  //1280*720, 25fps

		tp2860_write_reg(0x20, 0x30);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x0a); 
		tp2860_write_reg(0x2d, 0x30);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x48);  
		tp2860_write_reg(0x31, 0xbb); 
		tp2860_write_reg(0x32, 0x2e);
		tp2860_write_reg(0x33, 0x90);
		
		tp2860_write_reg(0x35, 0x25); 
		tp2860_write_reg(0x38, 0x00);	
		tp2860_write_reg(0x39, 0x18); 

		if(STD_HDA == std)  //AHD720p25 extra
		
{    	tp2860_write_reg(0x0d, 0x70);

    	tp2860_write_reg(0x16, 0x16);
    	tp2860_write_reg(0x1c, 0x87);
    	tp2860_write_reg(0x1d, 0xba);
    	    	    	
    	tp2860_write_reg(0x20, 0x38);
    	tp2860_write_reg(0x21, 0x46);

    	tp2860_write_reg(0x27, 0xad);
    	
    	tp2860_write_reg(0x2c, 0x3a);
    	tp2860_write_reg(0x2d, 0x48);
    	tp2860_write_reg(0x2e, 0x40);

    	tp2860_write_reg(0x30, 0x4f);
    	tp2860_write_reg(0x31, 0x10);
    	tp2860_write_reg(0x32, 0x08);
    	tp2860_write_reg(0x33, 0x40);
		}	
	}
	else if(HD30 == fmt)
{		
	pr_err("%s  HD30 \n",__func__);	

	  tp2860_write_reg(0x02, 0x42);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x13); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x13);
		tp2860_write_reg(0x16, 0x15); 
		tp2860_write_reg(0x17, 0x00); 
		tp2860_write_reg(0x18, 0x19);
		tp2860_write_reg(0x19, 0xd0);
		tp2860_write_reg(0x1a, 0x25);			
		tp2860_write_reg(0x1c, 0x06);  //1280*720, 30fps
		tp2860_write_reg(0x1d, 0x72);  //1280*720, 30fps

		tp2860_write_reg(0x20, 0x30);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x0a); 
		tp2860_write_reg(0x2d, 0x30);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x48);  
		tp2860_write_reg(0x31, 0xbb); 
		tp2860_write_reg(0x32, 0x2e);
		tp2860_write_reg(0x33, 0x90);
		
		tp2860_write_reg(0x35, 0x25); 
		tp2860_write_reg(0x38, 0x00);	
		tp2860_write_reg(0x39, 0x18); 

		if(STD_HDA == std) //AHD720p30 extra
		
    {    
	
		pr_err("%s  HD30 STD_HDA\n",__func__);	

	    tp2860_write_reg(0x0d, 0x70);
    	
    	tp2860_write_reg(0x16, 0x16);
    	tp2860_write_reg(0x1c, 0x86);
    	tp2860_write_reg(0x1d, 0x70);
    	
    	tp2860_write_reg(0x20, 0x38);
    	tp2860_write_reg(0x21, 0x46);

    	tp2860_write_reg(0x27, 0xad);

    	tp2860_write_reg(0x2c, 0x3a);
    	tp2860_write_reg(0x2d, 0x48);
    	tp2860_write_reg(0x2e, 0x40);

    	tp2860_write_reg(0x30, 0x4e);
    	tp2860_write_reg(0x31, 0xe5);
    	tp2860_write_reg(0x32, 0x00);
    	tp2860_write_reg(0x33, 0xf0);
		}	
	}
	else if(FHD30 == fmt)
	{	
		
	pr_err("%s  FHD30 \n",__func__);	
		
//			tp2860_write_reg(0x02, 0x40);
//s.y.s
			tp2860_write_reg(0x02, 0x50);
			tp2860_write_reg(0x07, 0xc0); 
			tp2860_write_reg(0x0b, 0xc0);  		
			tp2860_write_reg(0x0c, 0x03); 
			tp2860_write_reg(0x0d, 0x50);  

			tp2860_write_reg(0x15, 0x03);
			tp2860_write_reg(0x16, 0xd2); 
			tp2860_write_reg(0x17, 0x80); 
			tp2860_write_reg(0x18, 0x29);
			tp2860_write_reg(0x19, 0x38);
			tp2860_write_reg(0x1a, 0x47);				
			tp2860_write_reg(0x1c, 0x08);  //1920*1080, 30fps
			tp2860_write_reg(0x1d, 0x98);  //
	
			tp2860_write_reg(0x20, 0x30);  
			tp2860_write_reg(0x21, 0x84); 
			tp2860_write_reg(0x22, 0x36);
			tp2860_write_reg(0x23, 0x3c);

			tp2860_write_reg(0x2b, 0x60);  
			tp2860_write_reg(0x2c, 0x0a); 
			tp2860_write_reg(0x2d, 0x30);
			tp2860_write_reg(0x2e, 0x70);

			tp2860_write_reg(0x30, 0x48);  
			tp2860_write_reg(0x31, 0xbb); 
			tp2860_write_reg(0x32, 0x2e);
			tp2860_write_reg(0x33, 0x90);
			
			tp2860_write_reg(0x35, 0x05);
			tp2860_write_reg(0x38, 0x00); 
			tp2860_write_reg(0x39, 0x1C); 	
	
			if(STD_HDA == std) //AHD1080p30 extra
      {      
	    	pr_err("%s  STD_HDA \n",__func__);	

	    	tp2860_write_reg(0x0d, 0x70);
    
    		tp2860_write_reg(0x15, 0x01);
    		tp2860_write_reg(0x16, 0xf0);
    		tp2860_write_reg(0x1c, 0x88);
    		tp2860_write_reg(0x1d, 0x96);
    		    
    		tp2860_write_reg(0x20, 0x38);
    		tp2860_write_reg(0x21, 0x46);

    		tp2860_write_reg(0x27, 0xad);

    		tp2860_write_reg(0x2c, 0x3a);
    		tp2860_write_reg(0x2d, 0x48);
    		tp2860_write_reg(0x2e, 0x40);

    		tp2860_write_reg(0x30, 0x52);
    		tp2860_write_reg(0x31, 0xca);
    		tp2860_write_reg(0x32, 0xf0);
    		tp2860_write_reg(0x33, 0x20);
    		
    		tp2860_write_reg(0x35, 0x25);    		
			}
		}
		else if(FHD25 == fmt)
   {			
	    tp2860_write_reg(0x02, 0x40);
			tp2860_write_reg(0x07, 0xc0); 
			tp2860_write_reg(0x0b, 0xc0);  		
			tp2860_write_reg(0x0c, 0x03); 
			tp2860_write_reg(0x0d, 0x50);  
  		
			tp2860_write_reg(0x15, 0x03);
			tp2860_write_reg(0x16, 0xd2); 
			tp2860_write_reg(0x17, 0x80); 
			tp2860_write_reg(0x18, 0x29);
			tp2860_write_reg(0x19, 0x38);
			tp2860_write_reg(0x1a, 0x47);				
  		
			tp2860_write_reg(0x1c, 0x0a);  //1920*1080, 25fps
			tp2860_write_reg(0x1d, 0x50);  //
			
			tp2860_write_reg(0x20, 0x30);  
			tp2860_write_reg(0x21, 0x84); 
			tp2860_write_reg(0x22, 0x36);
			tp2860_write_reg(0x23, 0x3c);
  		
			tp2860_write_reg(0x2b, 0x60);  
			tp2860_write_reg(0x2c, 0x0a); 
			tp2860_write_reg(0x2d, 0x30);
			tp2860_write_reg(0x2e, 0x70);
  		
			tp2860_write_reg(0x30, 0x48);  
			tp2860_write_reg(0x31, 0xbb); 
			tp2860_write_reg(0x32, 0x2e);
			tp2860_write_reg(0x33, 0x90);
					
			tp2860_write_reg(0x35, 0x05);
			tp2860_write_reg(0x38, 0x00); 
			tp2860_write_reg(0x39, 0x1C); 	

			if(STD_HDA == std)  //AHD1080p25 extra                   
			{                                     
   			tp2860_write_reg(0x0d, 0x70);
   			
   			tp2860_write_reg(0x15, 0x01);
   			tp2860_write_reg(0x16, 0xf0);
    		tp2860_write_reg(0x1c, 0x8a);
    		tp2860_write_reg(0x1d, 0x4e);
    		   			
   			tp2860_write_reg(0x20, 0x3c);
   			tp2860_write_reg(0x21, 0x46);
   			
   			tp2860_write_reg(0x27, 0xad);
   			
   			tp2860_write_reg(0x2c, 0x3a);
   			tp2860_write_reg(0x2d, 0x48);
   			tp2860_write_reg(0x2e, 0x40);
   			
   			tp2860_write_reg(0x30, 0x52);
   			tp2860_write_reg(0x31, 0xc3);
   			tp2860_write_reg(0x32, 0x7d);
   			tp2860_write_reg(0x33, 0xa0);
   			
    		tp2860_write_reg(0x35, 0x25);    		   			
			}			
		}
	else if(HD50 == fmt)
	{		tp2860_write_reg(0x02, 0x42);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x03); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x13);
		tp2860_write_reg(0x16, 0x15); 
		tp2860_write_reg(0x17, 0x00); 
		tp2860_write_reg(0x18, 0x19);
		tp2860_write_reg(0x19, 0xd0);
		tp2860_write_reg(0x1a, 0x25);			
		tp2860_write_reg(0x1c, 0x07);  //1280*720, 25fps
		tp2860_write_reg(0x1d, 0xbc);  //1280*720, 25fps

		tp2860_write_reg(0x20, 0x30);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x1a); 
		tp2860_write_reg(0x2d, 0x30);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x48);  
		tp2860_write_reg(0x31, 0xbb); 
		tp2860_write_reg(0x32, 0x2e);
		tp2860_write_reg(0x33, 0x90);
		
		tp2860_write_reg(0x35, 0x05); 
		tp2860_write_reg(0x38, 0x00);	
		tp2860_write_reg(0x39, 0x1C); 

		if(STD_HDA == std)//subcarrier 22M
		
{    			tp2860_write_reg(0x18, 0x1b);
    			
    			tp2860_write_reg(0x20, 0x40);
    			tp2860_write_reg(0x21, 0x46);

    			tp2860_write_reg(0x25, 0xfe);
    			tp2860_write_reg(0x26, 0x01);

    			tp2860_write_reg(0x2c, 0x3a);
    			tp2860_write_reg(0x2d, 0x48);
    			tp2860_write_reg(0x2e, 0x40);

    			tp2860_write_reg(0x30, 0x29);
    			tp2860_write_reg(0x31, 0x67);
    			tp2860_write_reg(0x32, 0xF3);
    			tp2860_write_reg(0x33, 0x90);
		}

	}	
	
	else if(HD60 == fmt)
  {		
	  tp2860_write_reg(0x02, 0x42);


		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x03); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x13);
		tp2860_write_reg(0x16, 0x15); 
		tp2860_write_reg(0x17, 0x00); 
		tp2860_write_reg(0x18, 0x19);
		tp2860_write_reg(0x19, 0xd0);
		tp2860_write_reg(0x1a, 0x25);			
		tp2860_write_reg(0x1c, 0x06);  //1280*720, 60fps
		tp2860_write_reg(0x1d, 0x72);  //1280*720, 60fps

		tp2860_write_reg(0x20, 0x30);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x1a); 
		tp2860_write_reg(0x2d, 0x30);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x48);  
		tp2860_write_reg(0x31, 0xbb); 
		tp2860_write_reg(0x32, 0x2e);
		tp2860_write_reg(0x33, 0x90);
		
		tp2860_write_reg(0x35, 0x05); 
		tp2860_write_reg(0x38, 0x00);	
		tp2860_write_reg(0x39, 0x1C); 

		if(STD_HDA == std)//subcarrier 22M
		
{    			
	        tp2860_write_reg(0x18, 0x1b);
    			
    			tp2860_write_reg(0x20, 0x40);
    			tp2860_write_reg(0x21, 0x46);

    			tp2860_write_reg(0x25, 0xfe);
    			tp2860_write_reg(0x26, 0x01);

    			tp2860_write_reg(0x2c, 0x3a);
    			tp2860_write_reg(0x2d, 0x48);
    			tp2860_write_reg(0x2e, 0x40);

    			tp2860_write_reg(0x30, 0x29);
    			tp2860_write_reg(0x31, 0x62);
    			tp2860_write_reg(0x32, 0xFC);
    			tp2860_write_reg(0x33, 0x96);
		}

	}	
	else if(QHD30 == fmt)
  {		
	  tp2860_write_reg(0x02, 0x50);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x03); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x23);
		tp2860_write_reg(0x16, 0x1b); 
		tp2860_write_reg(0x17, 0x00); 
		tp2860_write_reg(0x18, 0x38);
		tp2860_write_reg(0x19, 0xa0);
		tp2860_write_reg(0x1a, 0x5a);				
		tp2860_write_reg(0x1c, 0x0c);  //2560*1440, 30fps
		tp2860_write_reg(0x1d, 0xe2);  //
	
		tp2860_write_reg(0x20, 0x50);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x27, 0xad);
		
		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x2a); 
		tp2860_write_reg(0x2d, 0x58);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x74);  
		tp2860_write_reg(0x31, 0x58); 
		tp2860_write_reg(0x32, 0x9f);
		tp2860_write_reg(0x33, 0x60);
			
		tp2860_write_reg(0x35, 0x15);
		tp2860_write_reg(0x36, 0xdc);		
		tp2860_write_reg(0x38, 0x40); 
		tp2860_write_reg(0x39, 0x48); 	

		if(STD_HDA == std)                    
		{   
#if 0
	    tmp = tp2860_read_reg(0x14);
	    tmp |= 0x40;
	    tp2860_write_reg(0x14, tmp);
#else
	    tmp = tp2860_read_reg(0x14);
	    tmp &= 0x9f;
      tp2860_write_reg(0x14, tmp);
	    tmp = tp2860_read_reg(0x1c);
	    tmp |= 0x80;
	    tp2860_write_reg(0x1c, tmp);
	    tp2860_write_reg(0x0d, 0x70);
	    tp2860_write_reg(0x0e, 0x0b);
#endif

    	tp2860_write_reg(0x13, 0x00);
    	tp2860_write_reg(0x15, 0x23);
    	tp2860_write_reg(0x16, 0x16);
    	tp2860_write_reg(0x18, 0x32);

    	tp2860_write_reg(0x20, 0x80);
    	tp2860_write_reg(0x21, 0x86);
    	tp2860_write_reg(0x22, 0x36);

    	tp2860_write_reg(0x2b, 0x60);
    	tp2860_write_reg(0x2d, 0xa0);
    	tp2860_write_reg(0x2e, 0x40);

    	tp2860_write_reg(0x30, 0x48);
    	tp2860_write_reg(0x31, 0x6a);
    	tp2860_write_reg(0x32, 0xbe);
    	tp2860_write_reg(0x33, 0x80);
 
    	tp2860_write_reg(0x39, 0x40);
		}
   				
	}
	else if(QHD25 == fmt)
  {		
	  tp2860_write_reg(0x02, 0x50);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x03); 
		tp2860_write_reg(0x0d, 0x50);  
  		
		tp2860_write_reg(0x15, 0x23);
		tp2860_write_reg(0x16, 0x1b); 
		tp2860_write_reg(0x17, 0x00); 
		tp2860_write_reg(0x18, 0x38);
		tp2860_write_reg(0x19, 0xa0);
		tp2860_write_reg(0x1a, 0x5a);				
		tp2860_write_reg(0x1c, 0x0f);  //2560*1440, 25fps
		tp2860_write_reg(0x1d, 0x76);  //
			
		tp2860_write_reg(0x20, 0x50);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x27, 0xad);
		
		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x2a); 
		tp2860_write_reg(0x2d, 0x58);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x74);  
		tp2860_write_reg(0x31, 0x58); 
		tp2860_write_reg(0x32, 0x9f);
		tp2860_write_reg(0x33, 0x60);
			
		tp2860_write_reg(0x35, 0x15);
		tp2860_write_reg(0x36, 0xdc);		
		tp2860_write_reg(0x38, 0x40); 
		tp2860_write_reg(0x39, 0x48); 	

		if(STD_HDA == std)                    
		{   
			                                  
#if 0
	    tmp = tp2860_read_reg(0x14);
	    tmp |= 0x40;
	    tp2860_write_reg(0x14, tmp);
#else
	    tmp = tp2860_read_reg(0x14);
	    tmp &= 0x9f;
	    tp2860_write_reg(0x14, tmp);
	    tmp = tp2860_read_reg(0x1c);
	    tmp |= 0x80;
	    tp2860_write_reg(0x1c, tmp);
	    tp2860_write_reg(0x0d, 0x70);
	    tp2860_write_reg(0x0e, 0x0b);
#endif

    	tp2860_write_reg(0x13, 0x00);
    	tp2860_write_reg(0x15, 0x23);
    	tp2860_write_reg(0x16, 0x16);
    	tp2860_write_reg(0x18, 0x32);

    	tp2860_write_reg(0x20, 0x80);
    	tp2860_write_reg(0x21, 0x86);
    	tp2860_write_reg(0x22, 0x36);

    	tp2860_write_reg(0x2b, 0x60);
    	tp2860_write_reg(0x2d, 0xa0);
    	tp2860_write_reg(0x2e, 0x40);
    
    	tp2860_write_reg(0x30, 0x48);
    	tp2860_write_reg(0x31, 0x6f);
    	tp2860_write_reg(0x32, 0xb5);
    	tp2860_write_reg(0x33, 0x80);
 
    	tp2860_write_reg(0x39, 0x40);
		}
   				
	}
		
	else if(FHD60 == fmt)
	
{		
		tp2860_write_reg(0x02, 0x50);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x03); 
		tp2860_write_reg(0x0d, 0x50);  

		tp2860_write_reg(0x15, 0x03);
		tp2860_write_reg(0x16, 0xf0); 
		tp2860_write_reg(0x17, 0x80); 
		tp2860_write_reg(0x18, 0x12);
		tp2860_write_reg(0x19, 0x38);
		tp2860_write_reg(0x1a, 0x47);				
		tp2860_write_reg(0x1c, 0x08);  //
		tp2860_write_reg(0x1d, 0x96);  //
	
		tp2860_write_reg(0x20, 0x38);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x27, 0xad);
		    
		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x0a); 
		tp2860_write_reg(0x2d, 0x40);
		tp2860_write_reg(0x2e, 0x70);

		tp2860_write_reg(0x30, 0x74);  
		tp2860_write_reg(0x31, 0x9b); 
		tp2860_write_reg(0x32, 0xa5);
		tp2860_write_reg(0x33, 0xe0);
			
		tp2860_write_reg(0x35, 0x05);
		tp2860_write_reg(0x38, 0x40); 
		tp2860_write_reg(0x39, 0x68); 	
	}
	else if(FHD50 == fmt)
	
{		
		tp2860_write_reg(0x02, 0x50);
		tp2860_write_reg(0x07, 0xc0); 
		tp2860_write_reg(0x0b, 0xc0);  		
		tp2860_write_reg(0x0c, 0x03); 
		tp2860_write_reg(0x0d, 0x50);  
  		
		tp2860_write_reg(0x15, 0x03);
		tp2860_write_reg(0x16, 0xe2); 
		tp2860_write_reg(0x17, 0x80); 
		tp2860_write_reg(0x18, 0x27);
		tp2860_write_reg(0x19, 0x38);
		tp2860_write_reg(0x1a, 0x47);				
  		
		tp2860_write_reg(0x1c, 0x0a);  //
		tp2860_write_reg(0x1d, 0x4e);  //
			
		tp2860_write_reg(0x20, 0x38);  
		tp2860_write_reg(0x21, 0x84); 
		tp2860_write_reg(0x22, 0x36);
		tp2860_write_reg(0x23, 0x3c);

		tp2860_write_reg(0x27, 0xad);
		  		
		tp2860_write_reg(0x2b, 0x60);  
		tp2860_write_reg(0x2c, 0x0a); 
		tp2860_write_reg(0x2d, 0x40);
		tp2860_write_reg(0x2e, 0x70);
  		
		tp2860_write_reg(0x30, 0x74);  
		tp2860_write_reg(0x31, 0x9b); 
		tp2860_write_reg(0x32, 0xa5);
		tp2860_write_reg(0x33, 0xe0);
					
		tp2860_write_reg(0x35, 0x05);
		tp2860_write_reg(0x38, 0x40); 
		tp2860_write_reg(0x39, 0x68); 	
	}		

	tp2860_mipi_out(fmt, std, lane);


	pr_err("%s  end \n",__func__);

}




static unsigned long tp_compute_sys_clk(struct tp_dev *sensor,
					    u8 pll_prediv, u8 pll_mult,
					    u8 sysdiv)
{
	unsigned long sysclk = sensor->xclk_freq / pll_prediv * pll_mult;

	/* PLL1 output cannot exceed 1GHz. */
	if (sysclk / 1000000 > 1000)
		return 0;

	return sysclk / sysdiv;
}

static unsigned long tp_calc_sys_clk(struct tp_dev *sensor,
					 unsigned long rate,
					 u8 *pll_prediv, u8 *pll_mult,
					 u8 *sysdiv)
{
	unsigned long best = ~0;
	u8 best_sysdiv = 1, best_mult = 1;
	u8 _sysdiv, _pll_mult;

	for (_sysdiv = TP_SYSDIV_MIN;
	     _sysdiv <= TP_SYSDIV_MAX;
	     _sysdiv++) {
		for (_pll_mult = TP_PLL_MULT_MIN;
		     _pll_mult <= TP_PLL_MULT_MAX;
		     _pll_mult++) {
			unsigned long _rate;

			/*
			 * The PLL multiplier cannot be odd if above
			 * 127.
			 */
			if (_pll_mult > 127 && (_pll_mult % 2))
				continue;

			_rate = tp_compute_sys_clk(sensor,
						       TP_PLL_PREDIV,
						       _pll_mult, _sysdiv);

			/*
			 * We have reached the maximum allowed PLL1 output,
			 * increase sysdiv.
			 */
			if (!_rate)
				break;

			/*
			 * Prefer rates above the expected clock rate than
			 * below, even if that means being less precise.
			 */
			if (_rate < rate)
				continue;

			if (abs(rate - _rate) < abs(rate - best)) {
				best = _rate;
				best_sysdiv = _sysdiv;
				best_mult = _pll_mult;
			}

			if (_rate == rate)
				goto out;
		}
	}

out:
	*sysdiv = best_sysdiv;
	*pll_prediv = TP_PLL_PREDIV;
	*pll_mult = best_mult;

	return best;
}

static int tp_check_valid_mode(struct tp_dev *sensor,
				   const struct tp_mode_info *mode,
				   enum tp_frame_rate rate)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	switch (mode->id) {
	case TP_MODE:
    if(rate != TP_FPS)
		ret = -EINVAL;
		else 
		{
			pr_err("%s   %dx%d %dfps\n",__FUNCTION__,tp_mode_data[TP_MODE].hact,tp_mode_data[TP_MODE].vact, tp_framerates[TP_FPS]);
		}
		
		 
		break;
	default:
		dev_err(&client->dev, "Invalid mode (%d)\n", mode->id);
		ret = -EINVAL;
	}

	return ret;
}

static int tp_set_mipi_pclk(struct tp_dev *sensor,
				unsigned long rate)
{
	const struct tp_mode_info *mode = sensor->current_mode;
	u8 prediv, mult, sysdiv;
	u8 mipi_div;

	/*
	 * 1280x720 is reported to use 'SUBSAMPLING' only,
	 * but according to the sensor manual it goes through the
	 * scaler before subsampling.
	 */
#if 0
	if (mode->dn_mode == SCALING ||
	   (mode->id == TP_MODE_1280_960))
		mipi_div = TP_MIPI_DIV_SCLK;
	else
#endif		
		mipi_div = TP_MIPI_DIV_PCLK;

	tp_calc_sys_clk(sensor, rate, &prediv, &mult, &sysdiv);

	return 0;
}

static unsigned long tp_calc_pclk(struct tp_dev *sensor,
				      unsigned long rate,
				      u8 *pll_prediv, u8 *pll_mult, u8 *sysdiv,
				      u8 *pll_rdiv, u8 *bit_div, u8 *pclk_div)
{
	unsigned long _rate = rate * TP_PLL_ROOT_DIV * TP_BIT_DIV *
				TP_PCLK_ROOT_DIV;

	_rate = tp_calc_sys_clk(sensor, _rate, pll_prediv, pll_mult,
                                                                                                                                                                                                                                                                                                                                                                                                                        				    sysdiv);
	*pll_rdiv = TP_PLL_ROOT_DIV;
	*bit_div = TP_BIT_DIV;
	*pclk_div = TP_PCLK_ROOT_DIV;

	return _rate / *pll_rdiv / *bit_div / *pclk_div;
}

/* set JPEG framing sizes */
static int tp_set_jpeg_timings(struct tp_dev *sensor,
				   const struct tp_mode_info *mode)
{
	return 0;
}

/* download tpcam settings to sensor through i2c */
static int tp_set_timings(struct tp_dev *sensor,
			      const struct tp_mode_info *mode)
{
	return 0;
}

static int tp_load_regs(struct tp_dev *sensor,
			    const struct tp_mode_info *mode)
{
	return tp_set_timings(sensor, mode);
}

static int tp_set_autoexposure(struct tp_dev *sensor, bool on)
{
	return 0;
}

/* read exposure, in number of line periods */
static int tp_get_exposure(struct tp_dev *sensor)
{
	return 0;
}

/* write exposure, given number of line periods */
static int tp_set_exposure(struct tp_dev *sensor, u32 exposure)
{
	return 0;
}

static int tp_get_gain(struct tp_dev *sensor)
{
	return 0;
}

static int tp_set_gain(struct tp_dev *sensor, int gain)
{
	return 0;
}

static int tp_set_autogain(struct tp_dev *sensor, bool on)
{
	return 0;
}

static int tp_set_stream_mipi(struct tp_dev *sensor, bool on)
{
	return 0;
}

static int tp_get_sysclk(struct tp_dev *sensor)
{
	/* calculate sysclk */
	u32 xvclk = sensor->xclk_freq / 10000;
	u32 sysclk;

	/*
	xvclk = 2400 = (24000000/10000)
	sysclk = VCO / sysdiv / pll_rdiv * 2 / bit_div2x / sclk_rdiv;
	4250   = 68000/ 2 / 2 *2 / 4 / 2
	*/
	// 640x480
	//6150 = 98400 / 2 / 2 * 2 / 4 / 2;
	sysclk = 6150*2;

	return sysclk;
}

static int tp_set_night_mode(struct tp_dev *sensor)
{
	return 0;
}

static int tp_get_hts(struct tp_dev *sensor)
{
	u16 hts;

	hts = 1896;

	return hts;
}

static int tp_get_vts(struct tp_dev *sensor)
{
	u16 vts;

	vts = 1080;

	return vts;
}

static int tp_set_vts(struct tp_dev *sensor, int vts)
{

	return 0;
}

static int tp_get_light_freq(struct tp_dev *sensor)
{
	return 120;

}

static int tp_set_bandingfilter(struct tp_dev *sensor)
{
	u32 band_step60, max_band60, band_step50, max_band50, prev_vts;
	int ret;

	/* read preview PCLK */
	ret = tp_get_sysclk(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	//sensor->prev_sysclk : 4250
	sensor->prev_sysclk = ret;

	/* read preview HTS */
	ret = tp_get_hts(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	//sensor->prev_hts    : 1892
	sensor->prev_hts = ret;

	/* read preview VTS */
	ret = tp_get_vts(sensor);
	if (ret < 0)
		return ret;
	//prev_vts            : 740
	prev_vts = ret;

	/* calculate banding filter */
	/* 60Hz */
#if 0
	//band_step60         : 186
	band_step60 = sensor->prev_sysclk * 100 / sensor->prev_hts * 100 / 120;
	ret = tp_write_reg16(sensor, TP_REG_AEC_B60_STEP, band_step60);
	if (ret)
		return ret;
	if (!band_step60)
		return -EINVAL;

	//max_band60          : 3
	max_band60 = (int)((prev_vts - 4) / band_step60);
	ret = tp_write_reg(sensor, TP_REG_AEC_CTRL0D, max_band60);
	if (ret)
		return ret;
#endif
	/* 50Hz */
#if 0
	//band_step50         : 224
	band_step50 = sensor->prev_sysclk * 100 / sensor->prev_hts;
	ret = tp_write_reg16(sensor, TP_REG_AEC_B50_STEP, band_step50);
	if (ret)
		return ret;
	if (!band_step50)
		return -EINVAL;
	//max_band50          : 3
	max_band50 = (int)((prev_vts - 4) / band_step50);
#endif

	return 0;
}

static int tp_set_ae_target(struct tp_dev *sensor, int target)
{
	/* target=52
	 * sensor->ae_low  : 47
	 * sensor->ae_high : 56
	 * fast_low        : 23
	 * fast_high       : 112
	*/
	sensor->ae_low  = target * 23 / 25;	/* 0.92 */
	sensor->ae_high = target * 27 / 25;	/* 1.08 */

	return 0;
}

static const struct tp_mode_info *
tp_find_mode(struct tp_dev *sensor, enum tp_frame_rate fr,
		 int width, int height, bool nearest)
{
	const struct tp_mode_info *mode;




//	pr_err("%s width %d height %d \n", __func__, width,height);
 //  pr_err("%s MDIN %d FPS \n", __func__,tp_framerates[fr]);



	mode = v4l2_find_nearest_size(tp_mode_data,
				      ARRAY_SIZE(tp_mode_data),
				      hact, vact,
				      width, height);

	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height)))
		return NULL;

	return mode;
}

/*
 * sensor changes between scaling and subsampling, go through
 * exposure calculation
 */
static int tp_set_mode_exposure_calc(struct tp_dev *sensor,
					 const struct tp_mode_info *mode)
{
	u32 prev_shutter, prev_gain16;
	u32 cap_shutter, cap_gain16;
	u32 cap_sysclk, cap_hts, cap_vts;
	u32 light_freq, cap_bandfilt, cap_maxband;
	u32 cap_gain16_shutter;
	u8 average;
	int ret;

	/* read preview shutter */
	ret = tp_get_exposure(sensor);
	if (ret < 0)
		return ret;
	prev_shutter = ret;

	/* read preview gain */
	ret = tp_get_gain(sensor);
	if (ret < 0)
		return ret;
	prev_gain16 = ret;

	/* turn off night mode for capture */
	ret = tp_set_night_mode(sensor);
	if (ret < 0)
		return ret;

	/* Write capture setting */
	ret = tp_load_regs(sensor, mode);
	if (ret < 0)
		return ret;

	/* read capture VTS */
	ret = tp_get_vts(sensor);
	if (ret < 0)
		return ret;
	cap_vts = ret;
	ret = tp_get_hts(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	cap_hts = ret;

	ret = tp_get_sysclk(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	cap_sysclk = ret;

	/* calculate capture banding filter */
	ret = tp_get_light_freq(sensor);
	if (ret < 0)
		return ret;
	light_freq = ret;

	if (light_freq == 60) {
		/* 60Hz */
		cap_bandfilt = cap_sysclk * 100 / cap_hts * 100 / 120;
	} else {
		/* 50Hz */
		cap_bandfilt = cap_sysclk * 100 / cap_hts;
	}

	if (!sensor->prev_sysclk) {
		ret = tp_get_sysclk(sensor);
		if (ret < 0)
			return ret;
		if (ret == 0)
			return -EINVAL;
		sensor->prev_sysclk = ret;
	}

	if (!cap_bandfilt)
		return -EINVAL;

	cap_maxband = (int)((cap_vts - 4) / cap_bandfilt);

	/* calculate capture shutter/gain16 */
	if (average > sensor->ae_low && average < sensor->ae_high) {
		/* in stable range */
		cap_gain16_shutter =
			prev_gain16 * prev_shutter *
			cap_sysclk / sensor->prev_sysclk *
			sensor->prev_hts / cap_hts *
			sensor->ae_target / average;
	} else {
		cap_gain16_shutter =
			prev_gain16 * prev_shutter *
			cap_sysclk / sensor->prev_sysclk *
			sensor->prev_hts / cap_hts;
	}

	/* gain to shutter */
	if (cap_gain16_shutter < (cap_bandfilt * 16)) {
		/* shutter < 1/100 */
		cap_shutter = cap_gain16_shutter / 16;
		if (cap_shutter < 1)
			cap_shutter = 1;

		cap_gain16 = cap_gain16_shutter / cap_shutter;
		if (cap_gain16 < 16)
			cap_gain16 = 16;
	} else {
		if (cap_gain16_shutter > (cap_bandfilt * cap_maxband * 16)) {
			/* exposure reach max */
			cap_shutter = cap_bandfilt * cap_maxband;
			if (!cap_shutter)
				return -EINVAL;

			cap_gain16 = cap_gain16_shutter / cap_shutter;
		} else {
			/* 1/100 < (cap_shutter = n/100) =< max */
			cap_shutter =
				((int)(cap_gain16_shutter / 16 / cap_bandfilt))
				* cap_bandfilt;
			if (!cap_shutter)
				return -EINVAL;

			cap_gain16 = cap_gain16_shutter / cap_shutter;
		}
	}

	/* set capture gain */
	ret = tp_set_gain(sensor, cap_gain16);
	if (ret)
		return ret;

	/* write capture shutter */
	if (cap_shutter > (cap_vts - 4)) {
		cap_vts = cap_shutter + 4;
		ret = tp_set_vts(sensor, cap_vts);
		if (ret < 0)
			return ret;
	}

	/* set exposure */
	return tp_set_exposure(sensor, cap_shutter);
}

/*
 * if sensor changes inside scaling or subsampling
 * change mode directly
 */
static int tp_set_mode_direct(struct tp_dev *sensor,
				  const struct tp_mode_info *mode)
{
	/* Write capture setting */
	return tp_load_regs(sensor, mode);
}

static int tp_set_mode(struct tp_dev *sensor)
{
	const struct tp_mode_info *mode = sensor->current_mode;
	const struct tp_mode_info *orig_mode = sensor->last_mode;
	enum tp_downsize_mode dn_mode, orig_dn_mode;
	unsigned long rate;
	int ret;

	dn_mode = mode->dn_mode;
	orig_dn_mode = orig_mode->dn_mode;

	/*
	 * All the formats we support have 16 bits per pixel, seems to require
	 * the same rate than YUV, so we can just use 16 bpp all the time.
	 */
	rate = mode->vtot * mode->htot * 16;
	rate *= tp_framerates[sensor->current_fr];

	if (sensor->ep.bus_type == V4L2_MBUS_CSI2_DPHY) {
		rate = rate / sensor->ep.bus.mipi_csi2.num_data_lanes;
		ret = tp_set_mipi_pclk(sensor, rate);
	}

	ret = tp_set_bandingfilter(sensor);
	if (ret < 0)
		return ret;

	sensor->pending_mode_change = false;
	sensor->last_mode = mode;

	return 0;
}

static int tp_set_framefmt(struct tp_dev *sensor,
			       struct v4l2_mbus_framefmt *format);

/* restore the last set video mode after chip power-on */
static int tp_restore_mode(struct tp_dev *sensor)
{
	int ret;

//s.y.s																																																																																																		
   return 0;
   
   
	/* now restore the last capture mode */
	ret = tp_set_mode(sensor);
	if (ret < 0)
		return ret;

	return tp_set_framefmt(sensor, &sensor->fmt);
}

static void tp_power(struct tp_dev *sensor, bool enable)
{




if(enable == 0)
	return;

 #if  APPL_USE_PWR 	

	pr_err("%s ==> %d  \n", __func__, enable);

   
 // gpio_set_value(sensor->pwr_gpio, enable ? 1 : 0);

	usleep_range(20000, 25000);
#endif
}

static void tp_reset(struct tp_dev *sensor)
{
	
	

	pr_err("%s   \n", __func__);

	
if (!sensor->reset_gpio)
		return;

//	gpio_set_value(sensor->reset_gpio,  1);
//	usleep_range(1000, 2000);

//	gpio_set_value(sensor->reset_gpio,  0);
//	usleep_range(20000, 25000);

//	gpio_set_value(sensor->reset_gpio,  1);
//	usleep_range(1000, 2000);
}

static int tp_set_power_on(struct tp_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;


  return 0;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

#if 0
	ret = regulator_bulk_enable(tp_NUM_SUPPLIES,
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto xclk_off;
	}
#endif


	tp_power(sensor, true);
	tp_reset(sensor);
	tp_power(sensor, true);

	return 0;

power_off:
	
	return 0;
	
	//tp_power(sensor, false);
	//regulator_bulk_disable(tp_NUM_SUPPLIES, sensor->supplies);
xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void tp_set_power_off(struct tp_dev *sensor)
{


return ;
	tp_power(sensor, false);
	//regulator_bulk_disable(tp_NUM_SUPPLIES, sensor->supplies);
	clk_disable_unprepare(sensor->xclk);
	sensor->streaming = false;
}

static int tp_set_power(struct tp_dev *sensor, bool on)
{
	int ret = 0;




//    tp2860_dum_reg();

    return 0;



	if (on) {
		ret = tp_set_power_on(sensor);
		if (ret)
			return ret;

		ret = tp_restore_mode(sensor);
		if (ret)
			goto power_off;

		/* We're done here for DVP bus, while CSI-2 needs setup. */
		if (sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY)
			return 0;

		usleep_range(500, 1000);
	} else {
		tp_set_power_off(sensor);
	}

	return 0;

power_off:
	tp_set_power_off(sensor);
	return ret;
}

/* --------------- Subdev Operations --------------- */

static int tp_s_power(struct v4l2_subdev *sd, int on)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	int ret = 0;

   pr_err("%s ==> \n", __func__);

   return 0;

  


	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		ret = tp_set_power(sensor, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);

	if (on && !ret && sensor->power_count == 1) {
		/* restore controls */
		ret = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	}

	return ret;
}

static int tp_try_frame_interval(struct tp_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct tp_mode_info *mode;
	enum tp_frame_rate rate = TP_FPS;//TP_08_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;


  rate = TP_FPS;
	minfps = tp_framerates[TP_FPS];
	maxfps = tp_framerates[TP_FPS];

#if 0
	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = TP_FPS;
		goto find_mode;
	}

	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	for (i = 0; i < ARRAY_SIZE(tp_framerates); i++) {
		int curr_fps = tp_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}

	fi->numerator = 1;
	fi->denominator = best_fps;
#endif


find_mode:
	mode = tp_find_mode(sensor, rate, width, height, false);
	return mode ? rate : -EINVAL;
}

static int tp_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	fmt->reserved[1] = (sensor->current_fr == TP_FPS) ? 120 : 120;
	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

static int tp_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum tp_frame_rate fr,
				   const struct tp_mode_info **new_mode)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	const struct tp_mode_info *mode;
	int i;

	mode = tp_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->hact;
	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(tp_formats); i++)
		if (tp_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(tp_formats))
		i = 0;

	fmt->code = tp_formats[i].code;
	fmt->colorspace = tp_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	//s.y.s
	//	fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
	//fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	/* The top field is always transferred first by the chip */
	fmt->field = V4L2_FIELD_INTERLACED_TB; 

	return 0;
}

static int tp_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	const struct tp_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = tp_try_fmt_internal(sd, mbus_fmt,
				      sensor->current_fr, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	else
		fmt = &sensor->fmt;

	*fmt = *mbus_fmt;

	if (new_mode != sensor->current_mode) {
		sensor->current_mode = new_mode;
		sensor->pending_mode_change = true;
	}

	if (mbus_fmt->code != sensor->fmt.code)
		sensor->pending_fmt_change = true;

	if (sensor->pending_mode_change || sensor->pending_fmt_change)
		sensor->fmt = *mbus_fmt;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int tp_set_framefmt(struct tp_dev *sensor,
			       struct v4l2_mbus_framefmt *format)
{
	return 0;
}

/*
 * Sensor Controls.
 */

static int tp_set_ctrl_hue(struct tp_dev *sensor, int value)
{
	return 0;
}

static int tp_set_ctrl_contrast(struct tp_dev *sensor, int value)
{
	return 0;
}

static int tp_set_ctrl_saturation(struct tp_dev *sensor, int value)
{
	return 0;
}

static int tp_set_ctrl_white_balance(struct tp_dev *sensor, int awb)
{
	return 0;
}

static int tp_set_ctrl_exposure(struct tp_dev *sensor,
				    enum v4l2_exposure_auto_type auto_exposure)
{
	return 0;
}

static int tp_set_ctrl_gain(struct tp_dev *sensor, bool auto_gain)
{
	return 0;
}

static int tp_set_ctrl_light_freq(struct tp_dev *sensor, int value)
{
	return 0;
}

static int tp_set_ctrl_hflip(struct tp_dev *sensor, int value)
{
	return 0;
}

static int tp_set_ctrl_vflip(struct tp_dev *sensor, int value)
{
	return 0;
}

static int tp_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct tp_dev *sensor = to_tp_dev(sd);
	int val;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		val = tp_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		val = tp_get_exposure(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.exposure->val = val;
		break;
	}

	return 0;
}

static int tp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct tp_dev *sensor = to_tp_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (sensor->power_count == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = tp_set_ctrl_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = tp_set_ctrl_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = tp_set_ctrl_white_balance(sensor, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = tp_set_ctrl_hue(sensor, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = tp_set_ctrl_contrast(sensor, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = tp_set_ctrl_saturation(sensor, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = tp_set_ctrl_light_freq(sensor, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = tp_set_ctrl_hflip(sensor, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = tp_set_ctrl_vflip(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops tp_ctrl_ops = {
	.g_volatile_ctrl = tp_g_volatile_ctrl,
	.s_ctrl = tp_s_ctrl,
};

static int tp_init_controls(struct tp_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &tp_ctrl_ops;
	struct tp_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Auto/manual white balance */
	ctrls->auto_wb = v4l2_ctrl_new_std(hdl, ops,
					   V4L2_CID_AUTO_WHITE_BALANCE,
					   0, 1, 1, 1);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4095, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4095, 1, 0);
	/* Auto/manual exposure */
	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
						 V4L2_CID_EXPOSURE_AUTO,
						 V4L2_EXPOSURE_MANUAL, 0,
						 V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN,
					     0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 1023, 1, 0);

	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 255, 1, 64);
	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE,
				       0, 359, 1, 0);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 255, 1, 0);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					 0, 1, 1, 0);

	ctrls->light_freq =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_POWER_LINE_FREQUENCY,
				       V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
				       V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, false);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, true);

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
} 

static int tp_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= TP_NUM_MODES)
		return -EINVAL;

	fse->min_width  = tp_mode_data[fse->index].hact;
	fse->max_width  = fse->min_width;
	fse->min_height = tp_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	return 0;
}

static int tp_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	int i, j, count;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= TP_NUM_FRAMERATES)
		return -EINVAL;


pr_err("%s fie->index =%d fie->width=%d fie->height=%d ",__func__,fie->index,fie->width,fie->height );

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

  
    

	count = 0;
	for (i = 0; i < TP_NUM_FRAMERATES; i++) {
		for (j = 0; j < TP_NUM_MODES; j++) {
			if (fie->width  == tp_mode_data[j].hact &&
			    fie->height == tp_mode_data[j].vact &&
			    !tp_check_valid_mode(sensor, &tp_mode_data[j], i))
				count++;

			if (fie->index == (count - 1)) {
				fie->interval.denominator = tp_framerates[i];
          //   pr_err("%s fie->index =%d  i=%d", __func__,fie->index,i );

				return 0;
			}
		}
	}
 // pr_err("%s fie->index =%d  i=%d  error ", __func__,fie->index,i );

	return -EINVAL;
}

static int tp_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct tp_dev *sensor = to_tp_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int tp_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	const struct tp_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = tp_try_frame_interval(sensor, &fi->interval,
					       mode->hact, mode->vact);
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}

	mode = tp_find_mode(sensor, frame_rate, mode->hact,
				mode->vact, true);
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}

	if (mode != sensor->current_mode ||
	    frame_rate != sensor->current_fr) {
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
		sensor->pending_mode_change = true;
	}

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int tp_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(tp_formats))
		return -EINVAL;

	code->code = tp_formats[code->index].code;
	return 0;
}

static int tp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tp_dev *sensor = to_tp_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !enable) {
		ret = tp_check_valid_mode(sensor,
					      sensor->current_mode,
					      sensor->current_fr);
		if (ret) {
			dev_err(&client->dev, "Not support WxH@fps=%dx%d@%d\n",
				sensor->current_mode->hact,
				sensor->current_mode->vact,
				tp_framerates[sensor->current_fr]);
			goto out;
		}

		if (enable && sensor->pending_mode_change) {
			ret = tp_set_mode(sensor);
			if (ret)
				goto out;
		}

		if (enable && sensor->pending_fmt_change) {
			ret = tp_set_framefmt(sensor, &sensor->fmt);
			if (ret)
				goto out;
			sensor->pending_fmt_change = false;
		}

		sensor->ep.bus_type = V4L2_MBUS_CSI2_DPHY;
		tp_set_stream_mipi(sensor, enable);
		sensor->streaming = enable;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops tp_core_ops = {
	.s_power           = tp_s_power,
	.log_status        = v4l2_ctrl_subdev_log_status,
	.subscribe_event   = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tp_video_ops = {
	.g_frame_interval  = tp_g_frame_interval,
	.s_frame_interval  = tp_s_frame_interval,
	.s_stream          = tp_s_stream,
};

static const struct v4l2_subdev_pad_ops tp_pad_ops = {
	.enum_mbus_code      = tp_enum_mbus_code,
	.get_fmt             = tp_get_fmt,
	.set_fmt             = tp_set_fmt,
	.enum_frame_size     = tp_enum_frame_size,
	.enum_frame_interval = tp_enum_frame_interval,
};

static const struct v4l2_subdev_ops tp_subdev_ops = {
	.core  = &tp_core_ops,
	.video = &tp_video_ops,
	.pad   = &tp_pad_ops,
};

static int tp_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations tp_sd_media_ops = {
	.link_setup = tp_link_setup,
};

#if 0
static int tp_get_regulators(struct tp_dev *sensor)
{
	int i;

	for (i = 0; i < TP_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = tp_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       TP_NUM_SUPPLIES,
				       sensor->supplies);
}
#endif





static ssize_t value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t			status;
	
	status = 0;
#if  APPL_USE_PWR 	



        tp2860_write_reg(0x40, 0x00); //back to decoder page
        
        
//s.y.s
//  	    pr_err(" reg 0x%02x   =>   value 0x%02x\n",buf[0], tp2860_read_reg(buf[0]));
//		       pr_err(" %s reg 0x%02x   =>   value 0x%02x\n", __func__,buf[0], tp2860_read_reg(buf[0]));
//		       pr_err(" %s reg 0x%02x   =>   value 0x%02x\n", __func__,buf[1], tp2860_read_reg(buf[1]));
//	status = gpio_get_value(tp_g_pwr_gpio);

//	if (status < 0)
//		goto err;
//	buf[0] = '0' + status;
//	buf[1] = '\n';


#endif
	status = 2;

err:
	return status;
} 
static ssize_t value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	int i;

//s.y.s
//		dev_err(dev, " %s   =========================>\n", __func__);
 #if  APPL_USE_PWR 	
   

   // dev_err(dev, "  %d \n", gpio_get_value(tp_g_pwr_gpio)); 
/*
   for(i=0; i<size; i++)
      pr_err("  %d   0x%02x", buf[i], buf[i]);
    pr_err("\n");
*/

    if(buf[0] == 0)
    {	
	//         gpio_set_value(tp_g_pwr_gpio, 0);
		       pr_err(" %s MIPI off   \n", __func__);
		       
		       	tp2860_write_reg(0x40, 0x08); //select MIPI page
	          /* Disable MIPI CSI2 output */
	          tp2860_write_reg(0x28, 0x02);	 //stream off 
	          tp2860_write_reg(0x40, 0x00); //back to decoder page

		       
		 }      
    else if(buf[0] == 1)
    {	
//	         gpio_set_value(tp_g_pwr_gpio, 1);
		       pr_err(" %s MIPI on   \n", __func__);
	         
	         
	         /* Enable MIPI CSI2 output */
	         tp2860_write_reg(0x40, 0x08); //select MIPI page
	         tp2860_write_reg(0x28, 0x00);	 //stream on
	         tp2860_write_reg(0x40, 0x00); //back to decoder page

		 }  
		 else if(buf[0] == 2)
	   {


// 		         tp2860_dum_reg();
#if 1          
         pr_err(" %s reset    \n", __func__);

 		        	gpio_set_value(tp_g_reset_gpio,  1);
	            usleep_range(1000, 2000);

	            gpio_set_value(tp_g_reset_gpio,  0);
	            usleep_range(20000, 25000);

	            gpio_set_value(tp_g_reset_gpio,  1);
	            usleep_range(1000, 2000);
#endif 
 
	   }	 
	    else if(buf[0] == 3) // on power // pattern display
	   {
         u8 tmp;
	 
         pr_err(" %s power on    \n", __func__);
 	       gpio_set_value(tp_g_pwr_gpio, 1);  	
#if 0
           pr_err(" %s pattern on   \n", __func__);
	         
	    		 tp2860_write_reg(0x40, 0x08); //select MIPI page
	         tp2860_write_reg(0x28, 0x02);	 //stream off 
	   	   	 tmp = tp2860_read_reg(0x22);
	         tp2860_write_reg(0x22, tmp|0x80);
	         tp2860_write_reg(0x28, 0x00);	 //stream on
	         tp2860_write_reg(0x40, 0x00); //back to decoder page
#endif
	   }
	    else if(buf[0] == 4) // off power // pattern display
	   {
	    		 u8 tmp;

         pr_err(" %s power off    \n", __func__);

	       gpio_set_value(tp_g_pwr_gpio, 0);  	

#if 0	    		 
				 pr_err(" %s pattern off   \n", __func__);
	    		 tp2860_write_reg(0x40, 0x08); //select MIPI page
	         tp2860_write_reg(0x28, 0x02);	 //stream off 
	   	   	 tmp = tp2860_read_reg(0x22);
	   	   	 tmp &= 0x7f;
	         tp2860_write_reg(0x22, tmp);
	         tp2860_write_reg(0x28, 0x00);	 //stream on
	         tp2860_write_reg(0x40, 0x00); //back to decoder page
#endif
	   }
	    else if(buf[0] == 5)
	   	{
	   		  mipi_stream_off();
	   		  if( buf[1] == 0)
	        {    		   
	   		    tp2860_write_reg(0x40, 0x00); //back to decoder page
	   		    tp2860_write_reg(buf[2], buf[3]);
		        pr_err(" decoder reg 0x%02x   =>   value 0x%02x\n", buf[2], tp2860_read_reg(buf[2]));
         	 }
         	 else 
         	 {
 	    		  tp2860_write_reg(0x40, 0x08); //select MIPI page
	   		    tp2860_write_reg(buf[2], buf[3]);
		        pr_err("mipi reg 0x%02x   =>   value 0x%02x\n",buf[2], tp2860_read_reg(buf[2]));
	   		    tp2860_write_reg(0x40, 0x00); //back to decoder page
         	 	
         	 	
         	 }  		    
	   		
	   	}
	    else if(buf[0] == 6)
	   	{
	   		  mipi_stream_off();
	   		  if( buf[1] == 0)
	        {    		   
	   		    tp2860_write_reg(0x40, 0x00); //back to decoder page
		        pr_err(" decoder reg 0x%02x   =>   value 0x%02x\n",buf[2], tp2860_read_reg(buf[2]));
	   		  }  
	   		  else 
	   		  {
 	    		  tp2860_write_reg(0x40, 0x08); //select MIPI page
		        pr_err(" mipi reg 0x%02x   =>   value 0x%02x\n",buf[2], tp2860_read_reg(buf[2]));
	   		    tp2860_write_reg(0x40, 0x00); //back to decoder page
	   		  	
	   		  }
	   		
	   	}
	   
	   
#endif
   status = 1;
	return status;
} 

//static DEVICE_ATTR_PREALLOC(value, S_IWUSR | S_IRUGO, value_show, value_store); 

static struct device_attribute dev_attr_value = \
	__ATTR(value, 0660, value_show, value_store);
	
 

static struct attribute *gpio_attrs[] = {
	&dev_attr_value.attr,
	NULL,
};
 
static const struct attribute_group gpio_group = {
	.attrs = gpio_attrs,
};



static int tp_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct tp_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	u32 rotation;
	int ret;
  int pwr_enable_gpio;
  int reset_gpio;
	struct device_node *np = dev->of_node;

  
   


  pr_err("%s module \n", __func__);

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

  tp_g_sensor = sensor;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

#if m_1920
	fmt->width = 1920;
	fmt->height = 1080;
#else
	fmt->width = 1280;
	fmt->height = 720;
#endif
  

	fmt->field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = tp_framerates[TP_FPS];
	sensor->current_fr = TP_FPS;

	sensor->current_mode =
		&tp_mode_data[TP_MODE];

	sensor->ae_target = 52;

	/* optional indication of physical rotation of sensor */
	ret = fwnode_property_read_u32(dev_fwnode(&client->dev), "rotation",
				       &rotation);
	if (!ret) {
		switch (rotation) {
		case 180:
			sensor->upside_down = true;
			/* fall through */
		case 0:
			break;
		default:
			dev_warn(dev, "%u degrees rotation is not supported, ignoring...\n",
				 rotation);
		}
	}

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	
	dev_err(dev, "  sensor->xclk_freq = 0x%08x \n",sensor->xclk_freq);
 //	sensor->xclk_freq = 108000000;	
	
	if (sensor->xclk_freq < TP_XCLK_MIN ||
	    sensor->xclk_freq > TP_XCLK_MAX) {
		dev_err(dev, "xclk frequency out of range: %d Hz\n",
			sensor->xclk_freq);
		return -EINVAL;
	}


  pwr_enable_gpio= of_get_named_gpio(np, "pwr-enable-gpio", 0);

	if (gpio_is_valid(pwr_enable_gpio)) {
		ret = gpio_request_one(pwr_enable_gpio, GPIOF_OUT_INIT_LOW, "camera-pwr-enable-gpio");
		if (ret) {
			dev_err(dev, "[CAM] %s: failed to request pwr-enable-gpio [%d]\n", __func__, pwr_enable_gpio);
		} else {
			gpio_set_value(pwr_enable_gpio, 1);
			dev_info(dev, "[CAM] %s: camera power enabled\n", __func__);
			gpio_free(pwr_enable_gpio);
		}
	} else {
		dev_err(dev, "[CAM] %s: failed to get pwr-enable-gpio\n", __func__);
		return -EINVAL;
	}


  tp_g_pwr_gpio = pwr_enable_gpio;
  sensor->pwr_gpio = pwr_enable_gpio;



  reset_gpio= of_get_named_gpio(np, "reset-gpio", 0);

	if (gpio_is_valid(pwr_enable_gpio)) {
		ret = gpio_request_one(reset_gpio, GPIOF_OUT_INIT_LOW, "camera-reset-gpio");
		if (ret) {
			dev_err(dev, "[CAM] %s: failed to request reset-gpio [%d]\n", __func__, reset_gpio);
		} else {
			gpio_set_value(reset_gpio, 1);
			dev_info(dev, "[CAM] %s: camera reset enabled\n", __func__);
			gpio_free(reset_gpio);
		}
	} else {
		dev_err(dev, "[CAM] %s: failed to get reset-gpio\n", __func__);
		return -EINVAL;
	}


  tp_g_reset_gpio = reset_gpio;
  sensor->reset_gpio = reset_gpio;


  
	v4l2_i2c_subdev_init(&sensor->sd, client, &tp_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &tp_sd_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

#if 0
	ret = tp_get_regulators(sensor);
	if (ret)
		return ret;
#endif

	mutex_init(&sensor->lock);

	ret = tp_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret)
		goto free_ctrls;

	dev_info(dev, "tpcam : i8452 probe success !\n");

#if APPL_USE_PWR	

	ret = sysfs_create_group(&client->dev.kobj, &gpio_group);
	if(ret < 0) {
		dev_err(dev, "failed to create sysfs files\n");
	}
#endif



//	tp_power(sensor, true);
//	tp_reset(sensor);
	//tp_power(sensor, true);
	gpio_set_value(sensor->reset_gpio,  1);
	usleep_range(1000, 2000);

	gpio_set_value(sensor->reset_gpio,  0);
	usleep_range(20000, 25000);

	gpio_set_value(sensor->reset_gpio,  1);
	usleep_range(1000, 2000);

//  ret = tp_set_power(sensor, true);
 

// tp2860_sensor_init(VIN1,HD30,STD_TVI,MIPI_2LANE);
 tp2860_sensor_init(VIN1,HD60,STD_TVI,MIPI_2LANE);
//tp2860_sensor_init(VIN1,HD60,STD_HDA,MIPI_2LANE);
//tp2860_sensor_init(VIN1,FHD50,STD_TVI,MIPI_2LANE);
 //   tp2860_dum_reg();

 pr_err("%s  OK \n", __func__);

 

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int tp_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tp_dev *sensor = to_tp_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);

	return 0;
}
#if 1
static const struct i2c_device_id tp_id[] = {
	{"tpcam", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, tp_id);
#endif

static const struct of_device_id tp_dt_ids[] = {
	{ .compatible = "tpcam,i8452" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tp_dt_ids);
 
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		.name  = "tpcam",
		.of_match_table	= tp_dt_ids,
	},  
	.id_table  = tp_id,
	.probe_new = tp_probe,
	.remove    = tp_remove,
};
#if 0
module_i2c_driver(tp_i2c_driver);
#endif


static int __init module_begin(void){
//  int ret;
	printk("tp2860, linux kernel module. \n");

//  ret = tp_probe(tp_client);
  return i2c_add_driver(&tp_i2c_driver);
//    return 0;
}
 
static void __exit module_end(void){
//	int ret;
	printk("tp2860 Bye!\n");
  i2c_del_driver(&tp_i2c_driver);
  return ;
	
}
 
module_init(module_begin);
module_exit(module_end);



MODULE_DESCRIPTION("tpcam MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
   