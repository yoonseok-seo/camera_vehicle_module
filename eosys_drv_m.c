
 
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
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
#define EOSYS_XCLK_MIN  6000000
#define EOSYS_XCLK_MAX 54000000

#define CASE_640_480
//#define CASE_1280_960


#define APPL_USE_PWR  1

enum eocam_mode_id {
	EOSYS_MODE_640_480 = 0,
//	EOSYS_MODE_1280_960,
	EOSYS_NUM_MODES,
};

enum eocam_frame_rate {
//	EOSYS_30_FPS = 0,
	EOSYS_60_FPS =0,
//	EOSYS_120_FPS,
	EOSYS_NUM_FRAMERATES,
};

enum eocam_format_mux {
	EOSYS_FMT_MUX_YUV422 = 0,
	EOSYS_FMT_MUX_RGB,
	EOSYS_FMT_MUX_DITHER,
	EOSYS_FMT_MUX_RAW_DPC,
	EOSYS_FMT_MUX_SNR_RAW,
	EOSYS_FMT_MUX_RAW_CIP,
};

struct eocam_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct eocam_pixfmt eocam_formats[] = {
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

static const int eocam_framerates[] = {
//	[EOSYS_30_FPS] = 30,
	[EOSYS_60_FPS] = 60,
//	[EOSYS_120_FPS] = 120,
};

/* regulator supplies */
static const char * const eocam_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};

#define EOSYS_NUM_SUPPLIES ARRAY_SIZE(eocam_supply_name)

/*
 * Image size under 1280 * 960 are SUBSAMPLING
 * Image size upper 1280 * 960 are SCALING
 */
enum eocam_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct eocam_mode_info {
	enum eocam_mode_id id;
	enum eocam_downsize_mode dn_mode;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
};

struct eocam_ctrls {
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

struct eocam_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to eocam */
	u32 xclk_freq;

	struct regulator_bulk_data supplies[EOSYS_NUM_SUPPLIES];

#if  APPL_USE_PWR 	
	int pwr_gpio;
#endif
	bool   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;

	const struct eocam_mode_info *current_mode;
	const struct eocam_mode_info *last_mode;
	enum eocam_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct eocam_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	bool pending_mode_change;
	bool streaming;
};

#if APPL_USE_PWR
	int eosys_g_pwr_gpio;
#endif
static const struct eocam_mode_info
eocam_mode_data[EOSYS_NUM_MODES] = {
	{EOSYS_MODE_640_480, SUBSAMPLING,
	 640, 1896, 480, 1080,
	},
//	{EOSYS_MODE_1280_960, SUBSAMPLING,
//	 1280, 1892, 960, 1080,
//	},
};

static inline struct eocam_dev *to_eocam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct eocam_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct eocam_dev,
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
#define EOSYS_PLL_PREDIV	3

#define EOSYS_PLL_MULT_MIN	4
#define EOSYS_PLL_MULT_MAX	252

/*
 * This is supposed to be ranging from 1 to 16, but the value is
 * always set to either 1 or 2 in the vendor kernels.
 */
#define EOSYS_SYSDIV_MIN	1
#define EOSYS_SYSDIV_MAX	16

/*
 * Hardcode these values for scaler and non-scaler modes.
 * FIXME: to be re-calcualted for 1 data lanes setups
 */
#define EOSYS_MIPI_DIV_PCLK	2
#define EOSYS_MIPI_DIV_SCLK	1

/*
 * This is supposed to be ranging from 1 to 2, but the value is always
 * set to 2 in the vendor kernels.
 */
#define EOSYS_PLL_ROOT_DIV			2
#define EOSYS_PLL_CTRL3_PLL_ROOT_DIV_2		BIT(4)

/*
 * We only supports 8-bit formats at the moment
 */
#define EOSYS_BIT_DIV				2
#define EOSYS_PLL_CTRL0_MIPI_MODE_8BIT		0x08

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 2 in the vendor kernels.
 */
#define EOSYS_SCLK_ROOT_DIV	2

/*
 * This is hardcoded so that the consistency is maintained between SCLK and
 * SCLK 2x.
 */
#define EOSYS_SCLK2X_ROOT_DIV (EOSYS_SCLK_ROOT_DIV / 2)

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 1 in the vendor kernels.
 */
#define EOSYS_PCLK_ROOT_DIV			1
#define EOSYS_PLL_SYS_ROOT_DIVIDER_BYPASS	0x00

static unsigned long eocam_compute_sys_clk(struct eocam_dev *sensor,
					    u8 pll_prediv, u8 pll_mult,
					    u8 sysdiv)
{
	unsigned long sysclk = sensor->xclk_freq / pll_prediv * pll_mult;

	/* PLL1 output cannot exceed 1GHz. */
	if (sysclk / 1000000 > 1000)
		return 0;

	return sysclk / sysdiv;
}

static unsigned long eocam_calc_sys_clk(struct eocam_dev *sensor,
					 unsigned long rate,
					 u8 *pll_prediv, u8 *pll_mult,
					 u8 *sysdiv)
{
	unsigned long best = ~0;
	u8 best_sysdiv = 1, best_mult = 1;
	u8 _sysdiv, _pll_mult;

	for (_sysdiv = EOSYS_SYSDIV_MIN;
	     _sysdiv <= EOSYS_SYSDIV_MAX;
	     _sysdiv++) {
		for (_pll_mult = EOSYS_PLL_MULT_MIN;
		     _pll_mult <= EOSYS_PLL_MULT_MAX;
		     _pll_mult++) {
			unsigned long _rate;

			/*
			 * The PLL multiplier cannot be odd if above
			 * 127.
			 */
			if (_pll_mult > 127 && (_pll_mult % 2))
				continue;

			_rate = eocam_compute_sys_clk(sensor,
						       EOSYS_PLL_PREDIV,
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
	*pll_prediv = EOSYS_PLL_PREDIV;
	*pll_mult = best_mult;

	return best;
}

static int eocam_check_valid_mode(struct eocam_dev *sensor,
				   const struct eocam_mode_info *mode,
				   enum eocam_frame_rate rate)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	switch (mode->id) {
	case EOSYS_MODE_640_480:
    if(rate != EOSYS_60_FPS)
		ret = -EINVAL;
		else 
		{
			pr_err("%s    640x480 60fps\n",__FUNCTION__);
		}
		
		 
		break;
//	case EOSYS_MODE_1280_960:
//		if (sensor->ep.bus_type == V4L2_MBUS_CSI2_DPHY) {
 //   if(rate != EOSYS_60_FPS)
//				ret = -EINVAL;
//		}
//		break;
	default:
		dev_err(&client->dev, "Invalid mode (%d)\n", mode->id);
		ret = -EINVAL;
	}

	return ret;
}

static int eocam_set_mipi_pclk(struct eocam_dev *sensor,
				unsigned long rate)
{
	const struct eocam_mode_info *mode = sensor->current_mode;
	u8 prediv, mult, sysdiv;
	u8 mipi_div;

	/*
	 * 1280x720 is reported to use 'SUBSAMPLING' only,
	 * but according to the sensor manual it goes through the
	 * scaler before subsampling.
	 */
#if 0
	if (mode->dn_mode == SCALING ||
	   (mode->id == EOSYS_MODE_1280_960))
		mipi_div = EOSYS_MIPI_DIV_SCLK;
	else
#endif		
		mipi_div = EOSYS_MIPI_DIV_PCLK;

	eocam_calc_sys_clk(sensor, rate, &prediv, &mult, &sysdiv);

	return 0;
}

static unsigned long eocam_calc_pclk(struct eocam_dev *sensor,
				      unsigned long rate,
				      u8 *pll_prediv, u8 *pll_mult, u8 *sysdiv,
				      u8 *pll_rdiv, u8 *bit_div, u8 *pclk_div)
{
	unsigned long _rate = rate * EOSYS_PLL_ROOT_DIV * EOSYS_BIT_DIV *
				EOSYS_PCLK_ROOT_DIV;

	_rate = eocam_calc_sys_clk(sensor, _rate, pll_prediv, pll_mult,
				    sysdiv);
	*pll_rdiv = EOSYS_PLL_ROOT_DIV;
	*bit_div = EOSYS_BIT_DIV;
	*pclk_div = EOSYS_PCLK_ROOT_DIV;

	return _rate / *pll_rdiv / *bit_div / *pclk_div;
}

/* set JPEG framing sizes */
static int eocam_set_jpeg_timings(struct eocam_dev *sensor,
				   const struct eocam_mode_info *mode)
{
	return 0;
}

/* download eocam settings to sensor through i2c */
static int eocam_set_timings(struct eocam_dev *sensor,
			      const struct eocam_mode_info *mode)
{
	return 0;
}

static int eocam_load_regs(struct eocam_dev *sensor,
			    const struct eocam_mode_info *mode)
{
	return eocam_set_timings(sensor, mode);
}

static int eocam_set_autoexposure(struct eocam_dev *sensor, bool on)
{
	return 0;
}

/* read exposure, in number of line periods */
static int eocam_get_exposure(struct eocam_dev *sensor)
{
	return 0;
}

/* write exposure, given number of line periods */
static int eocam_set_exposure(struct eocam_dev *sensor, u32 exposure)
{
	return 0;
}

static int eocam_get_gain(struct eocam_dev *sensor)
{
	return 0;
}

static int eocam_set_gain(struct eocam_dev *sensor, int gain)
{
	return 0;
}

static int eocam_set_autogain(struct eocam_dev *sensor, bool on)
{
	return 0;
}

static int eocam_set_stream_mipi(struct eocam_dev *sensor, bool on)
{
	return 0;
}

static int eocam_get_sysclk(struct eocam_dev *sensor)
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

static int eocam_set_night_mode(struct eocam_dev *sensor)
{
	return 0;
}

static int eocam_get_hts(struct eocam_dev *sensor)
{
	u16 hts;

	hts = 1896;

	return hts;
}

static int eocam_get_vts(struct eocam_dev *sensor)
{
	u16 vts;

	vts = 1080;

	return vts;
}

static int eocam_set_vts(struct eocam_dev *sensor, int vts)
{

	return 0;
}

static int eocam_get_light_freq(struct eocam_dev *sensor)
{
	return 120;

}

static int eocam_set_bandingfilter(struct eocam_dev *sensor)
{
	u32 band_step60, max_band60, band_step50, max_band50, prev_vts;
	int ret;

	/* read preview PCLK */
	ret = eocam_get_sysclk(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	//sensor->prev_sysclk : 4250
	sensor->prev_sysclk = ret;

	/* read preview HTS */
	ret = eocam_get_hts(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	//sensor->prev_hts    : 1892
	sensor->prev_hts = ret;

	/* read preview VTS */
	ret = eocam_get_vts(sensor);
	if (ret < 0)
		return ret;
	//prev_vts            : 740
	prev_vts = ret;

	/* calculate banding filter */
	/* 60Hz */
#if 0
	//band_step60         : 186
	band_step60 = sensor->prev_sysclk * 100 / sensor->prev_hts * 100 / 120;
	ret = eocam_write_reg16(sensor, EOSYS_REG_AEC_B60_STEP, band_step60);
	if (ret)
		return ret;
	if (!band_step60)
		return -EINVAL;

	//max_band60          : 3
	max_band60 = (int)((prev_vts - 4) / band_step60);
	ret = eocam_write_reg(sensor, EOSYS_REG_AEC_CTRL0D, max_band60);
	if (ret)
		return ret;
#endif
	/* 50Hz */
#if 0
	//band_step50         : 224
	band_step50 = sensor->prev_sysclk * 100 / sensor->prev_hts;
	ret = eocam_write_reg16(sensor, EOSYS_REG_AEC_B50_STEP, band_step50);
	if (ret)
		return ret;
	if (!band_step50)
		return -EINVAL;
	//max_band50          : 3
	max_band50 = (int)((prev_vts - 4) / band_step50);
#endif

	return 0;
}

static int eocam_set_ae_target(struct eocam_dev *sensor, int target)
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

static const struct eocam_mode_info *
eocam_find_mode(struct eocam_dev *sensor, enum eocam_frame_rate fr,
		 int width, int height, bool nearest)
{
	const struct eocam_mode_info *mode;

	mode = v4l2_find_nearest_size(eocam_mode_data,
				      ARRAY_SIZE(eocam_mode_data),
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
static int eocam_set_mode_exposure_calc(struct eocam_dev *sensor,
					 const struct eocam_mode_info *mode)
{
	u32 prev_shutter, prev_gain16;
	u32 cap_shutter, cap_gain16;
	u32 cap_sysclk, cap_hts, cap_vts;
	u32 light_freq, cap_bandfilt, cap_maxband;
	u32 cap_gain16_shutter;
	u8 average;
	int ret;

	/* read preview shutter */
	ret = eocam_get_exposure(sensor);
	if (ret < 0)
		return ret;
	prev_shutter = ret;

	/* read preview gain */
	ret = eocam_get_gain(sensor);
	if (ret < 0)
		return ret;
	prev_gain16 = ret;

	/* turn off night mode for capture */
	ret = eocam_set_night_mode(sensor);
	if (ret < 0)
		return ret;

	/* Write capture setting */
	ret = eocam_load_regs(sensor, mode);
	if (ret < 0)
		return ret;

	/* read capture VTS */
	ret = eocam_get_vts(sensor);
	if (ret < 0)
		return ret;
	cap_vts = ret;
	ret = eocam_get_hts(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	cap_hts = ret;

	ret = eocam_get_sysclk(sensor);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;
	cap_sysclk = ret;

	/* calculate capture banding filter */
	ret = eocam_get_light_freq(sensor);
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
		ret = eocam_get_sysclk(sensor);
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
	ret = eocam_set_gain(sensor, cap_gain16);
	if (ret)
		return ret;

	/* write capture shutter */
	if (cap_shutter > (cap_vts - 4)) {
		cap_vts = cap_shutter + 4;
		ret = eocam_set_vts(sensor, cap_vts);
		if (ret < 0)
			return ret;
	}

	/* set exposure */
	return eocam_set_exposure(sensor, cap_shutter);
}

/*
 * if sensor changes inside scaling or subsampling
 * change mode directly
 */
static int eocam_set_mode_direct(struct eocam_dev *sensor,
				  const struct eocam_mode_info *mode)
{
	/* Write capture setting */
	return eocam_load_regs(sensor, mode);
}

static int eocam_set_mode(struct eocam_dev *sensor)
{
	const struct eocam_mode_info *mode = sensor->current_mode;
	const struct eocam_mode_info *orig_mode = sensor->last_mode;
	enum eocam_downsize_mode dn_mode, orig_dn_mode;
	unsigned long rate;
	int ret;

	dn_mode = mode->dn_mode;
	orig_dn_mode = orig_mode->dn_mode;

	/*
	 * All the formats we support have 16 bits per pixel, seems to require
	 * the same rate than YUV, so we can just use 16 bpp all the time.
	 */
	rate = mode->vtot * mode->htot * 16;
	rate *= eocam_framerates[sensor->current_fr];

	if (sensor->ep.bus_type == V4L2_MBUS_CSI2_DPHY) {
		rate = rate / sensor->ep.bus.mipi_csi2.num_data_lanes;
		ret = eocam_set_mipi_pclk(sensor, rate);
	}

	ret = eocam_set_bandingfilter(sensor);
	if (ret < 0)
		return ret;

	sensor->pending_mode_change = false;
	sensor->last_mode = mode;

	return 0;
}

static int eocam_set_framefmt(struct eocam_dev *sensor,
			       struct v4l2_mbus_framefmt *format);

/* restore the last set video mode after chip power-on */
static int eocam_restore_mode(struct eocam_dev *sensor)
{
	int ret;

	/* now restore the last capture mode */
	ret = eocam_set_mode(sensor);
	if (ret < 0)
		return ret;

	return eocam_set_framefmt(sensor, &sensor->fmt);
}

static void eocam_power(struct eocam_dev *sensor, bool enable)
{

#if 0 // APPL_USE_PWR 	

	pr_err("%s ==> %d  \n", __func__, enable);

  gpio_set_value(sensor->pwr_gpio, enable ? 1 : 0);

	usleep_range(20000, 25000);
#endif
}

static void eocam_reset(struct eocam_dev *sensor)
{

}

static int eocam_set_power_on(struct eocam_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

#if 0
	ret = regulator_bulk_enable(eocam_NUM_SUPPLIES,
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto xclk_off;
	}
#endif

	eocam_reset(sensor);
	eocam_power(sensor, true);

	return 0;

power_off:
	eocam_power(sensor, false);
	//regulator_bulk_disable(eocam_NUM_SUPPLIES, sensor->supplies);
xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void eocam_set_power_off(struct eocam_dev *sensor)
{
	
	pr_err("%s ==> \n", __func__);

 return ;
 
	eocam_power(sensor, false);
	//regulator_bulk_disable(eocam_NUM_SUPPLIES, sensor->supplies);
	clk_disable_unprepare(sensor->xclk);
	sensor->streaming = false;
}

static int eocam_set_power(struct eocam_dev *sensor, bool on)
{
	int ret = 0;


	pr_err("%s ==> \n", __func__);

 return 0;

	if (on) {
		ret = eocam_set_power_on(sensor);
		if (ret)
			return ret;

		ret = eocam_restore_mode(sensor);
		if (ret)
			goto power_off;

		/* We're done here for DVP bus, while CSI-2 needs setup. */
		if (sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY)
			return 0;

		usleep_range(500, 1000);
	} else {
		eocam_set_power_off(sensor);
	}

	return 0;

power_off:
	eocam_set_power_off(sensor);
	return ret;
}

/* --------------- Subdev Operations --------------- */

static int eocam_s_power(struct v4l2_subdev *sd, int on)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	int ret = 0;



   pr_err("%s ==> \n", __func__);

   return 0;
 
 
	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		ret = eocam_set_power(sensor, !!on);
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

static int eocam_try_frame_interval(struct eocam_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct eocam_mode_info *mode;
	enum eocam_frame_rate rate = EOSYS_60_FPS;//EOSYS_08_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;

	minfps = eocam_framerates[EOSYS_60_FPS];
	maxfps = eocam_framerates[EOSYS_60_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = EOSYS_60_FPS;
		goto find_mode;
	}

	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	for (i = 0; i < ARRAY_SIZE(eocam_framerates); i++) {
		int curr_fps = eocam_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}

	fi->numerator = 1;
	fi->denominator = best_fps;

find_mode:
	mode = eocam_find_mode(sensor, rate, width, height, false);
	return mode ? rate : -EINVAL;
}

static int eocam_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	fmt->reserved[1] = (sensor->current_fr == EOSYS_60_FPS) ? 60 : 30;
	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

static int eocam_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum eocam_frame_rate fr,
				   const struct eocam_mode_info **new_mode)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	const struct eocam_mode_info *mode;
	int i;

	mode = eocam_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->hact;
	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(eocam_formats); i++)
		if (eocam_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(eocam_formats))
		i = 0;

	fmt->code = eocam_formats[i].code;
	fmt->colorspace = eocam_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int eocam_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	const struct eocam_mode_info *new_mode;
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

	ret = eocam_try_fmt_internal(sd, mbus_fmt,
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

static int eocam_set_framefmt(struct eocam_dev *sensor,
			       struct v4l2_mbus_framefmt *format)
{
	return 0;
}

/*
 * Sensor Controls.
 */

static int eocam_set_ctrl_hue(struct eocam_dev *sensor, int value)
{
	return 0;
}

static int eocam_set_ctrl_contrast(struct eocam_dev *sensor, int value)
{
	return 0;
}

static int eocam_set_ctrl_saturation(struct eocam_dev *sensor, int value)
{
	return 0;
}

static int eocam_set_ctrl_white_balance(struct eocam_dev *sensor, int awb)
{
	return 0;
}

static int eocam_set_ctrl_exposure(struct eocam_dev *sensor,
				    enum v4l2_exposure_auto_type auto_exposure)
{
	return 0;
}

static int eocam_set_ctrl_gain(struct eocam_dev *sensor, bool auto_gain)
{
	return 0;
}

static int eocam_set_ctrl_light_freq(struct eocam_dev *sensor, int value)
{
	return 0;
}

static int eocam_set_ctrl_hflip(struct eocam_dev *sensor, int value)
{
	return 0;
}

static int eocam_set_ctrl_vflip(struct eocam_dev *sensor, int value)
{
	return 0;
}

static int eocam_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct eocam_dev *sensor = to_eocam_dev(sd);
	int val;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		val = eocam_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		val = eocam_get_exposure(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.exposure->val = val;
		break;
	}

	return 0;
}

static int eocam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct eocam_dev *sensor = to_eocam_dev(sd);
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
		ret = eocam_set_ctrl_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = eocam_set_ctrl_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = eocam_set_ctrl_white_balance(sensor, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = eocam_set_ctrl_hue(sensor, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = eocam_set_ctrl_contrast(sensor, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = eocam_set_ctrl_saturation(sensor, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = eocam_set_ctrl_light_freq(sensor, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = eocam_set_ctrl_hflip(sensor, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = eocam_set_ctrl_vflip(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops eocam_ctrl_ops = {
	.g_volatile_ctrl = eocam_g_volatile_ctrl,
	.s_ctrl = eocam_s_ctrl,
};

static int eocam_init_controls(struct eocam_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &eocam_ctrl_ops;
	struct eocam_ctrls *ctrls = &sensor->ctrls;
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

static int eocam_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= EOSYS_NUM_MODES)
		return -EINVAL;

	fse->min_width  = eocam_mode_data[fse->index].hact;
	fse->max_width  = fse->min_width;
	fse->min_height = eocam_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	return 0;
}

static int eocam_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	int i, j, count;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= EOSYS_NUM_FRAMERATES)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < EOSYS_NUM_FRAMERATES; i++) {
		for (j = 0; j < EOSYS_NUM_MODES; j++) {
			if (fie->width  == eocam_mode_data[j].hact &&
			    fie->height == eocam_mode_data[j].vact &&
			    !eocam_check_valid_mode(sensor, &eocam_mode_data[j], i))
				count++;

			if (fie->index == (count - 1)) {
				fie->interval.denominator = eocam_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int eocam_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int eocam_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	const struct eocam_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = eocam_try_frame_interval(sensor, &fi->interval,
					       mode->hact, mode->vact);
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}

	mode = eocam_find_mode(sensor, frame_rate, mode->hact,
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

static int eocam_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(eocam_formats))
		return -EINVAL;

	code->code = eocam_formats[code->index].code;
	return 0;
}

static int eocam_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !enable) {
		ret = eocam_check_valid_mode(sensor,
					      sensor->current_mode,
					      sensor->current_fr);
		if (ret) {
			dev_err(&client->dev, "Not support WxH@fps=%dx%d@%d\n",
				sensor->current_mode->hact,
				sensor->current_mode->vact,
				eocam_framerates[sensor->current_fr]);
			goto out;
		}

		if (enable && sensor->pending_mode_change) {
			ret = eocam_set_mode(sensor);
			if (ret)
				goto out;
		}

		if (enable && sensor->pending_fmt_change) {
			ret = eocam_set_framefmt(sensor, &sensor->fmt);
			if (ret)
				goto out;
			sensor->pending_fmt_change = false;
		}

		sensor->ep.bus_type = V4L2_MBUS_CSI2_DPHY;
		eocam_set_stream_mipi(sensor, enable);
		sensor->streaming = enable;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops eocam_core_ops = {
	.s_power           = eocam_s_power,
	.log_status        = v4l2_ctrl_subdev_log_status,
	.subscribe_event   = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops eocam_video_ops = {
	.g_frame_interval  = eocam_g_frame_interval,
	.s_frame_interval  = eocam_s_frame_interval,
	.s_stream          = eocam_s_stream,
};

static const struct v4l2_subdev_pad_ops eocam_pad_ops = {
	.enum_mbus_code      = eocam_enum_mbus_code,
	.get_fmt             = eocam_get_fmt,
	.set_fmt             = eocam_set_fmt,
	.enum_frame_size     = eocam_enum_frame_size,
	.enum_frame_interval = eocam_enum_frame_interval,
};

static const struct v4l2_subdev_ops eocam_subdev_ops = {
	.core  = &eocam_core_ops,
	.video = &eocam_video_ops,
	.pad   = &eocam_pad_ops,
};

static int eocam_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations eocam_sd_media_ops = {
	.link_setup = eocam_link_setup,
};

#if 0
static int eocam_get_regulators(struct eocam_dev *sensor)
{
	int i;

	for (i = 0; i < EOSYS_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = eocam_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       EOSYS_NUM_SUPPLIES,
				       sensor->supplies);
}
#endif





static ssize_t value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t			status;
	
	status = 0;
#if  APPL_USE_PWR 	

//s.y.s
//	dev_err(dev, " %s   =========================>\n", __func__);
	status = gpio_get_value(eosys_g_pwr_gpio);

	if (status < 0)
		goto err;
	buf[0] = '0' + status;
	buf[1] = '\n';
#endif
	status = 2;

err:
	return status;
} 


static ssize_t value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
//s.y.s
//		dev_err(dev, " %s   =========================>\n", __func__);
 #if  APPL_USE_PWR 	
   

  //  dev_err(dev, "  %d \n", gpio_get_value(eosys_g_pwr_gpio)); 

    if(buf[0] == '0')
    {	
	         gpio_set_value(eosys_g_pwr_gpio, 0);
		       dev_err(dev, " %s Power off   %c ==>\n", __func__, buf[0]);
		 }      
    else
    {	
	         gpio_set_value(eosys_g_pwr_gpio, 1);
		       dev_err(dev, " %s Power on   %c ==>\n", __func__, buf[0]);
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


static int eocam_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct eocam_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	u32 rotation;
	int ret;
int pwr_enable_gpio;
	struct device_node *np = dev->of_node;

  pr_err("%s module \n", __func__);

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

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
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = eocam_framerates[EOSYS_60_FPS];
	sensor->current_fr = EOSYS_60_FPS;

	sensor->current_mode =
		&eocam_mode_data[EOSYS_MODE_640_480];

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
	if (sensor->xclk_freq < EOSYS_XCLK_MIN ||
	    sensor->xclk_freq > EOSYS_XCLK_MAX) {
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


#if  APPL_USE_PWR 	
  eosys_g_pwr_gpio = pwr_enable_gpio;
  sensor->pwr_gpio = pwr_enable_gpio;
#endif
  
	v4l2_i2c_subdev_init(&sensor->sd, client, &eocam_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &eocam_sd_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

#if 0
	ret = eocam_get_regulators(sensor);
	if (ret)
		return ret;
#endif

	mutex_init(&sensor->lock);

	ret = eocam_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret)
		goto free_ctrls;

	dev_info(dev, "eocam : i1643 probe success !\n");

#if APPL_USE_PWR	

	ret = sysfs_create_group(&client->dev.kobj, &gpio_group);
	if(ret < 0) {
		dev_err(dev, "failed to create sysfs files\n");
	}
#endif
 // ret = eocam_set_power(sensor, true);
 
 pr_err("%s  OK \n", __func__);

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int eocam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct eocam_dev *sensor = to_eocam_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);

	return 0;
}

static const struct i2c_device_id eocam_id[] = {
	{"eocam", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, eocam_id);

static const struct of_device_id eocam_dt_ids[] = {
	{ .compatible = "eocam,i1643" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, eocam_dt_ids);

static struct i2c_driver eocam_i2c_driver = {
	.driver = {
		.name  = "eocam",
		.of_match_table	= eocam_dt_ids,
	},
	.id_table  = eocam_id,
	.probe_new = eocam_probe,
	.remove    = eocam_remove,
};



static int __init module_begin(void){
  
	printk("eocam , linux kernel module. \n");
  return i2c_add_driver(&eocam_i2c_driver);
   
}
 
static void __exit module_end(void){
	
	printk("eocam Good Bye!\n");
  i2c_del_driver(&eocam_i2c_driver);
  return;
	
}
 
module_init(module_begin);
module_exit(module_end);

 
MODULE_DESCRIPTION("eocam MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
 