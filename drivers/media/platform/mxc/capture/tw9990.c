#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define GET_ID(val)  ((val & 0xF8) >> 3)
#define GET_REV(val) (val & 0x07)

/*
 * register offset
 */
#define ID			0x00 /* Product ID Code Register */
#define STATUS1		0x01 /* Chip Status Register I */
#define INFORM		0x02 /* Input Format */
#define OPFORM		0x03 /* Output Format Control Register */
#define DLYCTR		0x04 /* Hysteresis and HSYNC Delay Control */
#define OUTCTR1		0x05 /* Output Control I */
#define ACNTL1		0x06 /* Analog Control Register 1 */
#define CROP_HI		0x07 /* Cropping Register, High */
#define VDELAY_LO	0x08 /* Vertical Delay Register, Low */
#define VACTIVE_LO	0x09 /* Vertical Active Register, Low */
#define HDELAY_LO	0x0A /* Horizontal Delay Register, Low */
#define HACTIVE_LO	0x0B /* Horizontal Active Register, Low */
#define CNTRL1		0x0C /* Control Register I */
#define CMC 		0x0D /* Common Mode Clamp */
#define SDIAG		0x0F /* Short Diagnostic */
#define BRIGHT		0x10 /* BRIGHTNESS Control Register */
#define CONTRAST	0x11 /* CONTRAST Control Register */
#define SHARPNESS	0x12 /* SHARPNESS Control Register I */
#define SAT_U		0x13 /* Chroma (U) Gain Register */
#define SAT_V		0x14 /* Chroma (V) Gain Register */
#define HUE			0x15 /* Hue Control Register */
#define CORING1		0x17
#define CORING2		0x18 /* Coring and IF compensation */
#define VBICNTL		0x19 /* VBI Control Register */
#define ACNTL2		0x1A /* Analog Control 2 */
#define OUTCTR2		0x1B /* Output Control 2 */
#define SDT			0x1C /* Standard Selection */
#define SDTR		0x1D /* Standard Recognition */
#define TEST		0x1F /* Test Control Register */
#define CLMPG		0x20 /* Clamping Gain */
#define IAGC		0x21 /* Individual AGC Gain */
#define AGCGAIN		0x22 /* AGC Gain */
#define PEAKWT		0x23 /* White Peak Threshold */
#define CLMPL		0x24 /* Clamp level */
#define SYNCT		0x25 /* Sync Amplitude */
#define MISSCNT		0x26 /* Sync Miss Count Register */
#define PCLAMP		0x27 /* Clamp Position Register */
#define VCNTL1		0x28 /* Vertical Control I */
#define VCNTL2		0x29 /* Vertical Control II */
#define CKILL		0x2A /* Color Killer Level Control */
#define COMB		0x2B /* Comb Filter Control */
#define LDLY		0x2C /* Luma Delay and H Filter Control */
#define MISC1		0x2D /* Miscellaneous Control I */
#define LOOP		0x2E /* LOOP Control Register */
#define MISC2		0x2F /* Miscellaneous Control II */
#define MVSN		0x30 /* Macrovision Detection */
#define STATUS2		0x31 /* Chip STATUS II */
#define HFREF		0x32 /* H monitor */
#define CLMD		0x33 /* CLAMP MODE */
#define IDCNTL		0x34 /* ID Detection Control */
#define CLCNTL1		0x35 /* Clamp Control I */
#define ANAPLLCTL	0x4C
#define VBIMIN		0x4D
#define HSLOWCTL	0x4E
#define WSS3		0x4F
#define FILLDATA	0x50
#define SDID		0x51
#define DID			0x52
#define WSS1		0x53
#define WSS2		0x54
#define VVBI		0x55
#define LCTL6		0x56
#define LCTL7		0x57
#define LCTL8		0x58
#define LCTL9		0x59
#define LCTL10		0x5A
#define LCTL11		0x5B
#define LCTL12		0x5C
#define LCTL13		0x5D
#define LCTL14		0x5E
#define LCTL15		0x5F
#define LCTL16		0x60
#define LCTL17		0x61
#define LCTL18		0x62
#define LCTL19		0x63
#define LCTL20		0x64
#define LCTL21		0x65
#define LCTL22		0x66
#define LCTL23		0x67
#define LCTL24		0x68
#define LCTL25		0x69
#define LCTL26		0x6A
#define HSBEGIN		0x6B
#define HSEND		0x6C
#define OVSDLY		0x6D
#define OVSEND		0x6E
#define VBIDELAY	0x6F

/*
 * register detail
 */

/* INFORM */
#define FC27_ON     0x40 /* 1 : Input crystal clock frequency is 27MHz */
#define FC27_FF     0x00 /* 0 : Square pixel mode. */
			 /*     Must use 24.54MHz for 60Hz field rate */
			 /*     source or 29.5MHz for 50Hz field rate */
#define IFSEL_S     0x10 /* 01 : S-video decoding */
#define IFSEL_C     0x00 /* 00 : Composite video decoding */
			 /* Y input video selection */
#define YSEL_M0     0x00 /*  00 : Mux0 selected */
#define YSEL_M1     0x04 /*  01 : Mux1 selected */
#define YSEL_M2     0x08 /*  10 : Mux2 selected */
#define YSEL_M3     0x10 /*  11 : Mux3 selected */

/* OPFORM */
#define MODE        0x80 /* 0 : CCIR601 compatible YCrCb 4:2:2 format */
			 /* 1 : ITU-R-656 compatible data sequence format */
#define LEN         0x40 /* 0 : 8-bit YCrCb 4:2:2 output format */
			 /* 1 : 16-bit YCrCb 4:2:2 output format.*/
#define LLCMODE     0x20 /* 1 : LLC output mode. */
			 /* 0 : free-run output mode */
#define AINC        0x10 /* Serial interface auto-indexing control */
			 /* 0 : auto-increment */
			 /* 1 : non-auto */
#define VSCTL       0x08 /* 1 : Vertical out ctrl by DVALID */
			 /* 0 : Vertical out ctrl by HACTIVE and DVALID */
#define OEN_TRI_SEL_MASK	0x07
#define OEN_TRI_SEL_ALL_ON	0x00 /* Enable output for Rev0/Rev1 */
#define OEN_TRI_SEL_ALL_OFF_r0	0x06 /* All tri-stated for Rev0 */
#define OEN_TRI_SEL_ALL_OFF_r1	0x07 /* All tri-stated for Rev1 */

/* OUTCTR1 */
#define VSP_LO      0x00 /* 0 : VS pin output polarity is active low */
#define VSP_HI      0x80 /* 1 : VS pin output polarity is active high. */
			 /* VS pin output control */
#define VSSL_VSYNC  0x00 /*   0 : VSYNC  */
#define VSSL_VACT   0x10 /*   1 : VACT   */
#define VSSL_FIELD  0x20 /*   2 : FIELD  */
#define VSSL_VVALID 0x30 /*   3 : VVALID */
#define VSSL_ZERO   0x70 /*   7 : 0      */
#define HSP_LOW     0x00 /* 0 : HS pin output polarity is active low */
#define HSP_HI      0x08 /* 1 : HS pin output polarity is active high.*/
			 /* HS pin output control */
#define HSSL_HACT   0x00 /*   0 : HACT   */
#define HSSL_HSYNC  0x01 /*   1 : HSYNC  */
#define HSSL_DVALID 0x02 /*   2 : DVALID */
#define HSSL_HLOCK  0x03 /*   3 : HLOCK  */
#define HSSL_ASYNCW 0x04 /*   4 : ASYNCW */
#define HSSL_ZERO   0x07 /*   7 : 0      */

/* ACNTL1 */
#define SRESET      0x80 /* resets the device to its default state
			  * but all register content remain unchanged.
			  * This bit is self-resetting.
			  */
#define ACNTL1_PDN_MASK	0x0e
#define CLK_PDN		0x08 /* system clock power down */
#define Y_PDN		0x04 /* Luma ADC power down */
#define C_PDN		0x02 /* Chroma ADC power down */

/* ACNTL2 */
#define ACNTL2_PDN_MASK	0x40
#define PLL_PDN		0x40 /* PLL power down */

/* VBICNTL */

/* RTSEL : control the real time signal output from the MPOUT pin */
#define RTSEL_MASK  0x07
#define RTSEL_VLOSS 0x00 /* 0000 = Video loss */
#define RTSEL_HLOCK 0x01 /* 0001 = H-lock */
#define RTSEL_SLOCK 0x02 /* 0010 = S-lock */
#define RTSEL_VLOCK 0x03 /* 0011 = V-lock */
#define RTSEL_MONO  0x04 /* 0100 = MONO */
#define RTSEL_DET50 0x05 /* 0101 = DET50 */
#define RTSEL_FIELD 0x06 /* 0110 = FIELD */
#define RTSEL_RTCO  0x07 /* 0111 = RTCO ( Real Time Control ) */

static int tw9990_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int tw9990_detach(struct i2c_client *client);

static const struct i2c_device_id tw9990_id[] = {
	{"tw9990", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tw9990_id);

static struct i2c_driver tw9990_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "tw9990",
		   },
	.probe = tw9990_probe,
	.remove = tw9990_detach,
	.id_table = tw9990_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct sensor_data sen;
	v4l2_std_id std_id;
} tw9990_data;


/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	TW9990_NTSC = 0,    /*!< Locked on (M) NTSC video signal. */
	TW9990_PAL,         /*!< (B, G, H, I, N)PAL video signal. */
	TW9990_NOT_LOCKED,  /*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define TW9990_STD_MAX		(TW9990_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	int frame_rate;		/*!< Frame rate. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 .frame_rate = 30,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 25,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 0,
	 },
};

/*!* Standard index of TW9990. */
static video_fmt_idx video_idx = TW9990_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

/* supported controls */
/* This hasn't been fully implemented yet.
 * This is how it should work, though. */
static struct v4l2_queryctrl tw9990_qctrl[] = {
	{
	.id = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 0x1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}
};

static inline int tw9990_read(u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(tw9990_data.sen.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return val;
	}
	return val;
}

static int tw9990_write_reg(u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(tw9990_data.sen.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return ret;
	}
	return 0;
}

static int tw9990_mask_set(u8 reg, u8 mask, u8 val)
{
	s32 ret = tw9990_read(reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val & mask;

	return tw9990_write_reg(reg, ret);
}

static void tw9990_reset(void)
{
	tw9990_mask_set(ACNTL1, SRESET, SRESET);
	msleep(10);
}

unsigned char tw9990_reg_PAL[][2] = {
	0x00, 0x00,
	0x01, 0x68,
	0x02, 0xC0,
	0x03, 0xA0,
	0x04, 0x00,
	0x05, 0x00,
	0x06, 0x00,
	0x07, 0x12,
	0x08, 0x12,
	0x09, 0x20,
	0x0A, 0x10,
	0x0B, 0xD0,
	0x0C, 0xCC,
	0x0D, 0x74,
	0x0E, 0x11,
	0x0F, 0x31,
	0x10, 0x00,
	0x11, 0x64,
	0x12, 0x21,
	0x13, 0x80,
	0x14, 0x80,
	0x15, 0x00,
	0x16, 0x00,
	0x17, 0x30,
	0x18, 0x44,
	0x19, 0x58,
	0x1A, 0x0A,
	0x1B, 0x00,
	0x1C, 0x07,
	0x1D, 0x7F,
	0x1E, 0x08,
	0x1F, 0x00,
	0x20, 0xA0,
	0x21, 0x42,
	0x22, 0xF0,
	0x23, 0xD8,
	0x24, 0xBC,
	0x25, 0xB8,
	0x26, 0x44,
	0x27, 0x2A,
	0x28, 0x00,
	0x29, 0x00,
	0x2A, 0x78,
	0x2B, 0x44,
	0x2C, 0x40,
	0x2D, 0x14,
	0x2E, 0xA5,
	0x2F, 0xE0,
	0x30, 0x00,
	0x31, 0x10,
	0x32, 0xFF,
	0x33, 0x05,
	0x34, 0x1A,
	0x35, 0xA0,
	0x4C, 0x05,
	0x4D, 0x40,
	0x4E, 0x00,
	0x4F, 0x00,
	0x50, 0xA0,
	0x51, 0x22,
	0x52, 0x31,
	0x53, 0x80,
	0x54, 0x00,
	0x55, 0x10,
	0x6B, 0x06,
	0x6C, 0x24,
	0x6D, 0x00,
	0x6E, 0x20,
	0x6F, 0x13,

	0xFF, 0x00,  //; Page 0
};

unsigned char tw9990_reg_NTSC[][2] = {
	0x00, 0x00,
	0x01, 0x68,
	0x02, 0xC0,
	0x03, 0xA0,
	0x04, 0x00,
	0x05, 0x00,
	0x06, 0x00,
	0x07, 0x02,
	0x08, 0x12,
	0x09, 0xF0,
	0x0A, 0x10,
	0x0B, 0xD0,
	0x0C, 0xCC,
	0x0D, 0x74,
	0x0E, 0x11,
	0x0F, 0x31,
	0x10, 0x00,
	0x11, 0x64,
	0x12, 0x21,
	0x13, 0x80,
	0x14, 0x80,
	0x15, 0x00,
	0x16, 0x00,
	0x17, 0x30,
	0x18, 0x44,
	0x19, 0x58,
	0x1A, 0x0A,
	0x1B, 0x00,
	0x1C, 0x07,
	0x1D, 0x7F,
	0x1E, 0x08,
	0x1F, 0x00,
	0x20, 0xA0,
	0x21, 0x42,
	0x22, 0xF0,
	0x23, 0xD8,
	0x24, 0xBC,
	0x25, 0xB8,
	0x26, 0x44,
	0x27, 0x2A,
	0x28, 0x00,
	0x29, 0x00,
	0x2A, 0x78,
	0x2B, 0x44,
	0x2C, 0x40,
	0x2D, 0x14,
	0x2E, 0xA5,
	0x2F, 0xE0,
	0x30, 0x00,
	0x31, 0x10,
	0x32, 0xFF,
	0x33, 0x05,
	0x34, 0x1A,
	0x35, 0xA0,
	0x4C, 0x05,
	0x4D, 0x40,
	0x4E, 0x00,
	0x4F, 0x00,
	0x50, 0xA0,
	0x51, 0x22,
	0x52, 0x31,
	0x53, 0x80,
	0x54, 0x00,
	0x55, 0x00,
	0x56, 0x00,
	0x57, 0x00,
	0x58, 0x00,
	0x59, 0x00,
	0x5A, 0x00,
	0x5B, 0x00,
	0x5C, 0x00,
	0x5D, 0x00,
	0x5E, 0x00,
	0x5F, 0x00,
	0x60, 0x00,
	0x61, 0x00,
	0x62, 0x00,
	0x63, 0x00,
	0x64, 0x00,
	0x65, 0x00,
	0x66, 0x00,
	0x67, 0x00,
	0x68, 0x00,
	0x69, 0x00,
	0x6A, 0x00,
	0x6B, 0x06,
	0x6C, 0x24,
	0x6D, 0x00,
	0x6E, 0x20,
	0x6F, 0x13,
	0xAE, 0x1A,
	0xAF, 0x14,
	0xB0, 0x80,
	0xB1, 0x00,
	0xB2, 0x00,
	0xB3, 0x00,
	0xB4, 0x00,
	0xB5, 0x00,
	0xB6, 0x80,
	0xB7, 0x06,
	0xB8, 0x00,
	0xB9, 0x80,
	0xBA, 0x77,
	0xBB, 0x00,
	0xBC, 0x00,
	0xBD, 0x80,
	0xBE, 0x80,
	0xBF, 0x80,
	0xC0, 0x80,

	0xFF, 0x00,  //; Page 0
};

static void tw9990_PAL(void)
{
	int i = 0;
	printk("tw9990_PAL \n");
	while(1)
	{
		if(tw9990_reg_PAL[i][0] == 0xFF)
			break;
		tw9990_write_reg(tw9990_reg_PAL[i][0], tw9990_reg_PAL[i][1]);
		i++;
	}
}

static void tw9990_NTSC(void)
{
	int i = 0;
	printk("tw9990_NTSC \n");
	while(1)
	{
		if(tw9990_reg_NTSC[i][0] == 0xFF)
			break;
		tw9990_write_reg(tw9990_reg_NTSC[i][0], tw9990_reg_NTSC[i][1]);
		i++;
	}
}

static int tw9990_power(int enable)
{
	int ret;
	u8 acntl1;
	u8 acntl2;

	if (enable) {
		acntl1 = 0;
		acntl2 = 0;
	} else {
		acntl1 = CLK_PDN | Y_PDN | C_PDN;
		acntl2 = PLL_PDN;
	}

	ret = tw9990_mask_set(ACNTL1, ACNTL1_PDN_MASK, acntl1);
	if (ret < 0)
		return ret;

	return tw9990_mask_set(ACNTL2, ACNTL2_PDN_MASK, acntl2);
}

static void tw9990_get_std(v4l2_std_id *std)
{
	int val, locked, standard, idx;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990_get_std\n");

	val = tw9990_read(STATUS1);
	dev_info(&tw9990_data.sen.i2c_client->dev, "STATUS1=%x\n", val);
	locked = !(val & 0x80);
	val = tw9990_read(SDT);
	dev_info(&tw9990_data.sen.i2c_client->dev, "SDT=%x\n", val);
	standard = val & 0x70;

	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = TW9990_NOT_LOCKED;
	if (locked) {
		if (standard == 0x10) {
			*std = V4L2_STD_PAL;
			idx = TW9990_PAL;
		} else if (standard == 0) {
			*std = V4L2_STD_NTSC;
			idx = TW9990_NTSC;
		}
	}
	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != tw9990_data.std_id) {
		video_idx = idx;
		tw9990_data.std_id = *std;
		tw9990_data.sen.pix.width = video_fmts[video_idx].raw_width;
		tw9990_data.sen.pix.height = video_fmts[video_idx].raw_height;
		if (*std == V4L2_STD_NTSC) {
			/*tw9990_write_reg(CROP_HI, 0x02);
			tw9990_write_reg(VACTIVE_LO, 0xf4);
			tw9990_write_reg(VDELAY_LO, 0x12);
			tw9990_mask_set(VVBI, 0x10, 0x10);*/
			tw9990_NTSC();
		}
		else if (*std == V4L2_STD_PAL) {
			/*tw9990_write_reg(CROP_HI, 0x12);
			tw9990_write_reg(VDELAY_LO, 0x18);
			tw9990_write_reg(VACTIVE_LO, 0x20);
			tw9990_mask_set(VVBI, 0x10, 0x00);;*/
			tw9990_PAL();
		}
	}
}

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	dev_dbg(&tw9990_data.sen.i2c_client->dev, "tw9990:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.nobt_hs_inv = 1;
	p->u.bt656.bt_sync_correct = 1;

	/* TW9990 has a dedicated clock so no clock settings needed. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "tw9990:ioctl_s_power\n");

	if (on ^ sensor->sen.on)
		tw9990_power(on);
	sensor->sen.on = on;
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* added for changed tw9990 input channel */
        switch(a->parm.capture.reserved[0]){
        case 0:
	    	tw9990_write_reg(INFORM, 0x40);
            break;
        case 1:
    		tw9990_write_reg(INFORM, 0x44);
            break;
        default:
	    	tw9990_write_reg(INFORM, 0x40);
            break;
        }
		tw9990_write_reg(INFORM, 0x44);
		printk("ioctl_s_parm \n");
        break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "tw9990:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		tw9990_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	int i;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "tw9990:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(tw9990_qctrl); i++)
		if (qc->id && qc->id == tw9990_qctrl[i].id) {
			memcpy(qc, &(tw9990_qctrl[i]),
				sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	int sat = 0;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tw9990_data.sen.brightness = tw9990_read(BRIGHT);
		vc->value = tw9990_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		vc->value = tw9990_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		sat = tw9990_read(SAT_U);
		tw9990_data.sen.saturation = sat;
		vc->value = tw9990_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		vc->value = tw9990_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		vc->value = tw9990_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = tw9990_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		vc->value = tw9990_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   Default case\n");
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 tmp;

	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990:ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		tw9990_write_reg(BRIGHT, tmp);
		tw9990_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		tw9990_write_reg(SAT_U, tmp);
		tw9990_write_reg(SAT_V, tmp);
		tw9990_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&tw9990_data.sen.i2c_client->dev,
			"   Default case\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	video_fmt_t fmt;
	int i;

	if (fival->index != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(video_fmts) - 1; i++) {
		fmt = video_fmts[i];
		if (fival->width  == fmt.active_width &&
		    fival->height == fmt.active_height) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = fmt.frame_rate;
			return 0;
		}
	}

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"tw9990_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_TW9990;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990:ioctl_init\n");
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	dev_dbg(&tw9990_data.sen.i2c_client->dev, "tw9990:ioctl_dev_init\n");
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc tw9990_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)
				ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tw9990_slave = {
	.ioctls = tw9990_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tw9990_ioctl_desc),
};

static struct v4l2_int_device tw9990_int_device = {
	.module = THIS_MODULE,
	.name = "tw9990",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tw9990_slave,
	},
};

/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/*!
 * TW9990 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int tw9990_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int cid, rev;
	int ret = 0;
	enum of_gpio_flags flags;
	int pdn_gpio = -1, pdn_active = 0, rstb_gpio = -1, rstb_active = 0;
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev,
			"I2C-Adapter doesn't support "
			"I2C_FUNC_SMBUS_BYTE_DATA\n");
		return -EIO;
	}

	/* pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pdn_gpio = of_get_named_gpio_flags(dev->of_node, "pdn-gpios", 0, &flags);
	if (gpio_is_valid(pdn_gpio)) {
		unsigned long gpiof;
		if (flags == OF_GPIO_ACTIVE_LOW) {
			pdn_active = 0;
			gpiof = GPIOF_OUT_INIT_HIGH;
		}
		else {
			pdn_active = 1;
			gpiof = GPIOF_OUT_INIT_LOW;
		}

		if (devm_gpio_request_one(dev, pdn_gpio, gpiof, "tw9990_pdn")) {
			dev_warn(dev, "no power pin available!\n");
			pdn_gpio = -1;
		}
	}
	else
		dev_err(dev, "no sensor pdn pin available\n");

	/* request reset pin */
	rstb_gpio = of_get_named_gpio_flags(dev->of_node, "rstb-gpios", 0, &flags);
	if (gpio_is_valid(rstb_gpio)) {
		unsigned long gpiof;
		if (flags == OF_GPIO_ACTIVE_LOW) {
			rstb_active = 0;
			gpiof = GPIOF_OUT_INIT_HIGH;
		}
		else {
			rstb_active = 1;
			gpiof = GPIOF_OUT_INIT_LOW;
		}

		if (devm_gpio_request_one(dev, rstb_gpio, gpiof, "tw9990_rstb")) {
			dev_warn(dev, "no reset pin available!\n");
			rstb_gpio = -1;
		}
	}
	else
		dev_err(dev, "no sensor rstb pin available\n");

	if (pdn_gpio >= 0) {
		gpio_set_value_cansleep(pdn_gpio, !pdn_active);
		msleep(10);
	}
	if (rstb_gpio >= 0) {
		gpio_set_value_cansleep(rstb_gpio, rstb_active);
		msleep(10);
		gpio_set_value_cansleep(rstb_gpio, !rstb_active);
		msleep(10);
	}

	/* Set initial values for the sensor struct. */
	memset(&tw9990_data, 0, sizeof(tw9990_data));
	tw9990_data.sen.i2c_client = client;
	tw9990_data.sen.streamcap.timeperframe.denominator = 30;
	tw9990_data.sen.streamcap.timeperframe.numerator = 1;
	tw9990_data.std_id = V4L2_STD_ALL;
	video_idx = TW9990_NOT_LOCKED;
	tw9990_data.sen.pix.width = video_fmts[video_idx].raw_width;
	tw9990_data.sen.pix.height = video_fmts[video_idx].raw_height;
	tw9990_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	tw9990_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	tw9990_data.sen.on = true;

	tw9990_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(tw9990_data.sen.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(tw9990_data.sen.sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
					&tw9990_data.sen.mclk);
	if (ret) {
		dev_err(dev, "mclk frequency is invalid\n");
		return ret;
	}

	ret = of_property_read_u32(
		dev->of_node, "mclk_source",
		(u32 *) &(tw9990_data.sen.mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&(tw9990_data.sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	clk_prepare_enable(tw9990_data.sen.sensor_clk);

	dev_info(&tw9990_data.sen.i2c_client->dev,
		"%s:tw9990 probe i2c address is 0x%02X\n",
		__func__, tw9990_data.sen.i2c_client->addr);

	cid = tw9990_read(ID);
	rev = GET_REV(cid);
	cid = GET_ID(cid);
	dev_info(dev, "tw9990 Product ID %0x:%0x\n", cid, rev);

	/*! TW9990 initialization. */
	tw9990_power(1);
	tw9990_reset();

	tw9990_write_reg(INFORM, 0x44);
	tw9990_write_reg(OPFORM, 0xa2);
	tw9990_write_reg(OUTCTR1, 0x01);
	tw9990_write_reg(HACTIVE_LO, 0xd0);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the tw9990_data structure here.*/
	tw9990_int_device.priv = &tw9990_data;
	ret = v4l2_int_device_register(&tw9990_int_device);

	clk_disable_unprepare(tw9990_data.sen.sensor_clk);

	dev_info(dev, "tw9990 probe done %d\n", ret);

	return ret;
}

/*!
 * TW9990 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int tw9990_detach(struct i2c_client *client)
{
	dev_dbg(&tw9990_data.sen.i2c_client->dev,
		"%s:Removing tw9990 video decoder @ 0x%02X from adapter %s\n",
		__func__, client->addr << 1, client->adapter->name);

	/* Power down via i2c */
	tw9990_power(0);

	v4l2_int_device_unregister(&tw9990_int_device);

	return 0;
}

/*!
 * TW9990 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int tw9990_init(void)
{
	u8 err = 0;

	pr_debug("In tw9990_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&tw9990_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * TW9990 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit tw9990_clean(void)
{
	dev_dbg(&tw9990_data.sen.i2c_client->dev, "In tw9990_clean\n");
	i2c_del_driver(&tw9990_i2c_driver);
}

module_init(tw9990_init);
module_exit(tw9990_clean);

MODULE_AUTHOR("Shaojun Bian");
MODULE_DESCRIPTION("TW9990 video decoder driver");
MODULE_LICENSE("GPL");
















