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
#include "media/v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define GET_ID(val)  ((val & 0xFE) >> 1)
#define GET_REV(val) (val & 0x01)

/*
 * register offset
 */
#define I2C_SLAVE_ID	0x00 /* I2C_SLAVE_ID */
#define CONFIG1		0x01 /* CONFIG1 */
#define CONFIG2		0x02 /* CONFIG2 */
#define EQ			0x03 /* EQ Control */
#define CMLOUT		0x04 /* CMLOUT Config */
#define NA1			0x05 /* Reserved */
#define NA2			0x06 /* Reserved */
#define NA3			0x07 /* Reserved */
#define NA4			0x08 /* Reserved */
#define NA5			0x09 /* Reserved */
#define NA6			0x0A /* Reserved */
#define NA7			0x0B /* Reserved */
#define NA8			0x0C /* Reserved */
#define NA9			0x0D /* Reserved */
#define NA10			0x0F /* Reserved */
#define NA11			0x10 /* Reserved */
#define CSI_CONFIG		0x11 /* CSI config */
#define CSI_FRM_GAP_0	0x12 /* CSI_FRM_GAP_0 */
#define CSI_FRM_GAP_1	0x13 /* CSI_FRM_GAP_0 */
#define CSI_TIMING0		0x14 /* CSI_TIMING0 */
#define CSI_TIMING1		0x15 /* CSI_TIMING1 */
#define CSI_TIMING2		0x16 /* CSI_TIMING2 */
#define CSI_TIMING3		0x17 /* CSI_TIMING3 */
#define CSI_TIMING4		0x18 /* CSI_TIMING4 */
#define CSI_ULPS		0x19 /* CSI_ULPS */
#define NA		0x1A /* CSI_ULPS */
#define CSI_UNH1		0x1B /* CSI_UNH1 */
#define CSI_UNH2		0x1C /* CSI_UNH2 */
#define CSI_UNH3		0x1D /* CSI_UNH3 */
#define CSI_UNH4		0x1E /* CSI_UNH4 */
#define CSI_UNH5		0x1F /* CSI_UNH5 */
#define CSI_UNH6		0x20 /* CSI_UNH6 */
#define CSI_UNH7		0x21 /* CSI_UNH7 */
#define CSI_UNH8		0x22 /* CSI_UNH8 */
#define CSI_UNH9		0x23 /* CSI_UNH9 */
#define CSI_ID0		0x30 /* CSI_ID0 */
#define CSI_ID1		0x31 /* CSI_ID1 */
#define CSI_ID2		0x32 /* CSI_ID2 */
#define CSI_ID3		0x33 /* CSI_ID3 */
#define CSI_ID4		0x34 /* CSI_ID4 */
#define CSI_ID5		0x35 /* CSI_ID5 */
#define CSI_REVID	0x36 /* CSI_REVID */
#define REVID		0x3B /* REVID */


static int ds90ur910q_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int ds90ur910q_detach(struct i2c_client *client);

static const struct i2c_device_id ds90ur910q_id[] = {
	{"ti,ds90ur910q", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ds90ur910q_id);

static const struct of_device_id ds90ur910q_of_match[] = {
       { .compatible = "ti,ds90ur910q", },
       { }
};
MODULE_DEVICE_TABLE(of, ds90ur910q_of_match);

static struct i2c_driver ds90ur910q_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ds90ur910q",
		   .of_match_table = ds90ur910q_of_match,
		   },
	.probe = ds90ur910q_probe,
	.remove = ds90ur910q_detach,
	.id_table = ds90ur910q_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct sensor_data sen;
	v4l2_std_id std_id;
} ds90ur910q_data;


/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	ds90ur910q_NTSC = 0,    /*!< Locked on (M) NTSC video signal. */
	ds90ur910q_PAL,         /*!< (B, G, H, I, N)PAL video signal. */
	ds90ur910q_NOT_LOCKED,  /*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define ds90ur910q_STD_MAX		(ds90ur910q_PAL + 1)

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

/*!* Standard index of ds90ur910q. */
static video_fmt_idx video_idx = ds90ur910q_PAL;

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
static struct v4l2_queryctrl ds90ur910q_qctrl[] = {
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

static inline int ds90ur910q_read(u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(ds90ur910q_data.sen.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return val;
	}
	return val;
}

static int ds90ur910q_write_reg(u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(ds90ur910q_data.sen.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return ret;
	}
	return 0;
}

static int ds90ur910q_mask_set(u8 reg, u8 mask, u8 val)
{
	#if 0
	s32 ret = ds90ur910q_read(reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val & mask;

	return ds90ur910q_write_reg(reg, ret);
	#else
	return 0;
	#endif
}

static void ds90ur910q_reset(void)
{
	#if 0
	ds90ur910q_mask_set(ACNTL1, SRESET, SRESET);
	msleep(1);
	#endif
}

static int ds90ur910q_power(int enable)
{
	#if 0
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

	ret = ds90ur910q_mask_set(ACNTL1, ACNTL1_PDN_MASK, acntl1);
	if (ret < 0)
		return ret;

	return ds90ur910q_mask_set(ACNTL2, ACNTL2_PDN_MASK, acntl2);
	#else
	return 0;
	#endif
}
#if 0
static void ds90ur910q_get_std(v4l2_std_id *std)
{
	int val, locked, standard, idx;

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q_get_std\n");

	val = ds90ur910q_read(STATUS1);
	dev_info(&ds90ur910q_data.sen.i2c_client->dev, "STATUS1=%x\n", val);
	locked = !(val & 0x80);
	val = ds90ur910q_read(SDT);
	dev_info(&ds90ur910q_data.sen.i2c_client->dev, "SDT=%x\n", val);
	standard = val & 0x70;

	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = ds90ur910q_NOT_LOCKED;
	if (locked) {
		if (standard == 0x10) {
			*std = V4L2_STD_PAL;
			idx = ds90ur910q_PAL;
		} else if (standard == 0) {
			*std = V4L2_STD_NTSC;
			idx = ds90ur910q_NTSC;
		}
	}
	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != ds90ur910q_data.std_id) {
		video_idx = idx;
		ds90ur910q_data.std_id = *std;
		ds90ur910q_data.sen.pix.width = video_fmts[video_idx].raw_width;
		ds90ur910q_data.sen.pix.height = video_fmts[video_idx].raw_height;
		if (*std == V4L2_STD_NTSC) {
			ds90ur910q_write_reg(CROP_HI, 0x02);
			ds90ur910q_write_reg(VDELAY_LO, 0x12);
			ds90ur910q_write_reg(VACTIVE_LO, 0xf4);
			ds90ur910q_mask_set(VVBI, 0x10, 0x10);
		}
		else if (*std == V4L2_STD_PAL) {
			ds90ur910q_write_reg(CROP_HI, 0x12);
			ds90ur910q_write_reg(VDELAY_LO, 0x18);
			ds90ur910q_write_reg(VACTIVE_LO, 0x20);
			ds90ur910q_mask_set(VVBI, 0x10, 0x00);
		}
	}
}
#endif

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
	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "ds90ur910q:ioctl_g_ifparm\n");

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

	/* ds90ur910q has a dedicated clock so no clock settings needed. */

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

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "ds90ur910q:ioctl_s_power\n");

	if (on ^ sensor->sen.on)
		ds90ur910q_power(on);
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

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q:ioctl_g_parm\n");

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
	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
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
	//struct sensor *sensor = s->priv;

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "ds90ur910q:ioctl_g_fmt_cap\n");
#if 0
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		ds90ur910q_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}
#endif
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

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "ds90ur910q:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(ds90ur910q_qctrl); i++)
		if (qc->id && qc->id == ds90ur910q_qctrl[i].id) {
			memcpy(qc, &(ds90ur910q_qctrl[i]),
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
	//int sat = 0;

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		//ds90ur910q_data.sen.brightness = ds90ur910q_read(BRIGHT);
		vc->value = ds90ur910q_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		vc->value = ds90ur910q_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		//sat = ds90ur910q_read(SAT_U);
		//ds90ur910q_data.sen.saturation = sat;
		vc->value = ds90ur910q_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		vc->value = ds90ur910q_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		vc->value = ds90ur910q_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = ds90ur910q_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		vc->value = ds90ur910q_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
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

	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q:ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		//tmp = vc->value;
		//ds90ur910q_write_reg(BRIGHT, tmp);
		//ds90ur910q_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		//tmp = vc->value;
		//ds90ur910q_write_reg(SAT_U, tmp);
		//ds90ur910q_write_reg(SAT_V, tmp);
		//ds90ur910q_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
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
						"ds90ur910q_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = 9991;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q:ioctl_init\n");
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
	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "ds90ur910q:ioctl_dev_init\n");
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc ds90ur910q_ioctl_desc[] = {

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

static struct v4l2_int_slave ds90ur910q_slave = {
	.ioctls = ds90ur910q_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ds90ur910q_ioctl_desc),
};

static struct v4l2_int_device ds90ur910q_int_device = {
	.module = THIS_MODULE,
	.name = "ds90ur910q",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ds90ur910q_slave,
	},
};


/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/*!
 * ds90ur910q I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int ds90ur910q_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int cid, rev;
	int ret = 0;
	enum of_gpio_flags flags;
	int pdn_gpio = -1, pdn_active = 0, rstb_gpio = -1, rstb_active = 0;
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	dev_err(dev, "ds90ur910q_probe\n ");

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
	/*pdn_gpio = of_get_named_gpio_flags(dev->of_node, "pdn-gpios", 0, &flags);
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

		if (devm_gpio_request_one(dev, pdn_gpio, gpiof, "ds90ur910q_pdn")) {
			dev_warn(dev, "no power pin available!\n");
			pdn_gpio = -1;
		}
	}
	else
		dev_err(dev, "no sensor pdn pin available\n");
	*/
	/* request reset pin */
	/*rstb_gpio = of_get_named_gpio_flags(dev->of_node, "rstb-gpios", 0, &flags);
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

		if (devm_gpio_request_one(dev, rstb_gpio, gpiof, "ds90ur910q_rstb")) {
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
	*/

	/* Set initial values for the sensor struct. */
	memset(&ds90ur910q_data, 0, sizeof(ds90ur910q_data));
	ds90ur910q_data.sen.i2c_client = client;
	ds90ur910q_data.sen.streamcap.timeperframe.denominator = 30;
	ds90ur910q_data.sen.streamcap.timeperframe.numerator = 1;
	ds90ur910q_data.std_id = V4L2_STD_ALL;
	video_idx = ds90ur910q_NOT_LOCKED;
	ds90ur910q_data.sen.pix.width = video_fmts[video_idx].raw_width;
	ds90ur910q_data.sen.pix.height = video_fmts[video_idx].raw_height;
	ds90ur910q_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	ds90ur910q_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	ds90ur910q_data.sen.on = true;

	/*ds90ur910q_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ds90ur910q_data.sen.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(ds90ur910q_data.sen.sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
					&ds90ur910q_data.sen.mclk);
	if (ret) {
		dev_err(dev, "mclk frequency is invalid\n");
		return ret;
	}

	ret = of_property_read_u32(
		dev->of_node, "mclk_source",
		(u32 *) &(ds90ur910q_data.sen.mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source invalid\n");
		return ret;
	}*/

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&(ds90ur910q_data.sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	//clk_prepare_enable(ds90ur910q_data.sen.sensor_clk);

	dev_info(&ds90ur910q_data.sen.i2c_client->dev,
		"%s:ds90ur910q probe i2c address is 0x%02X\n",
		__func__, ds90ur910q_data.sen.i2c_client->addr);

	cid = ds90ur910q_read(REVID);
	rev = GET_REV(cid);
	cid = GET_ID(cid);
	dev_info(dev, "ds90ur910q Product ID %0x:%0x\n", cid, rev);

	/*! ds90ur910q initialization. */
	//ds90ur910q_power(1);
	//ds90ur910q_reset();

	//ds90ur910q_write_reg(OPFORM, 0xa2);
	//ds90ur910q_write_reg(OUTCTR1, 0x01);
	//ds90ur910q_write_reg(HACTIVE_LO, 0xd0);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the ds90ur910q_data structure here.*/
	ds90ur910q_int_device.priv = &ds90ur910q_data;
	ret = v4l2_int_device_register(&ds90ur910q_int_device);

	//clk_disable_unprepare(ds90ur910q_data.sen.sensor_clk);

	dev_info(dev, "ds90ur910q probe done %d\n", ret);

	return ret;
}

/*!
 * ds90ur910q I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int ds90ur910q_detach(struct i2c_client *client)
{
	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev,
		"%s:Removing ds90ur910q video decoder @ 0x%02X from adapter %s\n",
		__func__, client->addr << 1, client->adapter->name);

	/* Power down via i2c */
	ds90ur910q_power(0);

	v4l2_int_device_unregister(&ds90ur910q_int_device);

	return 0;
}

/*!
 * ds90ur910q init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int ds90ur910q_init(void)
{
	u8 err = 0;

	pr_err("In ds90ur910q_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&ds90ur910q_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);
	pr_err("In ds90ur910q_init ok\n");
	
	return err;
}

/*!
 * ds90ur910q cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit ds90ur910q_clean(void)
{
	dev_dbg(&ds90ur910q_data.sen.i2c_client->dev, "In ds90ur910q_clean\n");
	i2c_del_driver(&ds90ur910q_i2c_driver);
}

module_init(ds90ur910q_init);
module_exit(ds90ur910q_clean);

MODULE_AUTHOR("wynne wang");
MODULE_DESCRIPTION("ds90ur910q video decoder driver");
MODULE_LICENSE("GPL");
