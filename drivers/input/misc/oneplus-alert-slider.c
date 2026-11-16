// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus alert slider read through two MagnaChip MXM1120 Hall sensors
 * (OnePlus 10 Pro).
 *
 * The slider carries a magnet past two Hall sensors, one near each end.
 * Neither reading alone tells the three positions apart, but the
 * difference of the two does: on the OnePlus 10 Pro, its factory
 * calibration gives up - down = -86 at the top, 5 in the middle and 93 at
 * the bottom.
 *
 * Each sensor raises its interrupt when its reading leaves a window; the
 * driver then reads both, reports the position as ABS_SND_PROFILE (top:
 * silent, middle: vibrate, bottom: ring, as on other OnePlus phones), and
 * moves each window around the new reading.
 *
 * The register map and the settings (10 bit, 80 Hz, a persistence of 15
 * samples) come from OnePlus' GPL hall_mxm1120 driver.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#define MXM1120_PERSINT			0x00
#define MXM1120_PERSINT_PERS(n)		((n) << 4)
#define MXM1120_PERSINT_INTCLR		BIT(0)
#define MXM1120_INTSRS			0x01
#define MXM1120_INTSRS_INTON		BIT(7)
#define MXM1120_LTHL			0x02
#define MXM1120_LTHH			0x03
#define MXM1120_HTHL			0x04
#define MXM1120_HTHH			0x05
#define MXM1120_SRST			0x07
#define MXM1120_SRST_RESET		BIT(0)
#define MXM1120_OPF			0x08
#define MXM1120_OPF_FREQ_80HZ		(4 << 4)
#define MXM1120_OPF_HSSON		BIT(0)
#define MXM1120_DID			0x09
#define MXM1120_DID_VALUE		0x9c
#define MXM1120_ST1			0x10
#define MXM1120_ST1_DRDY		BIT(0)

/* 10-bit two's complement: the low byte, then bits 9:8 in bits 7:6 */
#define MXM1120_HIGH_BITS		GENMASK(7, 6)

/* A persistence of 15 samples at 80 Hz: about 190 ms of debounce */
#define MXM1120_PERSISTENCE		15

/* Each window: the last reading, plus or minus this much */
#define SLIDER_WINDOW			22

/*
 * Boundaries for up - down, about halfway between the calibrated
 * positions (-86, 5, 93), so a few tens of counts of drift between units
 * still lands on the right side.
 */
#define SLIDER_TOP_BELOW		-40
#define SLIDER_BOTTOM_ABOVE		50

enum { SLIDER_UP, SLIDER_DOWN, SLIDER_SENSORS };

static const char * const slider_names[SLIDER_SENSORS] = { "up", "down" };

static const char * const slider_supplies[] = { "vdd", "vio" };

struct slider {
	struct i2c_client *hall[SLIDER_SENSORS];
	struct input_dev *input;
	/* serializes the two interrupt threads and the sysfs reader */
	struct mutex lock;
	s16 value[SLIDER_SENSORS];
};

/*
 * Reading the data clears DRDY until the next sample, 12.5 ms later at
 * 80 Hz: both sensors interrupt on one move, and the second interrupt's
 * read can come before it. Wait for that sample then.
 */
static int mxm1120_read(struct i2c_client *client, s16 *value)
{
	u8 buf[3];
	int ret, tries;

	for (tries = 0; tries < 3; tries++) {
		ret = i2c_smbus_read_i2c_block_data(client, MXM1120_ST1,
						    sizeof(buf), buf);
		if (ret < 0)
			return ret;
		if (ret != sizeof(buf))
			return -EIO;
		if (buf[0] & MXM1120_ST1_DRDY) {
			*value = sign_extend32(FIELD_GET(MXM1120_HIGH_BITS, buf[2]) << 8 |
					       buf[1], 9);
			return 0;
		}
		usleep_range(13000, 15000);
	}

	return -EAGAIN;
}

static int mxm1120_write_value(struct i2c_client *client, u8 reg_low, s16 value)
{
	u16 raw = clamp_t(s16, value, -512, 511) & GENMASK(9, 0);
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg_low + 1,
					FIELD_PREP(MXM1120_HIGH_BITS, raw >> 8));
	if (ret)
		return ret;

	return i2c_smbus_write_byte_data(client, reg_low, raw & 0xff);
}

/* Interrupt again once the reading leaves value +/- SLIDER_WINDOW */
static int mxm1120_arm(struct i2c_client *client, s16 value)
{
	int ret;

	ret = mxm1120_write_value(client, MXM1120_HTHL, value + SLIDER_WINDOW);
	if (ret)
		return ret;

	ret = mxm1120_write_value(client, MXM1120_LTHL, value - SLIDER_WINDOW);
	if (ret)
		return ret;

	return i2c_smbus_write_byte_data(client, MXM1120_PERSINT,
					 MXM1120_PERSINT_PERS(MXM1120_PERSISTENCE) |
					 MXM1120_PERSINT_INTCLR);
}

static int mxm1120_init(struct i2c_client *client)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, MXM1120_SRST, MXM1120_SRST_RESET);
	if (ret)
		return ret;
	msleep(20);

	ret = i2c_smbus_read_byte_data(client, MXM1120_DID);
	if (ret < 0)
		return ret;
	if (ret != MXM1120_DID_VALUE) {
		dev_err(&client->dev, "unexpected device ID 0x%02x\n", ret);
		return -ENODEV;
	}

	ret = i2c_smbus_write_byte_data(client, MXM1120_PERSINT,
					MXM1120_PERSINT_PERS(MXM1120_PERSISTENCE));
	if (ret)
		return ret;

	/* interrupt when the reading is outside the window, 0.068 mT/LSB */
	ret = i2c_smbus_write_byte_data(client, MXM1120_INTSRS, MXM1120_INTSRS_INTON);
	if (ret)
		return ret;

	/* 80 Hz, 10 bit, measuring */
	return i2c_smbus_write_byte_data(client, MXM1120_OPF,
					 MXM1120_OPF_FREQ_80HZ | MXM1120_OPF_HSSON);
}

static int slider_profile(s16 up, s16 down)
{
	int diff = up - down;

	if (diff < SLIDER_TOP_BELOW)
		return SND_PROFILE_SILENT;
	if (diff > SLIDER_BOTTOM_ABOVE)
		return SND_PROFILE_RING;
	return SND_PROFILE_VIBRATE;
}

/* Read both sensors, report the position and re-arm both windows */
static int slider_update(struct slider *slider)
{
	int i, ret;

	guard(mutex)(&slider->lock);

	for (i = 0; i < SLIDER_SENSORS; i++) {
		ret = mxm1120_read(slider->hall[i], &slider->value[i]);
		if (ret)
			return ret;
	}

	input_report_abs(slider->input, ABS_SND_PROFILE,
			 slider_profile(slider->value[SLIDER_UP],
					slider->value[SLIDER_DOWN]));
	input_sync(slider->input);

	for (i = 0; i < SLIDER_SENSORS; i++) {
		ret = mxm1120_arm(slider->hall[i], slider->value[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static irqreturn_t slider_irq(int irq, void *data)
{
	struct slider *slider = data;
	int ret;

	ret = slider_update(slider);
	if (ret)
		dev_err_ratelimited(&slider->hall[SLIDER_UP]->dev,
				    "failed to read the sensors: %d\n", ret);

	return IRQ_HANDLED;
}

/* The two readings and their difference, for calibration */
static ssize_t hall_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct slider *slider = dev_get_drvdata(dev);
	s16 value[SLIDER_SENSORS];
	int i, ret;

	scoped_guard(mutex, &slider->lock) {
		for (i = 0; i < SLIDER_SENSORS; i++) {
			ret = mxm1120_read(slider->hall[i], &value[i]);
			if (ret)
				return ret;
		}
	}

	return sysfs_emit(buf, "%d %d %d\n", value[SLIDER_UP], value[SLIDER_DOWN],
			  value[SLIDER_UP] - value[SLIDER_DOWN]);
}
static DEVICE_ATTR_RO(hall);

static struct attribute *slider_attrs[] = {
	&dev_attr_hall.attr,
	NULL
};
ATTRIBUTE_GROUPS(slider);

static void slider_unregister(void *data)
{
	i2c_unregister_device(data);
}

static int slider_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct slider *slider;
	int i, irq, ret;

	slider = devm_kzalloc(dev, sizeof(*slider), GFP_KERNEL);
	if (!slider)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &slider->lock);
	if (ret)
		return ret;

	/* the "up" sensor is the node's first address; "down" its second */
	slider->hall[SLIDER_UP] = client;
	slider->hall[SLIDER_DOWN] = i2c_new_ancillary_device(client, "down", 0x0c);
	if (IS_ERR(slider->hall[SLIDER_DOWN]))
		return dev_err_probe(dev, PTR_ERR(slider->hall[SLIDER_DOWN]),
				     "failed to add the down sensor\n");
	ret = devm_add_action_or_reset(dev, slider_unregister,
				       slider->hall[SLIDER_DOWN]);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(slider_supplies),
					     slider_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");
	usleep_range(10000, 11000);

	for (i = 0; i < SLIDER_SENSORS; i++) {
		ret = mxm1120_init(slider->hall[i]);
		if (ret)
			return dev_err_probe(dev, ret, "failed to set up the %s sensor\n",
					     slider_names[i]);
	}
	/* the first measurement, at 80 Hz */
	msleep(20);

	slider->input = devm_input_allocate_device(dev);
	if (!slider->input)
		return -ENOMEM;

	slider->input->name = "Alert slider";
	slider->input->id.bustype = BUS_I2C;
	input_set_abs_params(slider->input, ABS_SND_PROFILE,
			     SND_PROFILE_SILENT, SND_PROFILE_RING, 0, 0);

	ret = input_register_device(slider->input);
	if (ret)
		return ret;

	i2c_set_clientdata(client, slider);

	ret = slider_update(slider);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the sensors\n");

	for (i = 0; i < SLIDER_SENSORS; i++) {
		irq = fwnode_irq_get_byname(dev_fwnode(dev), slider_names[i]);
		if (irq < 0)
			return dev_err_probe(dev, irq, "no %s interrupt\n",
					     slider_names[i]);

		ret = devm_request_threaded_irq(dev, irq, NULL, slider_irq,
						IRQF_ONESHOT, slider_names[i], slider);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request the %s interrupt\n",
					     slider_names[i]);
	}

	return 0;
}

static const struct of_device_id slider_of_match[] = {
	{ .compatible = "oneplus,mxm1120-alert-slider" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, slider_of_match);

static struct i2c_driver slider_driver = {
	.probe = slider_probe,
	.driver = {
		.name = "oneplus-alert-slider",
		.of_match_table = slider_of_match,
		.dev_groups = slider_groups,
	},
};
module_i2c_driver(slider_driver);

MODULE_DESCRIPTION("OnePlus alert slider on two MagnaChip MXM1120 Hall sensors");
MODULE_LICENSE("GPL");
