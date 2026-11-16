// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3HC3 DDIC with the AMB670YF07 command-mode AMOLED panel
 * (OnePlus 10 Pro), driven at its native 1440x3216 and 120 Hz with DSC.
 *
 * The init sequence, timings and DSC slice layout come from the stock
 * vendor devicetree (qcom,mdss_dsi_panel_samsung_amb670yf07_1440_3216_dsc_cmd,
 * its 1440x3216 120 Hz timing). Stock's 1080x2412 timings send the same
 * sequence with the DDIC's scaler (0xc3) on.
 *
 * The panel scans on a 120 Hz grid in every mode: at 60 Hz, stock's
 * sequence leaves its adaptive refresh on, and TE comes every second slot
 * or every slot. With the DSI link clocked for 60 Hz a frame takes a whole
 * 60 Hz period to send, misses the next slot and lands on the third (40
 * fps). Clocked for 120 Hz, a frame fits in one slot and TE comes on every
 * slot.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* Normal range; stock goes up to 4095 in high brightness mode (HBM). */
#define AMB670YF07_MAX_BRIGHTNESS	2047
#define AMB670YF07_DEFAULT_BRIGHTNESS	400

/* Samsung's DCS commands for the DSC picture parameter set and on/off */
#define AMB670YF07_DCS_PPS		0x9e
#define AMB670YF07_DCS_COMPRESSION	0x9d

struct amb670yf07 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator_bulk_data *supplies;
};

static const struct regulator_bulk_data amb670yf07_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vci" },
};

static inline struct amb670yf07 *to_amb670yf07(struct drm_panel *panel)
{
	return container_of(panel, struct amb670yf07, panel);
}

static void amb670yf07_reset(struct amb670yf07 *ctx)
{
	/* stock's qcom,mdss-dsi-reset-sequence: high 10 ms, low 5 ms, high 10 ms */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int amb670yf07_on(struct amb670yf07 *priv)
{
	struct mipi_dsi_device *dsi = priv->dsi;
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };
	u8 pps[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/*
	 * Stock sends a fixed PPS through Samsung's 0x9e command. Send the
	 * one packed from the encoder's own configuration instead, so the
	 * panel's decoder always matches what DPU encodes.
	 */
	pps[0] = AMB670YF07_DCS_PPS;
	drm_dsc_pps_payload_pack((struct drm_dsc_picture_parameter_set *)&pps[1],
				 &priv->dsc);
	dev_dbg(&dsi->dev, "PPS %*ph\n", 64, &pps[1]);
	dev_dbg(&dsi->dev, "PPS %*ph\n", 64, &pps[65]);
	mipi_dsi_dcs_write_buffer_multi(&ctx, pps, sizeof(pps));
	mipi_dsi_dcs_write_seq_multi(&ctx, AMB670YF07_DCS_COMPRESSION, 0x01);

	mipi_dsi_dcs_exit_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 6);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x22, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0xa1, 0xb1);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x3a, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x02);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x26, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x24, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x21);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x38, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x02);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x2a, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_msleep(&ctx, 121);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x16, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf2, 0x1b, 0x50);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x08, 0xcb);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xcb, 0x24);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x60, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x23, 0x02);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x10, 0xbd);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x16, 0xbd);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x77);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x14, 0xbd);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf2, 0x00, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x2a, 0x00, 0x00, 0x05, 0x9f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x2b, 0x00, 0x00, 0x0c, 0x8f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	/* the DDIC's scaler off: the panel is native 1440x3216 */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xc3, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x2b, 0xf6);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf6, 0x60, 0x63, 0x69);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x46, 0xf4);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf4, 0x08);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x18, 0xb1);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb1, 0x05);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x0d, 0x63);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x63, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x0c, 0x63);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x63, 0x20);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x53, 0x28);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x55, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x52, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x1f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x00, 0x54, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x1f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf0, 0xa5, 0xa5);

	return ctx.accum_err;
}

static int amb670yf07_prepare(struct drm_panel *panel)
{
	struct amb670yf07 *ctx = to_amb670yf07(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(amb670yf07_supplies),
				    ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(ctx->enable_gpio, 1);
	usleep_range(10000, 11000);

	amb670yf07_reset(ctx);

	ret = amb670yf07_on(ctx);
	if (ret < 0) {
		dev_err(dev, "failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		gpiod_set_value_cansleep(ctx->enable_gpio, 0);
		regulator_bulk_disable(ARRAY_SIZE(amb670yf07_supplies),
				       ctx->supplies);
		return ret;
	}

	return 0;
}

static int amb670yf07_enable(struct drm_panel *panel)
{
	struct amb670yf07 *ctx = to_amb670yf07(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* stock: display on right after the first backlight update */
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int amb670yf07_disable(struct drm_panel *panel)
{
	struct amb670yf07 *ctx = to_amb670yf07(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	/* stock's qcom,mdss-dsi-off-command */
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 21);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 101);

	return dsi_ctx.accum_err;
}

static int amb670yf07_unprepare(struct drm_panel *panel)
{
	struct amb670yf07 *ctx = to_amb670yf07(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	gpiod_set_value_cansleep(ctx->enable_gpio, 0);
	regulator_bulk_disable(ARRAY_SIZE(amb670yf07_supplies), ctx->supplies);

	return 0;
}

/* stock's timing@wqhd_sdc_120 */
static const struct drm_display_mode amb670yf07_mode = {
	.clock = (1440 + 88 + 8 + 57) * (3216 + 8 + 8 + 8) * 120 / 1000,
	.hdisplay = 1440,
	.hsync_start = 1440 + 88,
	.hsync_end = 1440 + 88 + 8,
	.htotal = 1440 + 88 + 8 + 57,
	.vdisplay = 3216,
	.vsync_start = 3216 + 8,
	.vsync_end = 3216 + 8 + 8,
	.vtotal = 3216 + 8 + 8 + 8,
	.width_mm = 70,
	.height_mm = 156,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int amb670yf07_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &amb670yf07_mode);
}

static const struct drm_panel_funcs amb670yf07_panel_funcs = {
	.prepare = amb670yf07_prepare,
	.enable = amb670yf07_enable,
	.disable = amb670yf07_disable,
	.unprepare = amb670yf07_unprepare,
	.get_modes = amb670yf07_get_modes,
};

static int amb670yf07_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	/* DBV, most significant byte first (stock: bl-inverted-dbv) */
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops amb670yf07_bl_ops = {
	.update_status = amb670yf07_bl_update_status,
};

static struct backlight_device *
amb670yf07_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = AMB670YF07_DEFAULT_BRIGHTNESS,
		.max_brightness = AMB670YF07_MAX_BRIGHTNESS,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &amb670yf07_bl_ops, &props);
}

static int amb670yf07_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct amb670yf07 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct amb670yf07, panel,
				   &amb670yf07_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(amb670yf07_supplies),
					    amb670yf07_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio),
				     "failed to get enable-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	/* stock: qcom,mdss-dsc-slice-per-pkt = <2> */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = amb670yf07_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 24;
	ctx->dsc.slice_width = 720;
	ctx->dsc.slice_count = 1440 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void amb670yf07_remove(struct mipi_dsi_device *dsi)
{
	struct amb670yf07 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id amb670yf07_of_match[] = {
	{ .compatible = "samsung,s6e3hc3-amb670yf07" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, amb670yf07_of_match);

static struct mipi_dsi_driver amb670yf07_driver = {
	.probe = amb670yf07_probe,
	.remove = amb670yf07_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc3-amb670yf07",
		.of_match_table = amb670yf07_of_match,
	},
};
module_mipi_dsi_driver(amb670yf07_driver);

MODULE_DESCRIPTION("DRM driver for the Samsung S6E3HC3 AMB670YF07 panel");
MODULE_LICENSE("GPL");
