// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus (OPLUS) OTG handshake with the ADSP's battery charger, over
 * pmic_glink.
 *
 * On OPLUS phones such as the OnePlus 10 Pro (wly), the ADSP only reports a
 * USB device plugged into the USB-C port (and asks for OTG power) after the
 * application processor has told it, once per ADSP boot, that it handles
 * OTG: the vendor USB property USB_OTG_AP_ENABLE. The ADSP then sends
 * BC_CID_DETECT on insertion and BC_OTG_ENABLE / BC_OTG_DISABLE, and the
 * vendor kernel switches an external boost converter accordingly.
 *
 * At start-up it checks the property numbering by reading a few properties,
 * then sets USB_OTG_AP_ENABLE. On BC_OTG_ENABLE it powers the port the way
 * the vendor kernel does (captured on stock): it isolates the wireless
 * receiver (wrx-en), waits 50 ms, then opens the OVP switch (ovp-en) and
 * starts the boost converter (boost-en); BC_OTG_DISABLE, an ADSP restart,
 * removal and shutdown undo it in reverse order.
 *
 * Property ids and notification codes come from OnePlus's GPL charger driver
 * (vendor/oplus/kernel/charger/charger_ic/oplus_battery_sm8450.h).
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/workqueue.h>

#define BC_NOTIFY		0x07
#define BC_USB_STATUS_GET	0x32
#define BC_USB_STATUS_SET	0x33

/* OPLUS notifications */
#define BC_OTG_ENABLE		0x50
#define BC_OTG_DISABLE		0x51
#define BC_CID_DETECT		0x53
#define BC_TYPEC_STATE_CHANGE	0x55
#define BC_PLUGIN_IRQ		0x57

/* OPLUS USB property ids (enum usb_property_id, OPLUS_FEATURE_CHG_BASIC) */
#define USB_OTG_AP_ENABLE	14
#define USB_OTG_SWITCH		15
#define USB_TYPEC_CC_ORIENTATION 17
#define USB_CID_STATUS		18
#define USB_TYPEC_MODE		19

#define OPLUS_OTG_TIMEOUT	msecs_to_jiffies(2000)

struct oplus_otg_req {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

struct oplus_otg_resp {
	struct pmic_glink_hdr hdr;
	__le32 property;
	__le32 value;
	__le32 result;
};

struct oplus_otg_notify {
	struct pmic_glink_hdr hdr;
	__le32 notification;
};

/* Handed from the DT device to the pmic_glink client it creates */
struct oplus_otg_gpios {
	struct gpio_desc *wrx;
	struct gpio_desc *ovp;
	struct gpio_desc *boost;
};

struct oplus_otg {
	struct device *dev;
	struct oplus_otg_gpios gpios;
	struct work_struct otg_work;
	struct mutex otg_lock;		/* serialises power changes */
	bool otg_want;			/* what the ADSP last asked for */
	bool otg_on;			/* what the GPIOs are set to */
	struct pmic_glink_client *client;
	struct work_struct init_work;
	struct mutex lock;		/* one request in flight */
	struct completion ack;
	unsigned int pending_opcode;
	unsigned int pending_property;
	u32 value;
	int result;
};

static int oplus_otg_request(struct oplus_otg *otg, unsigned int opcode,
			     unsigned int property, u32 value, u32 *out)
{
	struct oplus_otg_req req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};
	int ret;

	mutex_lock(&otg->lock);
	reinit_completion(&otg->ack);
	otg->pending_opcode = opcode;
	otg->pending_property = property;
	otg->result = -ETIMEDOUT;

	ret = pmic_glink_send(otg->client, &req, sizeof(req));
	if (!ret && !wait_for_completion_timeout(&otg->ack, OPLUS_OTG_TIMEOUT))
		ret = -ETIMEDOUT;
	if (!ret)
		ret = otg->result;
	if (!ret && out)
		*out = otg->value;

	otg->pending_opcode = 0;
	mutex_unlock(&otg->lock);
	return ret;
}

static void oplus_otg_read(struct oplus_otg *otg, unsigned int property,
			   const char *name)
{
	u32 val;
	int ret = oplus_otg_request(otg, BC_USB_STATUS_GET, property, 0, &val);

	if (ret)
		dev_info(otg->dev, "read %s (%u): error %d\n", name, property, ret);
	else
		dev_info(otg->dev, "read %s (%u) = %u\n", name, property, val);
}

static void oplus_otg_init_work(struct work_struct *work)
{
	struct oplus_otg *otg = container_of(work, struct oplus_otg, init_work);
	int ret;

	/* Known on stock with a cable to a PC: otg_switch 0, orientation 1 or 2 */
	oplus_otg_read(otg, USB_OTG_SWITCH, "otg_switch");
	oplus_otg_read(otg, USB_TYPEC_CC_ORIENTATION, "cc_orientation");
	oplus_otg_read(otg, USB_CID_STATUS, "cid_status");
	oplus_otg_read(otg, USB_TYPEC_MODE, "typec_mode");
	oplus_otg_read(otg, USB_OTG_AP_ENABLE, "otg_ap_enable");

	ret = oplus_otg_request(otg, BC_USB_STATUS_SET, USB_OTG_AP_ENABLE, 1, NULL);
	dev_info(otg->dev, "set otg_ap_enable = 1: %d\n", ret);

	oplus_otg_read(otg, USB_OTG_AP_ENABLE, "otg_ap_enable");
}

static void oplus_otg_power(struct oplus_otg *otg, bool on)
{
	struct oplus_otg_gpios *g = &otg->gpios;

	mutex_lock(&otg->otg_lock);
	if (on == otg->otg_on)
		goto out;

	if (on) {
		gpiod_set_value_cansleep(g->wrx, 1);
		msleep(50);
		gpiod_set_value_cansleep(g->ovp, 1);
		gpiod_set_value_cansleep(g->boost, 1);
	} else {
		gpiod_set_value_cansleep(g->ovp, 0);
		gpiod_set_value_cansleep(g->boost, 0);
		msleep(50);
		gpiod_set_value_cansleep(g->wrx, 0);
	}
	otg->otg_on = on;
	dev_info(otg->dev, "OTG power %s\n", on ? "on" : "off");
out:
	mutex_unlock(&otg->otg_lock);
}

static void oplus_otg_work(struct work_struct *work)
{
	struct oplus_otg *otg = container_of(work, struct oplus_otg, otg_work);

	oplus_otg_power(otg, READ_ONCE(otg->otg_want));
}

static const char *oplus_otg_name(unsigned int code)
{
	switch (code) {
	case BC_OTG_ENABLE:		return "OTG_ENABLE";
	case BC_OTG_DISABLE:		return "OTG_DISABLE";
	case BC_CID_DETECT:		return "CID_DETECT";
	case BC_TYPEC_STATE_CHANGE:	return "TYPEC_STATE_CHANGE";
	case BC_PLUGIN_IRQ:		return "PLUGIN_IRQ";
	default:			return NULL;
	}
}

static void oplus_otg_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct oplus_otg *otg = priv;
	unsigned int opcode = le32_to_cpu(hdr->opcode);

	if (opcode == BC_NOTIFY && len >= sizeof(struct oplus_otg_notify)) {
		const struct oplus_otg_notify *n = data;
		unsigned int code = le32_to_cpu(n->notification);
		const char *name = oplus_otg_name(code & 0xff);

		if (name)
			dev_info(otg->dev, "ADSP: %s (%#x)\n", name, code);
		else if ((code & 0xff) >= 0x50 && (code & 0xff) <= 0x7f)
			dev_info(otg->dev, "ADSP: notification %#x (len %zu)\n",
				 code, len);

		/* Called in atomic context; the PMIC GPIO may sleep */
		if ((code & 0xff) == BC_OTG_ENABLE || (code & 0xff) == BC_OTG_DISABLE) {
			WRITE_ONCE(otg->otg_want, (code & 0xff) == BC_OTG_ENABLE);
			schedule_work(&otg->otg_work);
		}
		return;
	}

	if ((opcode == BC_USB_STATUS_GET || opcode == BC_USB_STATUS_SET) &&
	    len == sizeof(struct oplus_otg_resp) && opcode == otg->pending_opcode) {
		const struct oplus_otg_resp *r = data;

		if (le32_to_cpu(r->property) != otg->pending_property)
			return;
		otg->value = le32_to_cpu(r->value);
		otg->result = le32_to_cpu(r->result) ? -EIO : 0;
		complete(&otg->ack);
	}
}

static void oplus_otg_pdr(void *priv, int state)
{
	struct oplus_otg *otg = priv;

	if (state == SERVREG_SERVICE_STATE_UP) {
		schedule_work(&otg->init_work);
	} else {
		/* ADSP gone: nobody to stop the boost, so stop it now */
		WRITE_ONCE(otg->otg_want, false);
		schedule_work(&otg->otg_work);
	}
}

/*
 * A pmic_glink client must be a child of the pmic_glink device:
 * devm_pmic_glink_client_alloc() takes the pmic_glink from its parent's
 * driver data. The DT node (a root node, pointing at pmic-glink) therefore
 * only creates such a child; the client driver below does the work.
 */
#define OPLUS_OTG_CLIENT	"oplus-otg-glink-client"

static int oplus_otg_client_probe(struct platform_device *pdev)
{
	const struct oplus_otg_gpios *gpios = dev_get_platdata(&pdev->dev);
	struct oplus_otg *otg;

	if (!gpios)
		return -EINVAL;

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	otg->dev = &pdev->dev;
	otg->gpios = *gpios;
	mutex_init(&otg->lock);
	mutex_init(&otg->otg_lock);
	INIT_WORK(&otg->otg_work, oplus_otg_work);
	init_completion(&otg->ack);
	INIT_WORK(&otg->init_work, oplus_otg_init_work);

	otg->client = devm_pmic_glink_client_alloc(&pdev->dev,
						   PMIC_GLINK_OWNER_BATTMGR,
						   oplus_otg_callback,
						   oplus_otg_pdr, otg);
	if (IS_ERR(otg->client))
		return dev_err_probe(&pdev->dev, PTR_ERR(otg->client),
				     "failed to allocate pmic_glink client\n");

	platform_set_drvdata(pdev, otg);
	pmic_glink_client_register(otg->client);
	return 0;
}

static void oplus_otg_client_remove(struct platform_device *pdev)
{
	struct oplus_otg *otg = platform_get_drvdata(pdev);

	if (!otg)
		return;
	cancel_work_sync(&otg->init_work);
	cancel_work_sync(&otg->otg_work);
	oplus_otg_power(otg, false);
}

static void oplus_otg_client_shutdown(struct platform_device *pdev)
{
	struct oplus_otg *otg = platform_get_drvdata(pdev);

	if (otg)
		oplus_otg_power(otg, false);
}

static struct platform_driver oplus_otg_client_driver = {
	.probe = oplus_otg_client_probe,
	.remove = oplus_otg_client_remove,
	.shutdown = oplus_otg_client_shutdown,
	.driver = {
		.name = OPLUS_OTG_CLIENT,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

static void oplus_otg_unregister(void *data)
{
	platform_device_unregister(data);
}

static void oplus_otg_put(void *data)
{
	put_device(data);
}

static int oplus_otg_probe(struct platform_device *pdev)
{
	struct platform_device *pg, *child;
	struct oplus_otg_gpios gpios;
	struct device_node *np;
	int ret;

	/* All off, as stock leaves them at boot */
	gpios.wrx = devm_gpiod_get(&pdev->dev, "wrx-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpios.wrx))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpios.wrx), "wrx-en\n");
	gpios.ovp = devm_gpiod_get(&pdev->dev, "ovp-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpios.ovp))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpios.ovp), "ovp-en\n");
	gpios.boost = devm_gpiod_get(&pdev->dev, "boost-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpios.boost))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpios.boost), "boost-en\n");

	np = of_parse_phandle(pdev->dev.of_node, "qcom,pmic-glink", 0);
	if (!np)
		return dev_err_probe(&pdev->dev, -EINVAL, "no qcom,pmic-glink\n");
	pg = of_find_device_by_node(np);
	of_node_put(np);
	if (!pg)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(&pdev->dev, oplus_otg_put, &pg->dev);
	if (ret)
		return ret;

	/* pmic_glink sets its driver data in probe; wait until it is bound */
	if (!device_is_bound(&pg->dev) || !dev_get_drvdata(&pg->dev))
		return -EPROBE_DEFER;

	child = platform_device_register_data(&pg->dev, OPLUS_OTG_CLIENT,
					      PLATFORM_DEVID_NONE,
					      &gpios, sizeof(gpios));
	if (IS_ERR(child))
		return PTR_ERR(child);

	return devm_add_action_or_reset(&pdev->dev, oplus_otg_unregister, child);
}

static const struct of_device_id oplus_otg_of_match[] = {
	{ .compatible = "oneplus,otg-glink" },
	{}
};
MODULE_DEVICE_TABLE(of, oplus_otg_of_match);

static struct platform_driver oplus_otg_driver = {
	.probe = oplus_otg_probe,
	.driver = {
		.name = "oplus-otg-glink",
		.of_match_table = oplus_otg_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

static int __init oplus_otg_init(void)
{
	int ret = platform_driver_register(&oplus_otg_client_driver);

	if (ret)
		return ret;
	ret = platform_driver_register(&oplus_otg_driver);
	if (ret)
		platform_driver_unregister(&oplus_otg_client_driver);
	return ret;
}
module_init(oplus_otg_init);

static void __exit oplus_otg_exit(void)
{
	platform_driver_unregister(&oplus_otg_driver);
	platform_driver_unregister(&oplus_otg_client_driver);
}
module_exit(oplus_otg_exit);

MODULE_DESCRIPTION("OnePlus OTG handshake with the ADSP over pmic_glink");
MODULE_LICENSE("GPL");
