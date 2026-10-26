// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics SALIXP chip-id reader (SPI).
 *
 * SALIXP is the FTS part whose hardware id is 36 40. Google's GPL FTM5
 * driver (kernel/google-modules/touch/fts_touch), Copyright (C) 2016
 * STMicroelectronics, calls that id SALIXP. Chip id 36 48 is ALIX and is
 * not this part. This reader resets the controller, reads the id, and
 * registers an input device. It can decode one 8-byte report. It does
 * not load firmware. Attention is level low and this driver does not
 * request it. After reset is released the line falls and stays low, so
 * a level interrupt would storm before any firmware handshake.
 *
 * Hardware register read, SPI: opcode 0xfb, then the 32-bit address in
 * big-endian, then a one-byte dummy, then the payload. Chip id is at
 * 0x20000000. Chip select stays asserted across the two transfers and
 * for 10 microseconds after the read. The reply half clocks zero bytes
 * on MOSI while it samples MISO, because the Samsung SPI controller
 * shifts a transfer only when it has a TX buffer. The power-cycle path
 * in that driver asserts reset, enables vdd, waits 1 ms, enables avdd,
 * and waits 15 ms before releasing reset.
 */

#include <linux/cache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define SALIXP_CMD_HW_REG_R	0xfb
#define SALIXP_ADDR_DCHIP_ID	0x20000000
#define SALIXP_CS_DELAY_US	10
#define SALIXP_VDD_DELAY_US	1000
#define SALIXP_AVDD_DELAY_US	15000
#define SALIXP_ID_TIMEOUT_MS	500
#define SALIXP_ID_INTERVAL_MS	20
#define SALIXP_MAX_SLOTS	10
#define SALIXP_EVT_TYPE_MASK	0xf3
#define SALIXP_EVT_ENTER	0x13
#define SALIXP_EVT_MOTION	0x23
#define SALIXP_EVT_LEAVE	0x33

struct salixp_info {
	u8 id[2];
};

struct salixp {
	struct spi_device *spi;
	struct gpio_desc *reset;
	struct input_dev *input;
	struct touchscreen_properties prop;
	u8 tx[5] __aligned(ARCH_DMA_MINALIGN);
	u8 reply_tx[3] __aligned(ARCH_DMA_MINALIGN);
	u8 rx[3] __aligned(ARCH_DMA_MINALIGN);
};

static const struct salixp_info salixp_info = {
	.id = { 0x36, 0x40 },
};

static void salixp_assert_reset(void *data)
{
	struct salixp *ts = data;

	/* GPIO_ACTIVE_LOW: logical 1 is physical low, which holds reset. */
	gpiod_set_value_cansleep(ts->reset, 1);
}

static int salixp_read_id(struct salixp *ts, u8 id[2])
{
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ts->tx,
			.len = sizeof(ts->tx),
		},
		{
			.tx_buf = ts->reply_tx,
			.rx_buf = ts->rx,
			.len = sizeof(ts->rx),
			.delay = {
				.value = SALIXP_CS_DELAY_US,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		},
	};
	int ret;

	ts->tx[0] = SALIXP_CMD_HW_REG_R;
	ts->tx[1] = (SALIXP_ADDR_DCHIP_ID >> 24) & 0xff;
	ts->tx[2] = (SALIXP_ADDR_DCHIP_ID >> 16) & 0xff;
	ts->tx[3] = (SALIXP_ADDR_DCHIP_ID >> 8) & 0xff;
	ts->tx[4] = SALIXP_ADDR_DCHIP_ID & 0xff;

	ret = spi_sync_transfer(ts->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	/* First received byte is the SPI dummy. */
	id[0] = ts->rx[1];
	id[1] = ts->rx[2];
	return 0;
}

/*
 * One FIFO report from the felix ftm5 driver. The slot is the high
 * nibble of event[1]. X and Y are packed in event[2..4]. Pressure is
 * event[5]. Enter, motion, and leave are compared after masking the
 * size bits in event[0]. Nothing in this driver calls this until
 * attention is requested, and this driver does not request it.
 */
static void __maybe_unused salixp_report_event(struct salixp *ts,
					       const u8 *event)
{
	u8 type = event[0] & SALIXP_EVT_TYPE_MASK;
	u8 slot = event[1] >> 4;
	unsigned int x, y;

	if (slot >= SALIXP_MAX_SLOTS)
		return;

	input_mt_slot(ts->input, slot);

	if (type == SALIXP_EVT_LEAVE) {
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		input_mt_sync_frame(ts->input);
		input_sync(ts->input);
		return;
	}

	if (type != SALIXP_EVT_ENTER && type != SALIXP_EVT_MOTION)
		return;

	x = ((event[3] & 0x0f) << 8) | event[2];
	y = (event[4] << 4) | ((event[3] & 0xf0) >> 4);

	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
	input_report_abs(ts->input, ABS_MT_PRESSURE, event[5] ? event[5] : 1);
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static int salixp_register_input(struct salixp *ts)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input;
	u32 size;
	int ret;

	if (device_property_read_u32(dev, "touchscreen-size-x", &size) ||
	    device_property_read_u32(dev, "touchscreen-size-y", &size))
		return dev_err_probe(dev, -EINVAL, "missing touchscreen size\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "SALIXP Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	ret = input_mt_init_slots(input, SALIXP_MAX_SLOTS, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ts->input = input;
	return input_register_device(input);
}

static int salixp_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct salixp_info *info;
	struct salixp *ts;
	ktime_t deadline;
	u8 id[2] = {};
	int ret;

	info = device_get_match_data(dev);
	if (!info)
		return -ENODEV;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	/*
	 * Assert reset before either rail. Cover rails are S2MPG12 LDO25M
	 * (1.8 V) and LDO26M (3.3 V). The board leaves them off, with no
	 * regulator-always-on.
	 */
	ts->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset))
		return dev_err_probe(dev, PTR_ERR(ts->reset),
				     "getting reset gpio\n");

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "enabling vdd\n");

	fsleep(SALIXP_VDD_DELAY_US);

	ret = devm_regulator_get_enable(dev, "avdd");
	if (ret)
		return dev_err_probe(dev, ret, "enabling avdd\n");

	/*
	 * Registered after the regulators, so remove asserts reset while
	 * the rails are still on. devm then disables them and releases
	 * the GPIO.
	 */
	ret = devm_add_action_or_reset(dev, salixp_assert_reset, ts);
	if (ret)
		return ret;

	fsleep(SALIXP_AVDD_DELAY_US);
	gpiod_set_value_cansleep(ts->reset, 0);

	deadline = ktime_add_ms(ktime_get(), SALIXP_ID_TIMEOUT_MS);
	do {
		ret = salixp_read_id(ts, id);
		if (!ret && id[0] == info->id[0] && id[1] == info->id[1]) {
			dev_info(dev, "chip id %02x %02x\n", id[0], id[1]);
			return salixp_register_input(ts);
		}
		msleep(SALIXP_ID_INTERVAL_MS);
	} while (ktime_before(ktime_get(), deadline));

	if (ret)
		return dev_err_probe(dev, ret, "reading chip id\n");

	dev_err(dev, "chip id %02x %02x, expected %02x %02x\n",
		id[0], id[1], info->id[0], info->id[1]);
	return -ENODEV;
}

static const struct of_device_id salixp_of_match[] = {
	{ .compatible = "st,salixp", .data = &salixp_info },
	{ }
};
MODULE_DEVICE_TABLE(of, salixp_of_match);

static const struct spi_device_id salixp_id[] = {
	{ "salixp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, salixp_id);

static struct spi_driver salixp_driver = {
	.driver = {
		.name = "salixp",
		.of_match_table = salixp_of_match,
	},
	.probe = salixp_probe,
	.id_table = salixp_id,
};
module_spi_driver(salixp_driver);

MODULE_AUTHOR("David A. Buser <code@booze.dog>");
MODULE_DESCRIPTION("STMicroelectronics SALIXP SPI touchscreen");
MODULE_LICENSE("GPL");
