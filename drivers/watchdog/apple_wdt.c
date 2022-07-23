// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Mark Kettenis <kettenis@openbsd.org>
 */

#include <asm/io.h>
#include <common.h>
#include <driver.h>
#include <watchdog.h>
#include <superio.h>

#define APPLE_WDT_CUR_TIME		0x10
#define APPLE_WDT_BARK_TIME		0x14
#define APPLE_WDT_CTRL			0x1c
#define  APPLE_WDT_CTRL_RESET_EN	BIT(2)

struct apple_wdt_priv {
	void *base;
	//struct restart_handler restart;
	ulong clk_rate;
};

static int apple_wdt_reset(struct udevice *dev)
{
	struct apple_wdt_priv *priv = dev_get_priv(dev);

	writel(0, priv->base + APPLE_WDT_CUR_TIME);

	return 0;
}

static int apple_wdt_start(struct udevice *dev, u64 timeout_ms, ulong flags)
{
	struct apple_wdt_priv *priv = dev_get_priv(dev);
	u64 timeout;

	timeout = (timeout_ms * priv->clk_rate) / 1000;
	if (timeout > U32_MAX)
		return -EINVAL;

	writel(0, priv->base + APPLE_WDT_CUR_TIME);
	writel(timeout, priv->base + APPLE_WDT_BARK_TIME);
	writel(APPLE_WDT_CTRL_RESET_EN, priv->base + APPLE_WDT_CTRL);

	return 0;
}

static int apple_wdt_stop(struct udevice *dev)
{
	struct apple_wdt_priv *priv = dev_get_priv(dev);

	writel(0, priv->base + APPLE_WDT_CTRL);

	return 0;
}

static int apple_wdt_expire_now(struct udevice *dev, ulong flags)
{
	int ret;

	ret = apple_wdt_start(dev, 0, flags);
	if (ret)
		return ret;

	/*
	 * It can take up to 25ms until the SoC actually resets, so
	 * wait 50ms just to be sure.
	 */
	mdelay(50);

	return 0;
}

static int apple_wdt_probe(struct device_d *dev)
{
	struct apple_wdt_priv *priv = dev_get_priv(dev);
	int ret;

	priv->base = dev_request_mem_resource(dev, 0)->start;
	if (!priv->base)
		return -EINVAL;

	priv->clk_rate = 24 * 1000 * 1000;

	return 0;
}

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for watchdog device controlled through GPIO-line
 *
 * Author: 2013, Alexander Shiyan <shc_work@mail.ru>
 */


enum {
	HW_ALGO_TOGGLE,
	HW_ALGO_LEVEL,
};

struct gpio_wdt_priv {
	int		gpio;
	bool		state;
	bool		started;
	unsigned int	hw_algo;
	struct watchdog	wdd;
};

static inline struct gpio_wdt_priv *to_gpio_wdt_priv(struct watchdog *wdd)
{
	return container_of(wdd, struct gpio_wdt_priv, wdd);
}

static void gpio_wdt_disable(struct gpio_wdt_priv *priv)
{
}

static void gpio_wdt_ping(struct gpio_wdt_priv *priv)
{
}

static int gpio_wdt_set_timeout(struct watchdog *wdd, unsigned int new_timeout)
{
	struct gpio_wdt_priv *priv = to_gpio_wdt_priv(wdd);

	if (!new_timeout) {
		gpio_wdt_disable(priv);
		return 0;
	}

	gpio_wdt_ping(priv);
	return 0;
}

static const struct of_device_id apple_wdt_ids[] = {
	{ .compatible = "apple,wdt", },
	{ }
};

static struct driver_d apple_wdt_driver = {
	.name = "apple_wdt",
	.of_compatible = apple_wdt_ids,
	.probe = apple_wdt_probe,
};
device_platform_driver(apple_wdt_driver);

MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("GPIO Watchdog");
MODULE_LICENSE("GPL");
