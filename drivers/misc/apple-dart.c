// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Pip Cet <pipcet@gmail.com>
 */

#include <asm/io.h>
#include <common.h>
#include <driver.h>
#include <io.h>
#include <of.h>

static int dart_probe(struct device_d *dev)
{
	struct resource *iores;
	int sid;
	iores = dev_request_mem_resource(dev, 0);
	if (!IS_ERR(iores))
		for (sid = 0; sid < 16; sid++)
			writel(BIT(8) | BIT(12), IOMEM(iores->start) + 0x100 + 4*sid);
	iores = dev_request_mem_resource(dev, 1);
	if (!IS_ERR(iores))
		for (sid = 0; sid < 16; sid++)
			writel(BIT(8) | BIT(12), IOMEM(iores->start) + 0x100 + 4*sid);

	dev_info(dev, "put dart into bypass mode!\n");

	return 0;
}

static const struct of_device_id dart_of_match[] = {
	{ .compatible = "apple,dart-m1", },
	{ },
};

static struct driver_d dart_driver = {
	.name = "apple-dart",
	.of_compatible = dart_of_match,
	.probe = dart_probe,
};

device_platform_driver(dart_driver);

MODULE_AUTHOR("Pip Cet <pipcet@gmail.com>");
MODULE_DESCRIPTION("Apple M1 DART driver");
MODULE_LICENSE("GPL v2");
