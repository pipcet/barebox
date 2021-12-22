// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Pip Cet <pipcet@gmail.com>
 */

#include <asm/io.h>
#include <common.h>
#include <driver.h>
#include <io.h>
#include <of.h>

static u64 *alloc_page(void)
{
	u64 *ret = memalign(16384, 16384);

	if (ret == NULL)
		BUG();

	return ret;
}

static u64 dart_l2(struct device_d *dev, u64 addr0)
{
	u64 *ret = alloc_page();
	u64 *p = ret;
	u64 addr;

	for (addr = addr0; addr < addr0 + 2048 * 16384; addr += 16384)
		*p++ = addr | (0xfffLL << 40) | 3;

	return (u64)ret | 3;
}

static u64 *dart_ttbr(struct device_d *dev)
{
	u64 *ret = alloc_page();
	u64 *p = ret;
	u64 addr;
	int i;
	for (addr = 0x800000000ULL; addr < 0x840000000ULL; addr += 2048 * 16384) {
		*p++ = dart_l2(dev, addr);
	}
	for (i = 0; i < 1023; i++) {
		ret[1024 + i] = ret[i];
		ret[2048 + i] = ret[i];
		ret[3072 + i] = ret[i];
	}
	return ret;
}

static int dart_probe(struct device_d *dev)
{
	struct resource *iores;
	int sid;
	static u64 *ttbr;
	ttbr = dart_ttbr(dev);

	iores = dev_request_mem_resource(dev, 0);
	if (IS_ERR(iores))
		PTR_ERR(iores);

	for (sid = 0; sid < 16; sid++) {
		writel(BIT(31)|(((u64)ttbr)>>12), IOMEM(iores->start) + 0x200 + 16 * sid);
		writel(BIT(31)|(((u64)ttbr)>>12), IOMEM(iores->start) + 0x200 + 16 * sid + 4);
		writel(BIT(31)|(((u64)ttbr)>>12), IOMEM(iores->start) + 0x200 + 16 * sid + 8);
		writel(BIT(31)|(((u64)ttbr)>>12), IOMEM(iores->start) + 0x200 + 16 * sid + 12);
		while(readl(IOMEM(iores->start) + 0x20) & BIT(2));
		writel(BIT(7), IOMEM(iores->start) + 0x100 + 4*sid);
		writel(0xffff, IOMEM(iores->start) + 0x34);
		writel(BIT(20), IOMEM(iores->start) + 0x20);
		while(readl(IOMEM(iores->start) + 0x20) & BIT(2));
		while(readl(IOMEM(iores->start) + 0x20) & BIT(2));
		writel(BIT(7), IOMEM(iores->start) + 0x100 + 4*sid);
		writel(BIT(sid), IOMEM(iores->start) + 0x34);
		writel(BIT(20), IOMEM(iores->start) + 0x20);
		while(readl(IOMEM(iores->start) + 0x20) & BIT(2));
	}


	dev_info(dev, "set up dart!\n");

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
