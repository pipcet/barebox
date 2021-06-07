// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Mark Kettenis <kettenis@openbsd.org>
 * Copyright (C) 2021 Pip Cet <pipcet@gmail.com>
 */

#include <asm/io.h>
#include <common.h>
#include <driver.h>
#include <io.h>
#include <of.h>

#define SZ_64K BIT(16)

#define REG_CPU_CTRL	0x0044
#define  REG_CPU_CTRL_RUN	BIT(4)
#define REG_A2I_STAT	0x8110
#define  REG_A2I_STAT_EMPTY	BIT(17)
#define  REG_A2I_STAT_FULL	BIT(16)
#define REG_I2A_STAT	0x8114
#define  REG_I2A_STAT_EMPTY	BIT(17)
#define  REG_I2A_STAT_FULL	BIT(16)
#define REG_A2I_MSG0	0x8800
#define REG_A2I_MSG1	0x8808
#define REG_I2A_MSG0	0x8830
#define REG_I2A_MSG1	0x8838

struct apple_mbox_priv {
	void *base;
	void *mbox[8];
};

unsigned long apple_mbox_phys_start;
unsigned long apple_mbox_phys_addr;
unsigned long apple_mbox_size = SZ_64K;

static void apple_mbox_wait(struct device_d *dev, struct apple_mbox_priv *priv)
{
	int retry;

	retry = 5000000;
	while (--retry && (readl(priv->base + REG_A2I_STAT) & REG_A2I_STAT_FULL))
		udelay(1);
	if (retry == 0)
		dev_err(dev, "%s: A2I timeout\n", __func__);

	retry = 5000000;
	while (--retry && (readl(priv->base + REG_I2A_STAT) & REG_I2A_STAT_EMPTY))
		udelay(1);
	if (retry == 0)
		dev_err(dev, "%s: I2A timeout\n", __func__);
	dev_info(dev, "you have mail %08x %08x\n",
		 readl(priv->base + REG_A2I_STAT),
		 readl(priv->base + REG_I2A_STAT));
}

static int mbox_probe(struct device_d *dev)
{
	struct resource *iores;
	struct apple_mbox_priv *priv = xzalloc(sizeof *priv);
	u64 msg[2];
	int endpoint;
	int endpoints[256];
	int nendpoints = 0;
	int msgtype;
	u64 subtype;
	u32 cpu_ctrl;
	int i, ret;
	void *base;

	iores = dev_request_mem_resource(dev, 0);
	if (IS_ERR(iores))
		return PTR_ERR(iores);

	priv->base = base = IOMEM(iores->start);
	dev_info(dev, "base at %p\n", base);
	if (!priv->base)
		return -EINVAL;

	if (apple_mbox_phys_start == 0) {
		apple_mbox_phys_start = (phys_addr_t)memalign(SZ_64K, SZ_64K);
		apple_mbox_phys_addr = apple_mbox_phys_start;
		apple_mbox_size = SZ_64K;
	}

	dev_info(dev, "mbox at %p/%lx\n", apple_mbox_phys_start, apple_mbox_size);

  wait_hello:
	/* EP0_IDLE */
	cpu_ctrl = readl(priv->base + REG_CPU_CTRL);
	if (cpu_ctrl & REG_CPU_CTRL_RUN) {
		writeq(0x0060000000000220, priv->base + REG_A2I_MSG0);
		writeq(0x0000000000000000, priv->base + REG_A2I_MSG1);
	} else {
		writel(cpu_ctrl | REG_CPU_CTRL_RUN, priv->base + REG_CPU_CTRL);
	}

	mdelay(1000);

	/* EP0_WAIT_HELLO */
	apple_mbox_wait(dev, priv);
	msg[0] = readq(priv->base + REG_I2A_MSG0);
	msg[1] = readq(priv->base + REG_I2A_MSG1);
	dev_info(dev, "message %lx %lx\n", msg[0], msg[1]);

	endpoint = msg[1] & 0xff;
	msgtype = (msg[0] >> 52) & 0xff;
	if (endpoint != 0) {
		dev_err(dev, "%s: unexpected endpoint %d\n", __func__, endpoint);
		return -EINVAL;
	}
	if (msgtype != 1) {
		dev_err(dev, "%s: unexpected message type %d\n", __func__, msgtype);
		return -EINVAL;
	}

	/* EP0_SEND_HELLO */
	subtype = msg[0] & 0xffffffff;
	writeq(0x0020000100000000 | subtype , priv->base + REG_A2I_MSG0);
	writeq(0x0000000000000000, priv->base + REG_A2I_MSG1);

wait_epmap:
	/* EP0_WAIT_EPMAP */
	apple_mbox_wait(dev, priv);
	msg[0] = readq(priv->base + REG_I2A_MSG0);
	msg[1] = readq(priv->base + REG_I2A_MSG1);
	dev_info(dev, "message[2] %lx %lx\n", msg[0], msg[1]);

	endpoint = msg[1] & 0xff;
	msgtype = (msg[0] >> 52) & 0xff;
	if (endpoint != 0) {
		dev_err(dev, "%s: unexpected endpoint %d\n", __func__, endpoint);
		return -EINVAL;
	}
	if (msgtype != 8) {
		dev_err(dev, "%s: unexpected message type %d\n", __func__, msgtype);
		return -EINVAL;
	}

	for (i = 0; i < 32; i++) {
		if (msg[0] & (1U << i)) {
			endpoint = i + 32 * ((msg[0] >> 32) & 7);
			endpoints[nendpoints++] = endpoint;
		}
	}

	/* EP0_SEND_EPACK */
	subtype = (msg[0] >> 32) & 0x80007;
	writeq(0x0080000000000000 | (subtype << 32) | !(subtype & 7), priv->base + REG_A2I_MSG0);
	writeq(0x0000000000000000, priv->base + REG_A2I_MSG1);

	if ((subtype & 0x80000) == 0)
		goto wait_epmap;

	for (i = 0; i < nendpoints; i++) {
		udelay(100);
		while (readl(priv->base + REG_A2I_STAT) & REG_A2I_STAT_FULL)
			udelay(1);

		/* EP0_SEND_EPSTART */
		subtype = endpoints[i];
		writeq(0x0050000000000002 | (subtype << 32), priv->base + REG_A2I_MSG0);
		writeq(0x0000000000000000, priv->base + REG_A2I_MSG1);
	}

wait_pwrok:
	/* EP0_WAIT_PWROK (discard) */
	apple_mbox_wait(dev, priv);
	msg[0] = readq(priv->base + REG_I2A_MSG0);
	msg[1] = readq(priv->base + REG_I2A_MSG1);
	dev_info(dev, "message[3] %lx %lx\n", msg[0], msg[1]);

	endpoint = msg[1] & 0xff;
	msgtype = (msg[0] >> 52) & 0xff;
	if (endpoint == 1 || endpoint == 4) {
		u64 size = (msg[0] >> 44) & 0xff;

		if (apple_mbox_phys_addr + (size << 12) >
		    apple_mbox_phys_start + apple_mbox_size) {
			dev_err(dev, "%s: out of memory\n", __func__);
			return -ENOMEM;
		}

		msg[0] &= ~0xfffffffffff;
		msg[0] |= apple_mbox_phys_addr;
		priv->mbox[endpoint] = apple_mbox_phys_addr;
		dev_info(dev, "size %lx for endpoint %d, mbox at %p\n", size, endpoint, apple_mbox_phys_addr);
		writeq(msg[0], priv->base + REG_A2I_MSG0);
		writeq(msg[1], priv->base + REG_A2I_MSG1);
		apple_mbox_phys_addr += (size << 12);
		goto wait_pwrok;
	}
	if (endpoint != 0) {
		dev_err(dev, "%s: unexpected endpoint %d\n", __func__, endpoint);
		return -EINVAL;
	}
	if (msgtype != 7) {
		dev_err(dev, "%s: unexpected message type %d\n", __func__, msgtype);
		return -EINVAL;
	}

	/* EP0_SEND_PWRACK */
	subtype = msg[0] & 0xffffffff;
	writeq(0x00b0000000000000 | subtype , priv->base + REG_A2I_MSG0);
	writeq(0x0000000000000000, priv->base + REG_A2I_MSG1);

	dev_info(dev, "initialized mbox!\n");

	return 0;
}

static const struct of_device_id mbox_of_match[] = {
	{ .compatible = "apple,iop-mailbox-m1", },
	{ },
};

static struct driver_d mbox_driver = {
	.name = "apple-mbox",
	.of_compatible = mbox_of_match,
	.probe = mbox_probe,
};

device_platform_driver(mbox_driver);

MODULE_AUTHOR("Pip Cet <pipcet@gmail.com>");
MODULE_DESCRIPTION("Apple M1 mailbox driver");
MODULE_LICENSE("GPL v2");
