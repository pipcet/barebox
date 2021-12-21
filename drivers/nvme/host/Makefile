# SPDX-License-Identifier: GPL-2.0-only

ccflags-y				+= -I$(src)

obj-$(CONFIG_NVME_CORE)			+= nvme-core.o
obj-$(CONFIG_BLK_DEV_NVME)		+= nvme.o
obj-$(CONFIG_BLK_DEV_NVME_APPLE)	+= apple.o

nvme-core-y				:= core.o
nvme-y					+= pci.o
