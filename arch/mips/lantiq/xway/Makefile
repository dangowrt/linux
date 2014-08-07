# SPDX-License-Identifier: GPL-2.0-only
obj-y := prom.o sysctrl.o clk.o dma.o dcdc.o

ifdef CONFIG_SOC_AMAZON_SE
obj-y += gptu.o
else
obj-y += timer.o
endif

obj-y += vmmc.o
