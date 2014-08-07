// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 *  Copyright (C) 2012 John Crispin <john@phrozen.org>
 */

#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci.h>
#include "ifxmips_pci_common.h"

int pcibios_plat_dev_init(struct pci_dev *dev)
{
#ifdef CONFIG_PCIE_LANTIQ
	if (pci_find_capability(dev, PCI_CAP_ID_EXP))
		ifx_pcie_bios_plat_dev_init(dev);
#endif
	return 0;
}

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
#ifdef CONFIG_PCIE_LANTIQ
	if (pci_find_capability((struct pci_dev *)dev, PCI_CAP_ID_EXP))
		return ifx_pcie_bios_map_irq(dev, slot, pin);
#endif

	return of_irq_parse_and_map_pci(dev, slot, pin);
}
