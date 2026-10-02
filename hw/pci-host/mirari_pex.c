/*
 * QEMU model of the QorIQ T104x PCI Express controller (PEX), as found on
 * the Mirari board.
 *
 * Derived from hw/pci-host/ppce500.c:
 *   Copyright (C) 2009 Freescale Semiconductor, Inc. All rights reserved.
 *   Author: Yu Liu,     <yu.liu@freescale.com>
 *
 * Differences from the e500v2 PCI block:
 *  - CFG_ADDR carries the PCIe extended register number in bits 27:24;
 *  - PEX_IP_BLK_REV1 at +0xBF8;
 *  - the root complex is a PCI-to-PCI bridge (header type 1) whose
 *    extended config register 0x404 (PCI_LTSSM) reads 0x16 (L0) when the
 *    link is up, 0xFFFFFFFF otherwise.  Config cycles to any device answer
 *    all ones while the link is down, like the real controller;
 *  - one INTx line: all four pins are ORed into the controller's MPIC
 *    interrupt;
 *  - firmware (u-boot) state is provided at reset: the outbound I/O window
 *    at 0xF:F80n0000 and the root complex bridge windows.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/mirari_pex.h"
#include "qom/object.h"

#define PEX_CFG_ADDR            0x0
#define PEX_CFG_DATA            0x4
#define PEX_INT_ACK             0x8
#define PEX_REG_BASE            0xC00
#define PEX_ALL_SIZE            0x1000
#define PEX_REG_SIZE            (PEX_ALL_SIZE - PEX_REG_BASE)

#define PEX_IP_BLK_REV1         0xBF8
#define PEX_IP_BLK_REV2         0xBFC
/* T1042: IP block rev 2.4 */
#define PEX_IP_BLK_REV1_T1042   0x02080204

#define PEX_PCI_IOLEN           0x10000ULL

#define PEX_OW1                 (0xC20 - PEX_REG_BASE)
#define PEX_OW2                 (0xC40 - PEX_REG_BASE)
#define PEX_OW3                 (0xC60 - PEX_REG_BASE)
#define PEX_OW4                 (0xC80 - PEX_REG_BASE)
#define PEX_IW3                 (0xDA0 - PEX_REG_BASE)
#define PEX_IW2                 (0xDC0 - PEX_REG_BASE)
#define PEX_IW1                 (0xDE0 - PEX_REG_BASE)

#define PEX_GASKET_TIMR         (0xE20 - PEX_REG_BASE)

#define PCI_POTAR               0x0
#define PCI_POTEAR              0x4
#define PCI_POWBAR              0x8
#define PCI_POWAR               0x10

#define PCI_PITAR               0x0
#define PCI_PIWBAR              0x8
#define PCI_PIWBEAR             0xC
#define PCI_PIWAR               0x10

#define PEX_NR_POBS             5
#define PEX_NR_PIBS             3

#define PIWAR_EN                0x80000000      /* Enable */
#define PIWAR_PF                0x20000000      /* prefetch */
#define PIWAR_TGI_LOCAL         0x00f00000      /* target - local memory */
#define PIWAR_READ_SNOOP        0x00050000
#define PIWAR_WRITE_SNOOP       0x00005000
#define PIWAR_SZ_MASK           0x0000003f

/* PCIe LTSSM state register in the root complex extended config space */
#define PEX_CFG_LTSSM           0x404
#define LTSSM_L0                0x16

struct pex_outbound {
    uint32_t potar;
    uint32_t potear;
    uint32_t powbar;
    uint32_t powar;
    MemoryRegion mem;
};

struct pex_inbound {
    uint32_t pitar;
    uint32_t piwbar;
    uint32_t piwbear;
    uint32_t piwar;
    MemoryRegion mem;
};

OBJECT_DECLARE_SIMPLE_TYPE(MirariPEXState, MIRARI_PEX_HOST)

struct MirariPEXState {
    PCIHostState parent_obj;

    struct pex_outbound pob[PEX_NR_POBS];
    struct pex_inbound pib[PEX_NR_PIBS];
    uint32_t gasket_time;
    qemu_irq irq;
    uint32_t pin_level[PCI_NUM_PINS];
    uint32_t index;
    bool link_up;
    /* firmware-programmed outbound I/O window (window 1) */
    uint64_t io_cpu_base;
    /* root complex bridge memory window on the PCI side */
    uint32_t mem_base;
    uint32_t mem_size;
    /* bump allocators used by the firmware BAR assignment at reset */
    uint64_t fw_mem;
    uint64_t fw_io;
    AddressSpace bm_as;
    MemoryRegion bm;
    /* mmio maps */
    MemoryRegion container;
    MemoryRegion iomem;
    MemoryRegion pio;
    MemoryRegion busmem;
    MemoryRegion cfg_data;
    PCIDevice *root;
};

#define TYPE_MIRARI_PEX_ROOT "mirari-pex-root"
OBJECT_DECLARE_SIMPLE_TYPE(MirariPEXRootState, MIRARI_PEX_ROOT)

struct MirariPEXRootState {
    /*< private >*/
    PCIBridge parent;
    /*< public >*/
    uint32_t mem_base;
    uint32_t mem_size;
};

static uint64_t pex_reg_read4(void *opaque, hwaddr addr, unsigned size)
{
    MirariPEXState *pci = opaque;
    unsigned long win;
    uint32_t value = 0;
    int idx;

    win = addr & 0xfe0;

    switch (win) {
    case PEX_OW1:
    case PEX_OW2:
    case PEX_OW3:
    case PEX_OW4:
        idx = (addr >> 5) & 0x7;
        switch (addr & 0x1F) {
        case PCI_POTAR:
            value = pci->pob[idx].potar;
            break;
        case PCI_POTEAR:
            value = pci->pob[idx].potear;
            break;
        case PCI_POWBAR:
            value = pci->pob[idx].powbar;
            break;
        case PCI_POWAR:
            value = pci->pob[idx].powar;
            break;
        default:
            break;
        }
        break;

    case PEX_IW3:
    case PEX_IW2:
    case PEX_IW1:
        idx = ((addr >> 5) & 0x3) - 1;
        switch (addr & 0x1F) {
        case PCI_PITAR:
            value = pci->pib[idx].pitar;
            break;
        case PCI_PIWBAR:
            value = pci->pib[idx].piwbar;
            break;
        case PCI_PIWBEAR:
            value = pci->pib[idx].piwbear;
            break;
        case PCI_PIWAR:
            value = pci->pib[idx].piwar;
            break;
        default:
            break;
        };
        break;

    case PEX_GASKET_TIMR:
        value = pci->gasket_time;
        break;

    default:
        break;
    }

    return value;
}

/* DMA mapping */
static void pex_update_piw(MirariPEXState *pci, int idx)
{
    uint64_t tar = ((uint64_t)pci->pib[idx].pitar) << 12;
    uint64_t wbar = ((uint64_t)pci->pib[idx].piwbar) << 12;
    uint64_t war = pci->pib[idx].piwar;
    uint64_t size = 2ULL << (war & PIWAR_SZ_MASK);
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *mem = &pci->pib[idx].mem;
    MemoryRegion *bm = &pci->bm;
    char *name;

    if (memory_region_is_mapped(mem)) {
        /* Before we modify anything, unmap and destroy the region */
        memory_region_del_subregion(bm, mem);
        object_unparent(OBJECT(mem));
    }

    if (!(war & PIWAR_EN)) {
        /* Not enabled, nothing to do */
        return;
    }

    /*
     * The controller matches an inbound transaction against its own
     * windows before the address ever reaches the bus, so local memory
     * wins over whatever a device behind the controller has a BAR at.
     * At the priority the other way round, every byte of RAM the PCI
     * memory window happens to cover -- from `-m 2560M' up, where RAM
     * reaches mem_base -- is shadowed for DMA by those BARs.
     */
    name = g_strdup_printf("PEX%d Inbound Window %d", pci->index, idx);
    memory_region_init_alias(mem, OBJECT(pci), name, address_space_mem, tar,
                             size);
    memory_region_add_subregion_overlap(bm, wbar, mem, 1);
    g_free(name);
}

/* BAR mapping */
static void pex_update_pow(MirariPEXState *pci, int idx)
{
    uint64_t tar = ((uint64_t)pci->pob[idx].potar) << 12;
    uint64_t wbar = ((uint64_t)pci->pob[idx].powbar) << 12;
    uint64_t war = pci->pob[idx].powar;
    uint64_t size = 2ULL << (war & PIWAR_SZ_MASK);
    MemoryRegion *mem = &pci->pob[idx].mem;
    MemoryRegion *address_space_mem = get_system_memory();
    char *name;

    if (memory_region_is_mapped(mem)) {
        /* Before we modify anything, unmap and destroy the region */
        memory_region_del_subregion(address_space_mem, mem);
        object_unparent(OBJECT(mem));
    }

    if (!(war & PIWAR_EN)) {
        /* Not enabled, nothing to do */
        return;
    }

    name = g_strdup_printf("PEX%d Outbound Window %d", pci->index, idx);
    memory_region_init_alias(mem, OBJECT(pci), name, &pci->busmem, tar,
                             size);
    memory_region_add_subregion(address_space_mem, wbar, mem);
    g_free(name);
}

static void pex_reg_write4(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    MirariPEXState *pci = opaque;
    unsigned long win;
    int idx;

    win = addr & 0xfe0;

    switch (win) {
    case PEX_OW1:
    case PEX_OW2:
    case PEX_OW3:
    case PEX_OW4:
        idx = (addr >> 5) & 0x7;
        switch (addr & 0x1F) {
        case PCI_POTAR:
            pci->pob[idx].potar = value;
            pex_update_pow(pci, idx);
            break;
        case PCI_POTEAR:
            pci->pob[idx].potear = value;
            pex_update_pow(pci, idx);
            break;
        case PCI_POWBAR:
            pci->pob[idx].powbar = value;
            pex_update_pow(pci, idx);
            break;
        case PCI_POWAR:
            pci->pob[idx].powar = value;
            pex_update_pow(pci, idx);
            break;
        default:
            break;
        };
        break;

    case PEX_IW3:
    case PEX_IW2:
    case PEX_IW1:
        idx = ((addr >> 5) & 0x3) - 1;
        switch (addr & 0x1F) {
        case PCI_PITAR:
            pci->pib[idx].pitar = value;
            pex_update_piw(pci, idx);
            break;
        case PCI_PIWBAR:
            pci->pib[idx].piwbar = value;
            pex_update_piw(pci, idx);
            break;
        case PCI_PIWBEAR:
            pci->pib[idx].piwbear = value;
            pex_update_piw(pci, idx);
            break;
        case PCI_PIWAR:
            pci->pib[idx].piwar = value;
            pex_update_piw(pci, idx);
            break;
        default:
            break;
        };
        break;

    case PEX_GASKET_TIMR:
        pci->gasket_time = value;
        break;

    default:
        break;
    };
}

static const MemoryRegionOps pex_reg_ops = {
    .read = pex_reg_read4,
    .write = pex_reg_write4,
    .endianness = DEVICE_BIG_ENDIAN,
};

/*
 * The block revision registers sit between the config access registers
 * and the ATMU block, outside of pex_reg_ops.
 */
static uint64_t pex_blkrev_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr) {
    case 0:
        return PEX_IP_BLK_REV1_T1042;
    default:
        return 0;
    }
}

static void pex_blkrev_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
}

static const MemoryRegionOps pex_blkrev_ops = {
    .read = pex_blkrev_read,
    .write = pex_blkrev_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

/*
 * Config data register.  CFG_ADDR: bit 31 enable, 27:24 extended register
 * number, 23:16 bus, 15:11 device, 10:8 function, 7:2 register.
 */
static bool pex_cfg_decode(MirariPEXState *s, hwaddr addr, PCIDevice **dev,
                           uint32_t *reg)
{
    PCIHostState *h = PCI_HOST_BRIDGE(s);
    uint32_t cfg = h->config_reg;
    uint8_t busnr, devfn;

    if (!(cfg & 0x80000000) || !s->link_up) {
        return false;
    }

    busnr = (cfg >> 16) & 0xff;
    devfn = (cfg >> 8) & 0xff;
    *reg = (((cfg >> 24) & 0xf) << 8) | (cfg & 0xfc) | (addr & 3);
    *dev = pci_find_device(h->bus, busnr, devfn);
    return *dev != NULL;
}

static uint64_t pex_cfg_data_read(void *opaque, hwaddr addr, unsigned len)
{
    MirariPEXState *s = opaque;
    PCIDevice *dev;
    uint32_t reg;

    if (!pex_cfg_decode(s, addr, &dev, &reg)) {
        return ~0ULL;
    }

    if (reg >= PCI_CONFIG_SPACE_SIZE) {
        /* PCIe extended config space: only modelled for the root complex */
        if (dev == s->root && (reg & ~3) == PEX_CFG_LTSSM) {
            return extract32(LTSSM_L0, (reg & 3) * 8, len * 8);
        }
        return 0;
    }

    return pci_host_config_read_common(dev, reg, PCI_CONFIG_SPACE_SIZE, len);
}

static void pex_cfg_data_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned len)
{
    MirariPEXState *s = opaque;
    PCIDevice *dev;
    uint32_t reg;

    if (!pex_cfg_decode(s, addr, &dev, &reg)) {
        return;
    }

    if (reg >= PCI_CONFIG_SPACE_SIZE) {
        return;
    }

    pci_host_config_write_common(dev, reg, PCI_CONFIG_SPACE_SIZE, val, len);
}

static const MemoryRegionOps pex_cfg_data_ops = {
    .read = pex_cfg_data_read,
    .write = pex_cfg_data_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static int pex_map_irq(PCIDevice *pci_dev, int pin)
{
    /* devices on the root bus: pin as is */
    return pin;
}

static void pex_set_irq(void *opaque, int pin, int level)
{
    MirariPEXState *s = opaque;
    int i, any = 0;

    s->pin_level[pin] = level;
    for (i = 0; i < PCI_NUM_PINS; i++) {
        any |= s->pin_level[i];
    }
    qemu_set_irq(s->irq, any);
}

static PCIINTxRoute pex_route_intx_pin_to_irq(void *opaque, int pin)
{
    PCIINTxRoute route;

    route.mode = PCI_INTX_ENABLED;
    route.irq = -1;

    return route;
}

static const VMStateDescription vmstate_pex_outbound = {
    .name = "mirari_pex_outbound",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(potar, struct pex_outbound),
        VMSTATE_UINT32(potear, struct pex_outbound),
        VMSTATE_UINT32(powbar, struct pex_outbound),
        VMSTATE_UINT32(powar, struct pex_outbound),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pex_inbound = {
    .name = "mirari_pex_inbound",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pitar, struct pex_inbound),
        VMSTATE_UINT32(piwbar, struct pex_inbound),
        VMSTATE_UINT32(piwbear, struct pex_inbound),
        VMSTATE_UINT32(piwar, struct pex_inbound),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_mirari_pex = {
    .name = "mirari_pex",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(pob, MirariPEXState, PEX_NR_POBS, 1,
                             vmstate_pex_outbound, struct pex_outbound),
        VMSTATE_STRUCT_ARRAY(pib, MirariPEXState, PEX_NR_PIBS, 1,
                             vmstate_pex_inbound, struct pex_inbound),
        VMSTATE_UINT32(gasket_time, MirariPEXState),
        VMSTATE_UINT32_ARRAY(pin_level, MirariPEXState, PCI_NUM_PINS),
        VMSTATE_END_OF_LIST()
    }
};

/* ---- root complex: a PCI-to-PCI bridge pre-programmed like u-boot did */

static void pex_root_realize(PCIDevice *d, Error **errp)
{
    pci_bridge_initfn(d, TYPE_PCI_BUS);
}

static void pex_root_reset(DeviceState *qdev)
{
    PCIDevice *d = PCI_DEVICE(qdev);
    MirariPEXRootState *r = MIRARI_PEX_ROOT(qdev);
    uint8_t *conf = d->config;
    uint32_t base = r->mem_base;
    uint32_t limit = r->mem_base + r->mem_size - 1;

    pci_bridge_reset(qdev);

    /*
     * Firmware state: u-boot leaves the memory window open over the whole
     * outbound range and the I/O window over the 64 KB I/O space.
     */
    conf[PCI_PRIMARY_BUS] = 0;
    conf[PCI_SECONDARY_BUS] = 1;
    conf[PCI_SUBORDINATE_BUS] = 1;
    pci_set_word(conf + PCI_MEMORY_BASE, (base >> 16) & 0xfff0);
    pci_set_word(conf + PCI_MEMORY_LIMIT, (limit >> 16) & 0xfff0);
    pci_set_word(conf + PCI_PREF_MEMORY_BASE, 0xfff0);
    pci_set_word(conf + PCI_PREF_MEMORY_LIMIT, 0x0);
    conf[PCI_IO_BASE] = 0x00;
    conf[PCI_IO_LIMIT] = 0xf0;
    pci_set_word(conf + PCI_COMMAND,
                 PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
    pci_bridge_update_mappings(PCI_BRIDGE(d));
}

static const Property pex_root_properties[] = {
    DEFINE_PROP_UINT32("mem-base", MirariPEXRootState, mem_base, 0xa0000000),
    DEFINE_PROP_UINT32("mem-size", MirariPEXRootState, mem_size, 0x20000000),
};

static void pex_root_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = pex_root_realize;
    k->exit = pci_bridge_exitfn;
    k->config_write = pci_bridge_write_config;
    k->vendor_id = PCI_VENDOR_ID_FREESCALE;
    k->device_id = 0x0825;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_BRIDGE_PCI;
    dc->desc = "QorIQ PCIe root complex";
    device_class_set_legacy_reset(dc, pex_root_reset);
    device_class_set_props(dc, pex_root_properties);
    /* Created by the host controller only */
    dc->user_creatable = false;
}

/* ---- host controller */

static AddressSpace *pex_set_iommu(PCIBus *bus, void *opaque, int devfn)
{
    MirariPEXState *s = opaque;

    return &s->bm_as;
}

static const PCIIOMMUOps pex_iommu_ops = {
    .get_address_space = pex_set_iommu,
};

/*
 * u-boot's PCI enumeration, as the ROM finds it.
 *
 * quark assigns the BARs itself but never touches the command register:
 * it expects the firmware to have enabled the devices already (it even
 * switches decoding back off for USB controllers on purpose).  Without a
 * firmware doing that, a -device on the bus is configured but stays
 * silent - every access to its BARs is dropped on the floor.
 *
 * So do what u-boot's pci_init() does: hand out BAR addresses from the
 * outbound windows and turn on I/O, memory and bus mastering.  The
 * addresses do not matter much (quark reassigns them), but they must not
 * overlap each other or the inbound DMA window at PCI address 0.
 */
static void pex_fw_config_bus(MirariPEXState *s, PCIBus *bus);

static void pex_fw_config_device(PCIBus *bus, PCIDevice *d, void *opaque)
{
    MirariPEXState *s = opaque;
    uint16_t cmd = PCI_COMMAND_MASTER;
    int i;

    if (IS_PCI_BRIDGE(d)) {
        /* The windows of a guest-visible bridge are the guest's business */
        pex_fw_config_bus(s, pci_bridge_get_sec_bus(PCI_BRIDGE(d)));
        return;
    }

    for (i = 0; i < PCI_NUM_REGIONS; i++) {
        PCIIORegion *r = &d->io_regions[i];
        uint64_t base, end, *ptr;

        /* u-boot leaves the expansion ROM window unassigned */
        if (!r->size || i == PCI_ROM_SLOT) {
            continue;
        }

        if (r->type & PCI_BASE_ADDRESS_SPACE_IO) {
            ptr = &s->fw_io;
            end = PEX_PCI_IOLEN;
        } else {
            ptr = &s->fw_mem;
            end = (uint64_t)s->mem_base + s->mem_size;
        }

        base = ROUND_UP(*ptr, r->size);
        if (base + r->size > end) {
            warn_report("mirari-pex%d: no room left for BAR%d of %s",
                        s->index, i, object_get_typename(OBJECT(d)));
            continue;
        }
        *ptr = base + r->size;
        cmd |= (r->type & PCI_BASE_ADDRESS_SPACE_IO) ?
               PCI_COMMAND_IO : PCI_COMMAND_MEMORY;

        pci_host_config_write_common(d, pci_bar(d, i), pci_config_size(d),
                                     base, 4);
        if ((r->type & (PCI_BASE_ADDRESS_SPACE_IO |
                        PCI_BASE_ADDRESS_MEM_TYPE_MASK)) ==
            PCI_BASE_ADDRESS_MEM_TYPE_64) {
            pci_host_config_write_common(d, pci_bar(d, i) + 4,
                                         pci_config_size(d), base >> 32, 4);
            i++;
        }
    }

    pci_host_config_write_common(d, PCI_COMMAND, pci_config_size(d),
                                 pci_get_word(d->config + PCI_COMMAND) | cmd,
                                 2);
}

static void pex_fw_config_bus(MirariPEXState *s, PCIBus *bus)
{
    pci_for_each_device_under_bus(bus, pex_fw_config_device, s);
}

static void pex_host_reset(DeviceState *dev)
{
    MirariPEXState *s = MIRARI_PEX_HOST(dev);
    int i;

    for (i = 0; i < PEX_NR_POBS; i++) {
        s->pob[i].potar = 0;
        s->pob[i].potear = 0;
        s->pob[i].powbar = 0;
        s->pob[i].powar = 0;
        pex_update_pow(s, i);
    }
    for (i = 0; i < PEX_NR_PIBS; i++) {
        s->pib[i].pitar = 0;
        s->pib[i].piwbar = 0;
        s->pib[i].piwbear = 0;
        s->pib[i].piwar = 0;
        pex_update_piw(s, i);
    }
    for (i = 0; i < PCI_NUM_PINS; i++) {
        s->pin_level[i] = 0;
    }

    /*
     * u-boot state: outbound window 1 = 64 KB of I/O space at
     * io_cpu_base (PCI I/O address 0), window 2 = the memory window,
     * inbound window 1 = the whole 4 GB of local memory (PEXIWAR
     * size 0x1f).
     */
    s->pob[1].potar = 0;
    s->pob[1].potear = 0;
    s->pob[1].powbar = s->io_cpu_base >> 12;
    s->pob[1].powar = 0x8008800f;
    pex_update_pow(s, 1);

    s->pob[2].potar = s->mem_base >> 12;
    s->pob[2].potear = 0;
    s->pob[2].powbar = (0xc00000000ULL + (s->mem_base - 0xa0000000)) >> 12;
    s->pob[2].powar = 0x80044000 | (ctz32(s->mem_size) - 1);
    pex_update_pow(s, 2);

    s->pib[0].pitar = 0;
    s->pib[0].piwbar = 0;
    s->pib[0].piwbear = 0;
    s->pib[0].piwar = PIWAR_EN | PIWAR_PF | PIWAR_TGI_LOCAL |
                      PIWAR_READ_SNOOP | PIWAR_WRITE_SNOOP | 0x1f;
    pex_update_piw(s, 0);

    /*
     * ... and the devices behind a controller whose link came up.  The
     * root complex, and every device on the bus, have been reset by now:
     * a device holds its reset before its parent bus does.
     */
    if (s->link_up) {
        s->fw_mem = s->mem_base;
        s->fw_io = 0x1000;
        pex_fw_config_bus(s, pci_bridge_get_sec_bus(PCI_BRIDGE(s->root)));
    }
}

static void pex_host_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    PCIHostState *h;
    MirariPEXState *s;
    PCIBus *b;
    char *name;

    h = PCI_HOST_BRIDGE(dev);
    s = MIRARI_PEX_HOST(dev);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init(&s->pio, OBJECT(s), "pex-pio", PEX_PCI_IOLEN);
    memory_region_init(&s->busmem, OBJECT(s), "pex bus memory", UINT64_MAX);

    /* PIO lives at the bottom of our bus space */
    memory_region_add_subregion_overlap(&s->busmem, 0, &s->pio, -2);

    name = g_strdup_printf("pex%d", s->index);
    b = pci_register_root_bus(dev, name, pex_set_irq, pex_map_irq, s,
                              &s->busmem, &s->pio, PCI_DEVFN(0, 0), 4,
                              TYPE_PCI_BUS);
    g_free(name);
    h->bus = b;

    /* Set up PCI view of memory */
    memory_region_init(&s->bm, OBJECT(s), "bm-pex", UINT64_MAX);
    memory_region_add_subregion(&s->bm, 0x0, &s->busmem);
    address_space_init(&s->bm_as, &s->bm, "pex-bm");
    pci_setup_iommu(b, &pex_iommu_ops, s);

    /* The root complex bridge, and the bus behind it for the devices */
    {
        PCIDevice *rdev = pci_new(PCI_DEVFN(0, 0), TYPE_MIRARI_PEX_ROOT);
        /* names the secondary bus, see pci_bridge_initfn() */
        DEVICE(rdev)->id = g_strdup_printf("pci.%d", s->index);
        qdev_prop_set_uint32(DEVICE(rdev), "mem-base", s->mem_base);
        qdev_prop_set_uint32(DEVICE(rdev), "mem-size", s->mem_size);
        pci_realize_and_unref(rdev, b, &error_fatal);
        s->root = rdev;
        /*
         * Steer bus-less -device options at the secondary bus: only
         * devices behind the root complex are usable.
         */
        BUS(b)->full = true;
        /*
         * A controller whose link is down has no devices behind it:
         * keep bus-less -device placement away from its secondary bus
         * too (an explicit bus=pci.N still works).
         */
        if (!s->link_up) {
            BUS(pci_bridge_get_sec_bus(PCI_BRIDGE(rdev)))->full = true;
        }
    }

    memory_region_init(&s->container, OBJECT(h), "pex-container",
                       PEX_ALL_SIZE);
    memory_region_init_io(&h->conf_mem, OBJECT(h), &pci_host_conf_be_ops, h,
                          "pex-conf-idx", 4);
    memory_region_init_io(&s->cfg_data, OBJECT(s), &pex_cfg_data_ops, s,
                          "pex-conf-data", 4);
    memory_region_init_io(&s->iomem, OBJECT(s), &pex_reg_ops, s,
                          "pex.reg", PEX_REG_SIZE);
    memory_region_add_subregion(&s->container, PEX_CFG_ADDR, &h->conf_mem);
    memory_region_add_subregion(&s->container, PEX_CFG_DATA, &s->cfg_data);
    memory_region_add_subregion(&s->container, PEX_REG_BASE, &s->iomem);
    {
        MemoryRegion *blkrev = g_new0(MemoryRegion, 1);
        memory_region_init_io(blkrev, OBJECT(s), &pex_blkrev_ops, s,
                              "pex.blkrev", 8);
        memory_region_add_subregion(&s->container, PEX_IP_BLK_REV1, blkrev);
    }
    sysbus_init_mmio(sbd, &s->container);
    pci_bus_set_route_irq_fn(b, pex_route_intx_pin_to_irq);
}

static const Property pex_host_properties[] = {
    DEFINE_PROP_UINT32("index", MirariPEXState, index, 0),
    DEFINE_PROP_BOOL("link-up", MirariPEXState, link_up, true),
    DEFINE_PROP_UINT64("io-cpu-base", MirariPEXState, io_cpu_base,
                       0xff8000000ULL),
    DEFINE_PROP_UINT32("mem-base", MirariPEXState, mem_base, 0xa0000000),
    DEFINE_PROP_UINT32("mem-size", MirariPEXState, mem_size, 0x20000000),
};

static void pex_host_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pex_host_realize;
    device_class_set_legacy_reset(dc, pex_host_reset);
    device_class_set_props(dc, pex_host_properties);
    dc->vmsd = &vmstate_mirari_pex;
}

static const TypeInfo mirari_pex_types[] = {
    {
        .name          = TYPE_MIRARI_PEX_ROOT,
        .parent        = TYPE_PCI_BRIDGE,
        .instance_size = sizeof(MirariPEXRootState),
        .class_init    = pex_root_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    },
    {
        .name          = TYPE_MIRARI_PEX_HOST,
        .parent        = TYPE_PCI_HOST_BRIDGE,
        .instance_size = sizeof(MirariPEXState),
        .class_init    = pex_host_class_init,
    },
};

DEFINE_TYPES(mirari_pex_types)
