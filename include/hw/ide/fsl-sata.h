/*
 * Freescale/NXP QorIQ SATA controller (P50x0, T104x, "fsl,pq-sata")
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_IDE_FSL_SATA_H
#define HW_IDE_FSL_SATA_H

#include "hw/core/sysbus.h"
#include "hw/ide/ide-bus.h"
#include "qom/object.h"

#define TYPE_FSL_SATA "fsl-sata"
OBJECT_DECLARE_SIMPLE_TYPE(FslSataState, FSL_SATA)

struct FslSataState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    AddressSpace *as;
    QEMUBH *run_bh;

    /* The single SATA port: one IDE bus with one device */
    IDEBus bus;
    IDEDMA dma;

    /* Host controller registers */
    uint32_t cqr;           /* command queue */
    uint32_t car;           /* command active */
    uint32_t ccr;           /* command completed */
    uint32_t cer;           /* command error */
    uint32_t der;           /* device error */
    uint32_t chba;          /* command header base address */
    uint32_t hstatus;
    uint32_t hcontrol;
    uint32_t cqpmp;
    uint32_t sig;           /* device signature */
    uint32_t icc;           /* interrupt coalescing control */
    uint32_t sstatus;
    uint32_t serror;
    uint32_t scontrol;
    uint32_t snotify;
    uint32_t transcfg;
    uint32_t linkcfg;
    uint32_t linkcfg1;
    uint32_t linkcfg2;
    uint32_t phyctrlcfg;

    /* Command in flight (busy_slot < 0: none) */
    int busy_slot;
    uint32_t cur_desc;      /* command descriptor address */
    uint32_t cur_info;      /* command header desc_info */
    uint16_t cur_prde;      /* number of PRD entries */
    uint8_t cur_cmd;        /* ATA command byte */
    bool cur_is_write;      /* PIO data direction, host to device */
    bool done_first_drq;
    bool srst;              /* SRST asserted by a control FIS */
};

#endif
