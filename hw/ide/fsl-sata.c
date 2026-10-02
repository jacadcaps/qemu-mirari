/*
 * Freescale/NXP QorIQ SATA controller (P50x0, T104x, "fsl,pq-sata")
 *
 * One SATA port per controller block.  The host reads a command header
 * table (CHBA, 16 bytes per tag) whose entries point at a command
 * descriptor: a 32 byte H2D register FIS, a 32 byte area where the
 * controller stores the D2H register FIS on completion, a 16 byte ATAPI
 * command and a PRDT of up to 16 direct entries (an entry with the EXT
 * bit set points at an indirect extension table).  All of it is little
 * endian, as is the register set.  The layout and the register bits
 * follow the Linux sata_fsl driver.
 *
 * Commands run one at a time: the queue bits are handled generically but
 * NCQ is not implemented.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/ide/fsl-sata.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ide-internal.h"

/* Host controller command register set */
#define REG_CQR             0x000
#define REG_CAR             0x008
#define REG_CCR             0x010
#define REG_CER             0x018
#define REG_DER             0x020
#define REG_CHBA            0x024
#define REG_HSTATUS         0x028
#define REG_HCONTROL        0x02c
#define REG_CQPMP           0x030
#define REG_SIG             0x034
#define REG_ICC             0x038
/* SATA superset (SCR) registers */
#define REG_SSTATUS         0x100
#define REG_SERROR          0x104
#define REG_SCONTROL        0x108
#define REG_SNOTIFY         0x10c
/* Transport / link / PHY */
#define REG_TRANSCFG        0x140
#define REG_TRANSSTATUS     0x144
#define REG_LINKCFG         0x148
#define REG_LINKCFG1        0x14c
#define REG_LINKCFG2        0x150
#define REG_LINKSTATUS      0x154
#define REG_LINKSTATUS1     0x158
#define REG_PHYCTRLCFG      0x15c
#define REG_COMMANDSTAT     0x184

#define FSL_SATA_MMIO_SIZE  0x1000

/* HSTATUS */
#define HSTATUS_ONLINE              (1u << 31)
#define HSTATUS_GOING_OFFLINE       (1u << 30)
#define HSTATUS_BIST_ERR            (1u << 29)
#define HSTATUS_INT_DATA_LEN        (1u << 12)
#define HSTATUS_INT_FATAL           (1u << 5)
#define HSTATUS_INT_PHYRDY          (1u << 4)
#define HSTATUS_INT_SIGNATURE       (1u << 3)
#define HSTATUS_INT_SNOTIFY         (1u << 2)
#define HSTATUS_INT_DEVICE_ERR      (1u << 1)
#define HSTATUS_INT_CMD_COMPLETE    (1u << 0)
#define HSTATUS_INT_MASK            0x3f

/* HCONTROL */
#define HCONTROL_ONLINE_PHY_RST     (1u << 31)
#define HCONTROL_FORCE_OFFLINE      (1u << 30)
#define HCONTROL_LEGACY             (1u << 28)
#define HCONTROL_CLEAR_ERROR        (1u << 27)
#define HCONTROL_INT_EN_MASK        0x3f

/* SCONTROL / SSTATUS */
#define SCR_DET_MASK                0xf
#define SCR_DET_COMRESET            1
#define SSTATUS_DEV_PHY_UP          0x113   /* DET=3, SPD=Gen1, IPM=active */

/* Command header (16 bytes per tag) */
#define CMD_HDR_SIZE                16
#define CMD_HDR_CDA                 0
#define CMD_HDR_PRDE_FIS_LEN        4
#define CMD_HDR_TTL                 8
#define CMD_HDR_DESC_INFO           12

/* desc_info */
#define DESC_INFO_CMD_DESC_RES      (1u << 11)
#define DESC_INFO_VENDOR_BIST       (1u << 10)
#define DESC_INFO_SNOOP             (1u << 9)
#define DESC_INFO_FPDMA_QUEUED      (1u << 8)
#define DESC_INFO_SRST              (1u << 7)
#define DESC_INFO_BIST              (1u << 6)
#define DESC_INFO_ATAPI             (1u << 5)
#define DESC_INFO_TAG_MASK          0x1f

/* Command descriptor */
#define CMD_DESC_CFIS               0x00
#define CMD_DESC_SFIS               0x20
#define CMD_DESC_ACMD               0x40
#define CMD_DESC_PRDT               0x60
#define CMD_DESC_PRD_DIRECT         16
#define CMD_DESC_PRD_MAX            64

/* PRD entry (16 bytes) */
#define PRD_DBA                     0
#define PRD_DDC_EXT                 12
#define PRD_EXT                     (1u << 31)
#define PRD_LEN_MASK                0x3fffff

/* FIS */
#define FIS_TYPE_REG_H2D            0x27
#define FIS_TYPE_REG_D2H            0x34
#define FIS_H2D_CMD                 0x80    /* byte 1: C bit */
#define FIS_H2D_PMP_MASK            0x0f
#define FIS_CONTROL_SRST            0x04    /* byte 15 of a control FIS */
#define FIS_D2H_INTERRUPT           0x40

#define SATA_SIG_DISK               0x00000101
#define SATA_SIG_CDROM              0xeb140101

static void fsl_sata_update_irq(FslSataState *s)
{
    qemu_set_irq(s->irq, !!(s->hstatus & s->hcontrol & HSTATUS_INT_MASK));
}

static inline IDEState *fsl_sata_drive(FslSataState *s)
{
    return &s->bus.ifs[0];
}

static bool fsl_sata_has_drive(FslSataState *s)
{
    return fsl_sata_drive(s)->blk != NULL;
}

/* ------------------------------------------------------------------ */
/* Link                                                                */

/*
 * The device came out of reset (COMRESET / going online, or SRST): the
 * drive presents its signature in the first D2H register FIS, which the
 * controller latches into SIG.
 *
 * Only the FIS-based device reset -- a command the host issued and is
 * waiting for -- also raises the signature-update event.  The bring-up
 * the host drives itself (HCONTROL going online, then a COMRESET pulse
 * on SCONTROL) leaves no interrupt status behind on a T1042: the host
 * polls SSTATUS and SIG for it.  Latching an event there would be worse
 * than cosmetic, because at that point the host has not armed any
 * interrupt yet: the bits would sit latched and first surface much
 * later attached to an unrelated completion.  A guest that only writes
 * HSTATUS back when it sees CMD_COMPLETE (the MorphOS T1042 driver does
 * exactly that) would then never acknowledge an interrupt carrying only
 * a signature or PHY event, and the level-triggered line would stay
 * asserted for good.
 */
static void fsl_sata_link_reset(FslSataState *s, IDEResetKind kind)
{
    IDEState *ide = fsl_sata_drive(s);
    uint32_t sig;

    ide_bus_reset(&s->bus, kind);
    s->busy_slot = -1;
    s->car = 0;
    s->srst = false;

    if (!fsl_sata_has_drive(s)) {
        s->sstatus = 0;
        return;
    }

    if (ide->drive_kind == IDE_CD) {
        sig = SATA_SIG_CDROM;
        ide->status = SEEK_STAT | WRERR_STAT | READY_STAT;
    } else {
        sig = SATA_SIG_DISK;
        ide->status = SEEK_STAT | WRERR_STAT;
    }
    ide->hcyl = sig >> 24;
    ide->lcyl = (sig >> 16) & 0xff;
    ide->sector = (sig >> 8) & 0xff;
    ide->nsector = sig & 0xff;
    ide->error = 1;             /* diagnostics passed */

    s->sig = sig;
    s->sstatus = SSTATUS_DEV_PHY_UP;
    if (kind == IDE_RESET_SOFTWARE) {
        s->hstatus |= HSTATUS_INT_SIGNATURE;
        fsl_sata_update_irq(s);
    }
}

/* ------------------------------------------------------------------ */
/* Command completion                                                  */

static void fsl_sata_write_d2h_fis(FslSataState *s, bool irq_bit)
{
    IDEState *ide = fsl_sata_drive(s);
    uint8_t fis[20] = { 0 };

    fis[0] = FIS_TYPE_REG_D2H;
    fis[1] = irq_bit ? FIS_D2H_INTERRUPT : 0;
    fis[2] = ide->status;
    fis[3] = ide->error;
    fis[4] = ide->sector;
    fis[5] = ide->lcyl;
    fis[6] = ide->hcyl;
    fis[7] = ide->select;
    fis[8] = ide->hob_sector;
    fis[9] = ide->hob_lcyl;
    fis[10] = ide->hob_hcyl;
    fis[12] = ide->nsector & 0xff;
    fis[13] = (ide->nsector >> 8) & 0xff;

    dma_memory_write(s->as, s->cur_desc + CMD_DESC_SFIS, fis, sizeof(fis),
                     MEMTXATTRS_UNSPECIFIED);
}

static void fsl_sata_run(FslSataState *s);

static void fsl_sata_run_bh(void *opaque)
{
    fsl_sata_run(opaque);
}

/* The command in flight is done: report it and look at the queue again */
static void fsl_sata_complete(FslSataState *s, bool write_fis)
{
    IDEState *ide = fsl_sata_drive(s);
    uint32_t bit;

    if (s->busy_slot < 0) {
        return;
    }
    bit = 1u << s->busy_slot;

    if (write_fis) {
        fsl_sata_write_d2h_fis(s, true);
    }
    s->car &= ~bit;
    s->ccr |= bit;
    s->hstatus |= HSTATUS_INT_CMD_COMPLETE;
    if (ide->status & ERR_STAT) {
        s->der |= bit;
        s->hstatus |= HSTATUS_INT_DEVICE_ERR;
    }
    s->busy_slot = -1;
    fsl_sata_update_irq(s);

    if (s->cqr) {
        qemu_bh_schedule(s->run_bh);
    }
}

/* Controller-side failure: the command never reached the device */
static void fsl_sata_fail(FslSataState *s, int slot, const char *why)
{
    uint32_t bit = 1u << slot;

    qemu_log_mask(LOG_GUEST_ERROR, "fsl-sata: command %d failed: %s\n",
                  slot, why);
    s->cqr &= ~bit;
    s->car &= ~bit;
    s->cer |= bit;
    s->hstatus |= HSTATUS_INT_FATAL;
    if (s->busy_slot == slot) {
        s->busy_slot = -1;
    }
    fsl_sata_update_irq(s);
}

/*
 * The drive raised its interrupt line.  Successful completions come
 * through the cmd_done DMA hook, and the core also pulses the line
 * between the phases of a PIO command, so the line alone means nothing.
 * Some error paths (an asynchronous ide_abort_command, the ATAPI unit
 * attention check) however report only this way.
 */
static void fsl_sata_bus_irq(void *opaque, int n, int level)
{
    FslSataState *s = opaque;
    IDEState *ide = fsl_sata_drive(s);

    if (level && s->busy_slot >= 0 && (ide->status & ERR_STAT) &&
        !(ide->status & (BUSY_STAT | DRQ_STAT))) {
        fsl_sata_complete(s, true);
    }
}

/* ------------------------------------------------------------------ */
/* Command issue                                                       */

/* PIO data direction cannot be told from the FIS, decide by opcode */
static bool fsl_sata_cmd_is_write(uint8_t cmd, uint8_t feature)
{
    switch (cmd) {
    case WIN_WRITE:
    case WIN_WRITE_ONCE:
    case WIN_WRITE_EXT:
    case CFA_WRITE_SECT_WO_ERASE:
    case WIN_MULTWRITE_EXT:
    case WIN_WRITE_VERIFY:
    case WIN_DOWNLOAD_MICROCODE:
    case WIN_MULTWRITE:
    case CFA_WRITE_MULTI_WO_ERASE:
    case WIN_WRITE_BUFFER:
    case WIN_SECURITY_SET_PASS:
    case WIN_SECURITY_UNLOCK:
    case WIN_SECURITY_DISABLE:
        return true;
    case WIN_SMART:
        return feature == SMART_WRITE_LOG;
    default:
        return false;
    }
}

static void fsl_sata_start(FslSataState *s, int slot)
{
    IDEState *ide = fsl_sata_drive(s);
    uint32_t bit = 1u << slot;
    uint8_t hdr[CMD_HDR_SIZE];
    uint8_t cfis[32];
    uint32_t cda, prde_fis_len, info;
    unsigned fis_len;

    s->cqr &= ~bit;

    if (!(s->hstatus & HSTATUS_ONLINE)) {
        fsl_sata_fail(s, slot, "controller offline");
        return;
    }
    if (!fsl_sata_has_drive(s)) {
        fsl_sata_fail(s, slot, "no device attached");
        return;
    }
    if (dma_memory_read(s->as, s->chba + slot * CMD_HDR_SIZE, hdr,
                        sizeof(hdr), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        fsl_sata_fail(s, slot, "command header unreadable");
        return;
    }
    cda = ldl_le_p(hdr + CMD_HDR_CDA);
    prde_fis_len = ldl_le_p(hdr + CMD_HDR_PRDE_FIS_LEN);
    info = ldl_le_p(hdr + CMD_HDR_DESC_INFO);
    fis_len = prde_fis_len & 0xffff;

    if (fis_len < 8 || fis_len > sizeof(cfis) || (fis_len & 3)) {
        fsl_sata_fail(s, slot, "bad FIS length");
        return;
    }
    if (dma_memory_read(s->as, cda + CMD_DESC_CFIS, cfis, sizeof(cfis),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        fsl_sata_fail(s, slot, "command descriptor unreadable");
        return;
    }
    if (cfis[0] != FIS_TYPE_REG_H2D) {
        fsl_sata_fail(s, slot, "not a register H2D FIS");
        return;
    }
    if (cfis[1] & FIS_H2D_PMP_MASK) {
        fsl_sata_fail(s, slot, "port multiplier ports not supported");
        return;
    }
    if (info & (DESC_INFO_FPDMA_QUEUED | DESC_INFO_BIST |
                DESC_INFO_VENDOR_BIST)) {
        fsl_sata_fail(s, slot, "NCQ/BIST not supported");
        return;
    }

    s->busy_slot = slot;
    s->car |= bit;
    s->cur_desc = cda;
    s->cur_info = info;
    s->cur_prde = prde_fis_len >> 16;
    s->done_first_drq = false;

    if (!(cfis[1] & FIS_H2D_CMD)) {
        /*
         * Control FIS: only the SRST bit matters.  Asserting it gets no
         * reply from the drive; releasing it runs the diagnostics and
         * the drive answers with the D2H FIS carrying its signature.
         */
        if (cfis[15] & FIS_CONTROL_SRST) {
            s->srst = true;
            fsl_sata_complete(s, false);
        } else {
            bool was_srst = s->srst;

            if (was_srst) {
                fsl_sata_link_reset(s, IDE_RESET_SOFTWARE);
                /* link_reset cleared the slot bookkeeping */
                s->busy_slot = slot;
                s->car |= bit;
            }
            fsl_sata_complete(s, was_srst);
        }
        return;
    }

    ide->feature = cfis[3];
    ide->sector = cfis[4];
    ide->lcyl = cfis[5];
    ide->hcyl = cfis[6];
    ide->select = cfis[7];
    ide->hob_sector = cfis[8];
    ide->hob_lcyl = cfis[9];
    ide->hob_hcyl = cfis[10];
    ide->hob_feature = cfis[11];
    ide->nsector = (cfis[13] << 8) | cfis[12];
    ide->error = 0;

    if (info & DESC_INFO_ATAPI) {
        dma_memory_read(s->as, cda + CMD_DESC_ACMD, ide->io_buffer, 16,
                        MEMTXATTRS_UNSPECIFIED);
        /*
         * The transport layer paces PIO data by the descriptor, so hosts
         * leave the ATAPI byte count limit at zero; the IDE core needs a
         * real one to size its DRQ blocks.
         */
        if (!ide->lcyl && !ide->hcyl) {
            ide->lcyl = 0xfe;
            ide->hcyl = 0xff;
        }
    }

    s->cur_cmd = cfis[2];
    s->cur_is_write = fsl_sata_cmd_is_write(cfis[2], cfis[3]);

    ide_bus_exec_cmd(&s->bus, cfis[2]);

    /*
     * Commands that finish without going through cmd_done or the
     * interrupt line (DEVICE RESET on an ATAPI drive) are done once the
     * drive is neither busy nor asking for data.
     */
    if (s->busy_slot == slot && !(ide->status & (BUSY_STAT | DRQ_STAT))) {
        fsl_sata_complete(s, true);
    }
}

static void fsl_sata_run(FslSataState *s)
{
    IDEState *ide = fsl_sata_drive(s);

    while (s->cqr && s->busy_slot < 0) {
        if (ide->status & (BUSY_STAT | DRQ_STAT)) {
            /* Drive still busy with the previous command, come back */
            qemu_bh_schedule(s->run_bh);
            return;
        }
        fsl_sata_start(s, ctz32(s->cqr));
    }
}

/* ------------------------------------------------------------------ */
/* Data transfer: the PRDT of the command in flight                    */

static int fsl_sata_populate_sglist(FslSataState *s, QEMUSGList *sglist,
                                    int64_t limit, uint64_t offset)
{
    struct {
        uint32_t addr;
        uint32_t len;
    } prd[CMD_DESC_PRD_MAX];
    hwaddr direct = s->cur_desc + CMD_DESC_PRDT;
    hwaddr ext = 0;
    unsigned nprd = 0, i, first;
    uint64_t sum = 0;
    int64_t off_pos;

    if (s->busy_slot < 0 || !s->cur_prde) {
        return -1;
    }

    /* Flatten the direct entries and the optional extension table */
    for (i = 0; i < s->cur_prde && nprd < CMD_DESC_PRD_MAX; i++) {
        uint8_t e[16];
        uint32_t ddc;
        hwaddr addr;

        if (i < CMD_DESC_PRD_DIRECT) {
            addr = direct + i * 16;
        } else if (ext) {
            addr = ext + (i - CMD_DESC_PRD_DIRECT) * 16;
        } else {
            break;
        }
        if (dma_memory_read(s->as, addr, e, sizeof(e),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return -1;
        }
        ddc = ldl_le_p(e + PRD_DDC_EXT);
        if (ddc & PRD_EXT) {
            ext = ldl_le_p(e + PRD_DBA);
            continue;
        }
        prd[nprd].addr = ldl_le_p(e + PRD_DBA);
        prd[nprd].len = ddc & PRD_LEN_MASK;
        nprd++;
    }
    if (!nprd) {
        return -1;
    }

    /* Skip what earlier chunks of this command already transferred */
    for (first = 0; first < nprd; first++) {
        if (offset < sum + prd[first].len) {
            break;
        }
        sum += prd[first].len;
    }
    if (first == nprd) {
        return -1;
    }
    off_pos = offset - sum;

    qemu_sglist_init(sglist, DEVICE(s), nprd - first, s->as);
    qemu_sglist_add(sglist, prd[first].addr + off_pos,
                    MIN(prd[first].len - off_pos, limit));
    for (i = first + 1; i < nprd && sglist->size < limit; i++) {
        qemu_sglist_add(sglist, prd[i].addr,
                        MIN(prd[i].len, limit - sglist->size));
    }
    return 0;
}

static int32_t fsl_sata_dma_prepare_buf(const IDEDMA *dma, int32_t limit)
{
    FslSataState *s = container_of(dma, FslSataState, dma);
    IDEState *ide = fsl_sata_drive(s);

    if (fsl_sata_populate_sglist(s, &ide->sg, limit,
                                 ide->io_buffer_offset) < 0) {
        return -1;
    }
    ide->io_buffer_size = ide->sg.size;
    return ide->io_buffer_size;
}

static void fsl_sata_dma_commit_buf(const IDEDMA *dma, uint32_t tx_bytes)
{
}

static int fsl_sata_dma_rw_buf(const IDEDMA *dma, bool is_write)
{
    FslSataState *s = container_of(dma, FslSataState, dma);
    IDEState *ide = fsl_sata_drive(s);
    uint8_t *p = ide->io_buffer + ide->io_buffer_index;
    int l = ide->io_buffer_size - ide->io_buffer_index;

    if (fsl_sata_populate_sglist(s, &ide->sg, l, ide->io_buffer_offset) < 0) {
        return 0;
    }
    if (is_write) {
        dma_buf_read(p, l, NULL, &ide->sg, MEMTXATTRS_UNSPECIFIED);
    } else {
        dma_buf_write(p, l, NULL, &ide->sg, MEMTXATTRS_UNSPECIFIED);
    }
    ide_dma_buf_commit(ide, l);
    ide->io_buffer_index += l;
    return 1;
}

/*
 * PIO data phases are moved through the PRDT in one go, like AHCI does.
 * The first DRQ of a PACKET command is the command packet itself, which
 * was already copied from the ACMD field.
 */
static bool fsl_sata_pio_transfer(const IDEDMA *dma)
{
    FslSataState *s = container_of(dma, FslSataState, dma);
    IDEState *ide = fsl_sata_drive(s);
    uint32_t size = ide->data_end - ide->data_ptr;
    bool is_atapi = s->cur_info & DESC_INFO_ATAPI;

    if (s->busy_slot < 0) {
        return false;
    }
    if (is_atapi && !s->done_first_drq) {
        goto out;
    }
    if (size && fsl_sata_dma_prepare_buf(dma, size) > 0) {
        if (s->cur_is_write) {
            dma_buf_write(ide->data_ptr, size, NULL, &ide->sg,
                          MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_buf_read(ide->data_ptr, size, NULL, &ide->sg,
                         MEMTXATTRS_UNSPECIFIED);
        }
    }
    ide_dma_buf_commit(ide, size);

out:
    ide->data_ptr = ide->data_end;
    s->done_first_drq = true;
    return true;
}

static void fsl_sata_dma_start(const IDEDMA *dma, IDEState *ide,
                               BlockCompletionFunc *dma_cb)
{
    ide->io_buffer_offset = 0;
    dma_cb(ide, 0);
}

static void fsl_sata_dma_nop(const IDEDMA *dma)
{
}

static void fsl_sata_dma_cmd_done(const IDEDMA *dma)
{
    FslSataState *s = container_of(dma, FslSataState, dma);
    IDEState *ide = fsl_sata_drive(s);

    if (s->busy_slot < 0) {
        /*
         * Already reported (the IDE core calls this twice for commands
         * that complete inside their handler); refresh the status FIS.
         */
        if (s->cur_desc) {
            fsl_sata_write_d2h_fis(s, true);
        }
        return;
    }
    if (ide->status & BUSY_STAT) {
        /* Nested in ide_bus_exec_cmd, the final call follows */
        return;
    }
    fsl_sata_complete(s, true);
}

static const IDEDMAOps fsl_sata_dma_ops = {
    .start_dma = fsl_sata_dma_start,
    .restart_dma = fsl_sata_dma_nop,
    .pio_transfer = fsl_sata_pio_transfer,
    .prepare_buf = fsl_sata_dma_prepare_buf,
    .commit_buf = fsl_sata_dma_commit_buf,
    .rw_buf = fsl_sata_dma_rw_buf,
    .cmd_done = fsl_sata_dma_cmd_done,
};

/* ------------------------------------------------------------------ */
/* Registers                                                           */

static uint64_t fsl_sata_read(void *opaque, hwaddr addr, unsigned size)
{
    FslSataState *s = opaque;

    switch (addr) {
    case REG_CQR:
        return s->cqr;
    case REG_CAR:
        return s->car;
    case REG_CCR:
        return s->ccr;
    case REG_CER:
        return s->cer;
    case REG_DER:
        return s->der;
    case REG_CHBA:
        return s->chba;
    case REG_HSTATUS:
        return s->hstatus;
    case REG_HCONTROL:
        return s->hcontrol;
    case REG_CQPMP:
        return s->cqpmp;
    case REG_SIG:
        return s->sig;
    case REG_ICC:
        return s->icc;
    case REG_SSTATUS:
        return s->sstatus;
    case REG_SERROR:
        return s->serror;
    case REG_SCONTROL:
        return s->scontrol;
    case REG_SNOTIFY:
        return s->snotify;
    case REG_TRANSCFG:
        return s->transcfg;
    case REG_TRANSSTATUS:
        return 0;
    case REG_LINKCFG:
        return s->linkcfg;
    case REG_LINKCFG1:
        return s->linkcfg1;
    case REG_LINKCFG2:
        return s->linkcfg2;
    case REG_LINKSTATUS:
    case REG_LINKSTATUS1:
        return 0;
    case REG_PHYCTRLCFG:
        return s->phyctrlcfg;
    case REG_COMMANDSTAT:
        return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "fsl-sata: read of register 0x%03x\n",
                      (unsigned)addr);
        return 0;
    }
}

static void fsl_sata_set_online(FslSataState *s, bool online)
{
    if (online == !!(s->hstatus & HSTATUS_ONLINE)) {
        return;
    }
    if (online) {
        s->hstatus |= HSTATUS_ONLINE;
        /* Going online starts the OOB sequence: the link comes up */
        if ((s->scontrol & SCR_DET_MASK) != SCR_DET_COMRESET) {
            fsl_sata_link_reset(s, IDE_RESET_HARDWARE);
        }
    } else {
        s->hstatus &= ~HSTATUS_ONLINE;
        s->sstatus = 0;
        s->cqr = 0;
        s->car = 0;
        s->busy_slot = -1;
    }
}

static void fsl_sata_write(void *opaque, hwaddr addr, uint64_t val64,
                           unsigned size)
{
    FslSataState *s = opaque;
    uint32_t val = val64;

    switch (addr) {
    case REG_CQR:
        s->cqr |= val;
        fsl_sata_run(s);
        break;
    case REG_CAR:
        break;
    case REG_CCR:
        s->ccr &= ~val;
        break;
    case REG_CER:
        s->cer &= ~val;
        break;
    case REG_DER:
        s->der &= ~val;
        break;
    case REG_CHBA:
        s->chba = val;
        break;
    case REG_HSTATUS:
        s->hstatus &= ~(val & (HSTATUS_INT_MASK | HSTATUS_INT_DATA_LEN));
        fsl_sata_update_irq(s);
        break;
    case REG_HCONTROL:
        if (val & HCONTROL_CLEAR_ERROR) {
            s->hstatus &= ~(HSTATUS_INT_FATAL | HSTATUS_INT_DEVICE_ERR |
                            HSTATUS_INT_DATA_LEN);
        }
        s->hcontrol = val & ~HCONTROL_CLEAR_ERROR;
        fsl_sata_set_online(s, (val & HCONTROL_ONLINE_PHY_RST) &&
                               !(val & HCONTROL_FORCE_OFFLINE));
        fsl_sata_update_irq(s);
        break;
    case REG_CQPMP:
        s->cqpmp = val;
        break;
    case REG_ICC:
        s->icc = val;
        break;
    case REG_SERROR:
        s->serror &= ~val;
        break;
    case REG_SCONTROL:
        if ((val & SCR_DET_MASK) == SCR_DET_COMRESET) {
            /* COMRESET held: the link is down until DET is released */
            s->sstatus = 0;
        } else if ((s->scontrol & SCR_DET_MASK) == SCR_DET_COMRESET &&
                   (s->hstatus & HSTATUS_ONLINE)) {
            fsl_sata_link_reset(s, IDE_RESET_HARDWARE);
        }
        s->scontrol = val;
        break;
    case REG_SNOTIFY:
        s->snotify &= ~val;
        break;
    case REG_TRANSCFG:
        s->transcfg = val;
        break;
    case REG_LINKCFG:
        s->linkcfg = val;
        break;
    case REG_LINKCFG1:
        s->linkcfg1 = val;
        break;
    case REG_LINKCFG2:
        s->linkcfg2 = val;
        break;
    case REG_PHYCTRLCFG:
        s->phyctrlcfg = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "fsl-sata: write of 0x%08x to register "
                      "0x%03x\n", val, (unsigned)addr);
        break;
    }
}

static const MemoryRegionOps fsl_sata_ops = {
    .read = fsl_sata_read,
    .write = fsl_sata_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* Device                                                              */

static void fsl_sata_reset(DeviceState *dev)
{
    FslSataState *s = FSL_SATA(dev);

    s->cqr = s->car = s->ccr = s->cer = s->der = 0;
    s->chba = 0;
    s->hstatus = 0;
    s->hcontrol = 0;
    s->cqpmp = 0;
    s->sig = 0;
    s->icc = 0;
    s->sstatus = 0;
    s->serror = 0;
    s->scontrol = 0;
    s->snotify = 0;
    s->transcfg = 0;
    s->linkcfg = s->linkcfg1 = s->linkcfg2 = 0;
    s->phyctrlcfg = 0;
    s->cur_desc = 0;
    s->cur_info = 0;
    s->cur_prde = 0;
    s->cur_cmd = 0;
    s->cur_is_write = false;
    s->done_first_drq = false;
    s->srst = false;
    s->busy_slot = -1;

    /* All of the above is in vmstate, so a stale BH must not survive */
    if (s->run_bh) {
        qemu_bh_cancel(s->run_bh);
    }

    ide_bus_reset(&s->bus, IDE_RESET_HARDWARE);
    fsl_sata_update_irq(s);
}

static void fsl_sata_init(Object *obj)
{
    FslSataState *s = FSL_SATA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &fsl_sata_ops, s, "fsl-sata",
                          FSL_SATA_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
}

static void fsl_sata_realize(DeviceState *dev, Error **errp)
{
    FslSataState *s = FSL_SATA(dev);

    s->as = &address_space_memory;
    s->busy_slot = -1;
    s->run_bh = qemu_bh_new_guarded(fsl_sata_run_bh, s,
                                    &dev->mem_reentrancy_guard);

    ide_bus_init(&s->bus, sizeof(s->bus), dev, 0, 1);
    ide_bus_init_output_irq(&s->bus, qemu_allocate_irq(fsl_sata_bus_irq, s, 0));
    s->bus.dma = &s->dma;
    s->dma.ops = &fsl_sata_dma_ops;
    ide_bus_register_restart_cb(&s->bus);
}

static const VMStateDescription vmstate_fsl_sata = {
    .name = "fsl-sata",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_IDE_BUS(bus, FslSataState),
        VMSTATE_IDE_DRIVE(bus.ifs[0], FslSataState),
        VMSTATE_UINT32(cqr, FslSataState),
        VMSTATE_UINT32(car, FslSataState),
        VMSTATE_UINT32(ccr, FslSataState),
        VMSTATE_UINT32(cer, FslSataState),
        VMSTATE_UINT32(der, FslSataState),
        VMSTATE_UINT32(chba, FslSataState),
        VMSTATE_UINT32(hstatus, FslSataState),
        VMSTATE_UINT32(hcontrol, FslSataState),
        VMSTATE_UINT32(cqpmp, FslSataState),
        VMSTATE_UINT32(sig, FslSataState),
        VMSTATE_UINT32(icc, FslSataState),
        VMSTATE_UINT32(sstatus, FslSataState),
        VMSTATE_UINT32(serror, FslSataState),
        VMSTATE_UINT32(scontrol, FslSataState),
        VMSTATE_UINT32(snotify, FslSataState),
        VMSTATE_UINT32(transcfg, FslSataState),
        VMSTATE_UINT32(linkcfg, FslSataState),
        VMSTATE_UINT32(linkcfg1, FslSataState),
        VMSTATE_UINT32(linkcfg2, FslSataState),
        VMSTATE_UINT32(phyctrlcfg, FslSataState),
        VMSTATE_INT32(busy_slot, FslSataState),
        VMSTATE_UINT32(cur_desc, FslSataState),
        VMSTATE_UINT32(cur_info, FslSataState),
        VMSTATE_UINT16(cur_prde, FslSataState),
        VMSTATE_UINT8(cur_cmd, FslSataState),
        VMSTATE_BOOL(cur_is_write, FslSataState),
        VMSTATE_BOOL(done_first_drq, FslSataState),
        VMSTATE_BOOL(srst, FslSataState),
        VMSTATE_END_OF_LIST()
    },
};

static void fsl_sata_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = fsl_sata_realize;
    dc->vmsd = &vmstate_fsl_sata;
    device_class_set_legacy_reset(dc, fsl_sata_reset);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    /* Created by the SoC model, the MMIO block and IRQ have no defaults */
    dc->user_creatable = false;
}

static const TypeInfo fsl_sata_types[] = {
    {
        .name          = TYPE_FSL_SATA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(FslSataState),
        .instance_init = fsl_sata_init,
        .class_init    = fsl_sata_class_init,
    },
};

DEFINE_TYPES(fsl_sata_types)
