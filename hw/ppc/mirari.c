/*
 * QEMU model of the Mirari board: a QorIQ T1042 (4x e5500) SoC, or with
 * -cpu e6500 its T2081 (4x e6500) sibling.
 *
 * The SoC blocks the board firmware and boot image touch, with the
 * u-boot state those paths rely on provided at reset:
 *
 *   CoreNet CCM       CCSR+0x0       CCSRBAR, boot space translation
 *                                    (BSTRH/BSTRL/BSTAR), LAWs at +0xC00
 *   DDR controllers   CCSR+0x8000    CSn_BNDS/CSn_CONFIG (RAM sizing)
 *   MPIC              CCSR+0x40000   FSL MPIC 4.2
 *   DCFG              CCSR+0xE0000   BRR (core release)
 *   Clocking          CCSR+0xE1000   PLLC1GSR (CPU frequency)
 *   RCPM              CCSR+0xE2000   core nap set/clear
 *   I2C 1-4           CCSR+0x118000  EEPROM with the board EUI, DS1339 RTC,
 *                                    ADT7461 temperature sensor (EMC1413 model)
 *   DUART             CCSR+0x11C500  two ns16550
 *   SATA 1-2          CCSR+0x220000  SoC SATA controllers
 *   PEX 1-4           CCSR+0x240000  PCI Express controllers
 *   Board CPLD        0xF:FFDF0000   version / slot occupancy
 *
 * CCSR sits at 0xF:FE000000 (36-bit); 0xFE000000 is the effective address
 * u-boot maps it at.  Everything else in the CCSR reads as zero and
 * swallows writes (logged with -d unimp).
 *
 * Boot: -kernel takes the board boot image, a 32-bit ELF loaded the way
 * u-boot's "bootelf" loads it.  With -append the command line is handed
 * over through the Hyperbootloader tag list.
 *
 * SMP: BSTAR points the boot space translation window at a boot page, the
 * core is released through DCFG BRR (or an MPIC processor-init pulse) and
 * takes its reset fetch from EA 0xFFFFF000 mapped to that page.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "exec/target_page.h"
#include "cpu.h"
#include "cpu-models.h"
#include "elf.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/or-irq.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/ide/fsl-sata.h"
#include "hw/nvram/eeprom_at24c.h"
#include "hw/misc/unimp.h"
#include "migration/vmstate.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci-host/mirari_pex.h"
#include "hw/usb/usb.h"
#include "hw/ppc/openpic.h"
#include "hw/ppc/ppc.h"
#include "net/net.h"
#include "system/blockdev.h"
#include "system/hw_accel.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"
#include "system/physmem.h"
#include "system/qtest.h"
#include "qom/object.h"

#define TYPE_MIRARI_MACHINE MACHINE_TYPE_NAME("mirari")
OBJECT_DECLARE_SIMPLE_TYPE(MirariMachineState, MIRARI_MACHINE)

/* 36-bit physical map */
#define MIRARI_CCSR_BASE        0xffe000000ULL
#define MIRARI_CCSR_SIZE        (16 * MiB)
#define MIRARI_CPLD_BASE        0xfffdf0000ULL
#define MIRARI_PEX_IO_BASE      0xff8000000ULL
#define MIRARI_PEX_MEM_BASE     0xc00000000ULL
#define MIRARI_MAX_RAM          (4 * GiB)

/* Effective addresses u-boot maps these blocks at */
#define MIRARI_CCSR_EA          0xfe000000
#define MIRARI_CPLD_EA          0xffdf0000
#define MIRARI_PEX_IO_EA        0xf8000000

/* CCSR block offsets */
#define CCSR_CCM_OFFSET         0x0
#define CCSR_DDR1_OFFSET        0x8000
#define CCSR_DDR2_OFFSET        0x9000
#define CCSR_MPIC_OFFSET        0x40000
#define CCSR_I2C1_OFFSET        0x118000
#define CCSR_I2C2_OFFSET        0x118100
#define CCSR_I2C3_OFFSET        0x119000
#define CCSR_I2C4_OFFSET        0x119100
#define CCSR_DUART1_OFFSET      0x11c500
#define CCSR_DUART2_OFFSET      0x11c600
#define CCSR_DCFG_OFFSET        0xe0000
#define CCSR_CLK_OFFSET         0xe1000
#define CCSR_RCPM_OFFSET        0xe2000
#define CCSR_SATA1_OFFSET       0x220000
#define CCSR_SATA2_OFFSET       0x221000
#define CCSR_PEX_OFFSET         0x240000
#define CCSR_PEX_STRIDE         0x10000

#define MIRARI_NR_PEX           4
#define MIRARI_NR_LAW           16
#define MIRARI_NR_DDR_LAW       5
#define MIRARI_NR_CS            4
#define MIRARI_MAX_CPUS         4
#define MIRARI_DEFAULT_IMAGE    "bootloadermirarirom.img"

/* MPIC: FSL internal interrupt n is openpic source 16 + n */
#define MPIC_INT(n)             (16 + (n))
#define MPIC_IRQ_PEX(n)         MPIC_INT(24 + (n))
#define MPIC_IRQ_DUART          MPIC_INT(30)
#define MPIC_IRQ_I2C            MPIC_INT(38)
#define MPIC_IRQ_SATA(n)        MPIC_INT(52 + (n))

#define CCSR_L2_CLUSTER_OFFSET  0xc20000

/*
 * SoC identity, selected by -cpu.  The board is the same; the e6500 build
 * of it is a T2081, a four-core e6500 QorIQ part, whose CCSR blocks the
 * boot image uses all sit at the T1042 offsets.  PIR reset value: the
 * chassis 2 layout is thread | core << 3 | cluster << 5, one core per
 * cluster on the T104x and four on the T208x.
 */
typedef struct MirariSoC {
    const char *name;
    uint32_t pvr_version;       /* PVR[0:15] of the core that selects it */
    uint32_t pvr;
    uint32_t svr;
    int pir_shift;              /* PIR reset value = core << pir_shift */
    bool cluster_l2;            /* memory-mapped cluster L2 at +0xC20000 */
    int cpu_pll_mult;           /* core PLL multiplier, x 100 MHz */
} MirariSoC;

static const MirariSoC mirari_socs[] = {
    /* T1042 rev 1.1, e5500 at 1.4 GHz */
    { "T1042", 0x8024, 0x80241021, 0x85200211, 5, false, 14 },
    /* T2081 rev 1.1, e6500 r2.0 at 1.8 GHz */
    { "T2081", 0x8040, 0x80400020, 0x85310011, 3, true, 18 },
};

/* Cluster L2 (e6500): L2CSR0 bits */
#define L2CSR0_L2E              0x80000000
#define L2CSR0_L2PE             0x40000000
#define L2CSR0_L2FI             0x00200000
#define L2CSR0_L2FL             0x00000800
#define L2CSR0_L2LFC            0x00000400
/* L2CFG0: size in 64 KB units, 2 MB on the T2081 */
#define L2CFG0_T2081            0x00000020

/* Clocks */
#define MIRARI_PLATFORM_CLK     600000000
#define MIRARI_TB_FREQ          (MIRARI_PLATFORM_CLK / 16)

/* Hyperbootloader tag list hand-over */
#define HBL_MAGIC1              0x4967e3a0
#define HBL_MAGIC2              0x0edd6a5a
#define HBL_TAG_CMDLINE         1
#define HBL_TAG_CPUFREQ         2
#define HBL_TAGS_ADDR           0x00ff0000
#define HBL_CMDLINE_ADDR        0x00ff0100
#define HBL_CMDLINE_MAX         0xf00

/* Loader stack, below the 0xc00000 image */
#define MIRARI_LOADER_STACK     0x00bffff0

struct MirariMachineState {
    MachineState parent_obj;

    PowerPCCPU *cpu[MIRARI_MAX_CPUS];
    MemoryRegion ccsr;
    MemoryRegion ccm_mr;
    MemoryRegion ddr_mr[2];
    MemoryRegion dcfg_mr;
    MemoryRegion clk_mr;
    MemoryRegion rcpm_mr;
    MemoryRegion l2_mr;
    MemoryRegion cpld_mr;
    DeviceState *mpic;
    const MirariSoC *soc;
    PCIBus *pci_bus;            /* PEX0 secondary bus, "pci.0" */

    /* CoreNet CCM */
    uint32_t bstrh;
    uint32_t bstrl;
    uint32_t bstar;
    uint32_t law[MIRARI_NR_LAW][3];

    /* DCFG */
    uint32_t brr;

    /* RCPM */
    uint32_t nap;

    /* Cluster L2 (T2081 only) */
    uint32_t l2csr0;
    uint32_t l2csr1;

    FslSataState *sata[2];

    /*
     * The T1040 shares one MPIC input between UART1/UART2 and another
     * between the four I2C controllers.  qemu_irq has no OR semantics --
     * the last setter wins -- so the sharing needs a real OR gate in
     * front of the MPIC input.
     */
    OrIRQState duart_orirq;
    OrIRQState i2c_orirq;

    hwaddr entry;
    bool have_cmdline;
};

/* ------------------------------------------------------------------ */
/* CoreNet coherency manager: CCSRBAR, boot space translation, LAWs   */

static uint64_t mirari_ccm_read(void *opaque, hwaddr addr, unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x00:                      /* CCSRBARH */
        return MIRARI_CCSR_BASE >> 32;
    case 0x04:                      /* CCSRBARL */
        return (uint32_t)MIRARI_CCSR_BASE;
    case 0x20:                      /* BSTRH */
        return s->bstrh;
    case 0x24:                      /* BSTRL */
        return s->bstrl;
    case 0x28:                      /* BSTAR */
        return s->bstar;
    case 0xc00 ... 0xcff: {
        int law = (addr - 0xc00) >> 4;
        int reg = (addr & 0xf) >> 2;
        return reg < 3 ? s->law[law][reg] : 0;
    }
    default:
        return 0;
    }
}

static void mirari_ccm_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x20:
        s->bstrh = val & 0xf;
        break;
    case 0x24:
        s->bstrl = val & 0xfffff000;
        break;
    case 0x28:
        s->bstar = val & 0x8fff003f;
        break;
    case 0xc00 ... 0xcff: {
        int law = (addr - 0xc00) >> 4;
        int reg = (addr & 0xf) >> 2;
        if (reg == 0) {
            s->law[law][0] = val & 0xf;
        } else if (reg == 1) {
            s->law[law][1] = val & 0xfffff000;
        } else if (reg == 2) {
            /* LAWARn: EN, TRGT_ID 27:20, SIZE 5:0 */
            s->law[law][2] = val & 0x8ff0003f;
        }
        break;
    }
    default:
        break;
    }
}

static const MemoryRegionOps mirari_ccm_ops = {
    .read = mirari_ccm_read,
    .write = mirari_ccm_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void mirari_law_set(MirariMachineState *s, int n, uint64_t base,
                           int target, int size)
{
    s->law[n][0] = base >> 32;
    s->law[n][1] = (uint32_t)base;
    s->law[n][2] = 0x80000000 | (target << 20) | size;
}

/*
 * u-boot sizes the DDR LAWs to the RAM that is actually fitted: every LAW
 * is a power of two, aligned to its own size and never larger than 2 GB,
 * so a total that is not a power of two takes more than one (3 GB is 2 GB
 * at 0 plus 1 GB at 0x80000000).  Fill in the first @max of them and
 * return how many the size needs.
 *
 * This has to agree with the CS0_BNDS that mirari_ddr_read() derives from
 * the same ram_size: a LAW that overstates DDR describes RAM that is not
 * there, and one that understates it leaves real RAM outside DDR, which
 * the guest is then free to reuse as unallocated address space.
 */
static int mirari_ddr_laws(uint64_t size, uint64_t *base, uint64_t *len,
                           int max)
{
    uint64_t at = 0;
    int n = 0;

    while (size) {
        /* the largest aligned power of two that still fits in the rest */
        uint64_t chunk = at ? (at & -at) : 2 * GiB;

        while (chunk > size) {
            chunk >>= 1;
        }
        if (n < max) {
            base[n] = at;
            len[n] = chunk;
        }
        n++;
        at += chunk;
        size -= chunk;
    }
    return n;
}

static void mirari_ccm_reset(MirariMachineState *s)
{
    uint64_t ddr_base[MIRARI_NR_DDR_LAW], ddr_len[MIRARI_NR_DDR_LAW];
    int i, j, nddr;

    s->bstrh = 0;
    s->bstrl = 0;
    s->bstar = 0;
    memset(s->law, 0, sizeof(s->law));

    /* u-boot's LAW table for the T104xRDB.  Sizes are 2^(n+1). */
    i = 0;
    nddr = mirari_ddr_laws(MACHINE(s)->ram_size, ddr_base, ddr_len,
                           MIRARI_NR_DDR_LAW);
    assert(nddr <= MIRARI_NR_DDR_LAW);                      /* mirari_init() */
    for (j = 0; j < nddr; j++) {                            /* DDR */
        mirari_law_set(s, i++, ddr_base[j], 0x10, ctz64(ddr_len[j]) - 1);
    }
    mirari_law_set(s, i++, 0xf00000000ULL, 0x1d, 21);       /* DCSR */
    mirari_law_set(s, i++, 0xfe8000000ULL, 0x1f, 27);       /* IFC NOR */
    mirari_law_set(s, i++, MIRARI_CPLD_BASE, 0x1f, 15);     /* IFC CPLD */
    mirari_law_set(s, i++, 0xc00000000ULL, 0x00, 28);       /* PEX1 mem */
    mirari_law_set(s, i++, 0xff8000000ULL, 0x00, 15);       /* PEX1 I/O */
    mirari_law_set(s, i++, 0xc20000000ULL, 0x01, 27);       /* PEX2 mem */
    mirari_law_set(s, i++, 0xff8010000ULL, 0x01, 15);       /* PEX2 I/O */
    mirari_law_set(s, i++, 0xc30000000ULL, 0x02, 26);       /* PEX3 mem */
    mirari_law_set(s, i++, 0xff8020000ULL, 0x02, 15);       /* PEX3 I/O */
    mirari_law_set(s, i++, 0xc38000000ULL, 0x03, 26);       /* PEX4 mem */
    mirari_law_set(s, i++, 0xff8030000ULL, 0x03, 15);       /* PEX4 I/O */
}

/* ------------------------------------------------------------------ */
/* DDR controllers: RAM size is reported through the chip select regs  */

static uint64_t mirari_ddr_read(void *opaque, hwaddr addr, unsigned size)
{
    MirariMachineState *s = opaque;
    MachineState *machine = MACHINE(s);
    int ctrl = (addr >= 0x1000);
    uint64_t ram = machine->ram_size;

    addr &= 0xfff;

    if (ctrl != 0) {
        return 0;
    }

    switch (addr) {
    case 0x000:                     /* CS0_BNDS: SA 31:16, EA 15:0, >> 24 */
        return (ram - 1) >> 24;
    case 0x080:                     /* CS0_CONFIG */
        return 0x80014202;          /* EN, 4 row bits ext, 14 rows, 10 cols */
    default:
        return 0;
    }
}

static void mirari_ddr_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
}

static const MemoryRegionOps mirari_ddr_ops = {
    .read = mirari_ddr_read,
    .write = mirari_ddr_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* Aux core release                                                   */

static void mirari_set_tlb(ppcmas_tlb_t *tlb, target_ulong va, hwaddr pa,
                           hwaddr len, uint32_t mas2_flags)
{
    tlb->mas1 = (63 - clz64(len / KiB)) << MAS1_TSIZE_SHIFT;
    tlb->mas1 |= MAS1_VALID | MAS1_IPROT;
    tlb->mas2 = (va & TARGET_PAGE_MASK) | mas2_flags;
    tlb->mas7_3 = pa & TARGET_PAGE_MASK;
    tlb->mas7_3 |= MAS3_UR | MAS3_UW | MAS3_UX | MAS3_SR | MAS3_SW | MAS3_SX;
}

/*
 * A core coming out of reset fetches from EA 0xFFFFF000 through its
 * default 4 KB boot TLB entry; the CoreNet boot space translation
 * redirects that window to the BSTRH/BSTRL page when BSTAR[EN] is set.
 */
static void mirari_core_kick(CPUState *cs, run_on_cpu_data data)
{
    MirariMachineState *s = data.host_ptr;
    CPUPPCState *env = cpu_env(cs);
    ppcmas_tlb_t *tlb;
    hwaddr page;

    cpu_synchronize_state(cs);
    cpu_reset(cs);

    if (s->bstar & 0x80000000) {
        page = ((hwaddr)s->bstrh << 32) | s->bstrl;
    } else {
        page = 0xffffff000ULL;
    }

    tlb = booke206_get_tlbm(env, 1, 0, 0);
    mirari_set_tlb(tlb, 0xfffff000, page, 4 * KiB, 0);
    env->nip = 0xfffffffc;
#ifdef CONFIG_KVM
    env->tlb_dirty = true;
#endif

    cs->halted = 0;
    cs->exception_index = -1;
    cpu_resume(cs);
}

static void mirari_release_core(MirariMachineState *s, int n)
{
    CPUState *cs;

    if (n <= 0 || n >= MIRARI_MAX_CPUS || !s->cpu[n]) {
        return;
    }
    cs = CPU(s->cpu[n]);
    run_on_cpu(cs, mirari_core_kick, RUN_ON_CPU_HOST_PTR(s));
}

/* MPIC processor initialization register (PIR) output, per core */
static void mirari_mpic_core_reset(void *opaque, int n, int level)
{
    MirariMachineState *s = opaque;

    /* the reset line is released: the core restarts on the boot page */
    if (!level) {
        mirari_release_core(s, n);
    }
}

/* ------------------------------------------------------------------ */
/* DCFG: BRR (boot release), SVR mirror, RCW/PORSR mostly zero        */

static uint64_t mirari_dcfg_read(void *opaque, hwaddr addr, unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x0a4:                     /* SVR */
        return s->soc->svr;
    case 0x0e4:                     /* BRR */
        return s->brr;
    default:
        return 0;
    }
}

static void mirari_dcfg_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    MirariMachineState *s = opaque;
    uint32_t old, new;
    int i;

    switch (addr) {
    case 0x0e4:
        old = s->brr;
        new = val & ((1 << MIRARI_MAX_CPUS) - 1);
        s->brr = new | 1;
        for (i = 1; i < MIRARI_MAX_CPUS; i++) {
            if ((new & (1 << i)) && !(old & (1 << i))) {
                mirari_release_core(s, i);
            }
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mirari_dcfg_ops = {
    .read = mirari_dcfg_read,
    .write = mirari_dcfg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Clocking: PLLC1GSR[23:30] is the core cluster PLL multiplier */
static uint64_t mirari_clk_read(void *opaque, hwaddr addr, unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x800:                     /* PLLC1GSR */
        return s->soc->cpu_pll_mult << 1;
    default:
        return 0;
    }
}

static void mirari_clk_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
}

static const MemoryRegionOps mirari_clk_ops = {
    .read = mirari_clk_read,
    .write = mirari_clk_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* RCPM: core nap (PH10) set/clear/status */
static uint64_t mirari_rcpm_read(void *opaque, hwaddr addr, unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x00c:                     /* TPH10SR0 */
    case 0x01c:                     /* TPH10SETR0 */
    case 0x02c:                     /* TPH10CLRR0 */
    case 0x03c:                     /* TPH10PSR0 */
        return s->nap;
    default:
        return 0;
    }
}

static void mirari_rcpm_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x01c:
        s->nap |= val;
        break;
    case 0x02c:
        s->nap &= ~val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mirari_rcpm_ops = {
    .read = mirari_rcpm_read,
    .write = mirari_rcpm_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * Cluster L2 (e6500): the shared L2 is enabled through these registers
 * rather than the e5500's per-core L2CSR0 SPR.  u-boot has enabled it
 * before the boot image runs; the flash invalidate/flush/lock-clear bits
 * complete immediately.
 */
static uint64_t mirari_l2_read(void *opaque, hwaddr addr, unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x0:                       /* L2CSR0 */
        return s->l2csr0;
    case 0x4:                       /* L2CSR1 */
        return s->l2csr1;
    case 0x8:                       /* L2CFG0 */
        return L2CFG0_T2081;
    default:
        return 0;
    }
}

static void mirari_l2_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    MirariMachineState *s = opaque;

    switch (addr) {
    case 0x0:
        s->l2csr0 = val & ~(L2CSR0_L2FI | L2CSR0_L2FL | L2CSR0_L2LFC);
        break;
    case 0x4:
        s->l2csr1 = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mirari_l2_ops = {
    .read = mirari_l2_read,
    .write = mirari_l2_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* Board CPLD ("FPGA"): 16-bit registers                              */

#define CPLD_VERSION            0x0102
#define CPLD_STATUS_PCIE0       (1 << 0)
#define CPLD_STATUS_PCIE1       (1 << 1)
#define CPLD_STATUS_SATA2       (1 << 2)
/* Control register: active-low power and reset */
#define CPLD_CONTROL_POWER      (1 << 0)
#define CPLD_CONTROL_RESET      (1 << 1)

static uint64_t mirari_cpld_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr) {
    case 0x0:
        return CPLD_VERSION;
    case 0x2:
        return CPLD_STATUS_PCIE0 | CPLD_STATUS_SATA2;
    case 0x4:
        return CPLD_CONTROL_POWER | CPLD_CONTROL_RESET;
    default:
        return 0;
    }
}

static void mirari_cpld_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    if (addr == 0x4) {
        if (!(val & CPLD_CONTROL_POWER)) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        } else if (!(val & CPLD_CONTROL_RESET)) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
    }
}

static const MemoryRegionOps mirari_cpld_ops = {
    .read = mirari_cpld_read,
    .write = mirari_cpld_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 2,
    .impl.max_access_size = 2,
};

/* ------------------------------------------------------------------ */
/* CPU reset                                                          */

static void mirari_cpu_reset_sec(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);
    cs->exception_index = EXCP_HLT;
}

static void mirari_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    MirariMachineState *s = MIRARI_MACHINE(qdev_get_machine());
    MachineState *machine = MACHINE(s);
    uint64_t ram = machine->ram_size;
    int n = 0;
    hwaddr base;

    cpu_reset(cs);

    /* SoC state reachable through CCSR */
    mirari_ccm_reset(s);
    s->brr = 1;
    s->nap = 0;
    s->l2csr0 = L2CSR0_L2E | L2CSR0_L2PE;
    s->l2csr1 = 0;

    cs->halted = 0;
    env->gpr[1] = MIRARI_LOADER_STACK;
    env->gpr[3] = 0;
    if (s->have_cmdline) {
        env->gpr[4] = HBL_MAGIC1;
        env->gpr[5] = HBL_MAGIC2;
        env->gpr[6] = HBL_TAGS_ADDR;
    } else {
        env->gpr[4] = 0;
        env->gpr[5] = 0;
        env->gpr[6] = 0;
    }
    env->nip = s->entry;

    /*
     * u-boot's TLB1 as the boot image finds it: RAM identity mapped, plus
     * CCSR, the board CPLD and the PCI I/O space in the upper 36-bit page.
     */
    for (base = 0; base < ram && n < 2; base += GiB) {
        mirari_set_tlb(booke206_get_tlbm(env, 1, 0, n++), base, base,
                       MIN(ram - base, GiB), MAS2_M);
    }
    mirari_set_tlb(booke206_get_tlbm(env, 1, 0, n++), MIRARI_CCSR_EA,
                   MIRARI_CCSR_BASE, MIRARI_CCSR_SIZE, MAS2_I | MAS2_G);
    mirari_set_tlb(booke206_get_tlbm(env, 1, 0, n++), MIRARI_CPLD_EA,
                   MIRARI_CPLD_BASE, 64 * KiB, MAS2_I | MAS2_G);
    mirari_set_tlb(booke206_get_tlbm(env, 1, 0, n++), MIRARI_PEX_IO_EA,
                   MIRARI_PEX_IO_BASE, 256 * KiB, MAS2_I | MAS2_G);
#ifdef CONFIG_KVM
    env->tlb_dirty = true;
#endif
}

/* ------------------------------------------------------------------ */

static void mirari_load_kernel(MirariMachineState *s)
{
    MachineState *machine = MACHINE(s);
    const char *bios_name = machine->firmware ?: MIRARI_DEFAULT_IMAGE;
    g_autofree char *filename = NULL;
    uint64_t entry, loadaddr;
    ssize_t size;

    /*
     * The boot image comes from -kernel, else -bios, else the default name
     * on the firmware search path (-L).
     */
    if (machine->kernel_filename) {
        filename = g_strdup(machine->kernel_filename);
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    }
    if (!filename) {
        if (qtest_enabled()) {
            /*
             * qtest creates the machine with no arguments at all (qom-test,
             * test-hmp) and never runs the CPUs; there is nothing to load.
             */
            return;
        }
        error_report("mirari: no boot image: use -kernel <image>, "
                     "-bios <image> or put '%s' on the -L path",
                     MIRARI_DEFAULT_IMAGE);
        exit(1);
    }

    size = load_elf(filename, NULL, NULL, NULL, &entry,
                    &loadaddr, NULL, NULL, ELFDATA2MSB, PPC_ELF_MACHINE,
                    0, 0);
    if (size < 0) {
        error_report("mirari: could not load '%s' (32-bit big endian "
                     "PowerPC ELF expected)", filename);
        exit(1);
    }
    s->entry = entry;

    if (machine->kernel_cmdline && machine->kernel_cmdline[0]) {
        const char *cmdline = machine->kernel_cmdline;
        size_t len = strlen(cmdline) + 1;
        uint32_t tags[6];

        if (len > HBL_CMDLINE_MAX) {
            error_report("mirari: command line too long");
            exit(1);
        }
        tags[0] = cpu_to_be32(HBL_TAG_CMDLINE);
        tags[1] = cpu_to_be32(HBL_CMDLINE_ADDR);
        tags[2] = cpu_to_be32(HBL_TAG_CPUFREQ);
        tags[3] = cpu_to_be32(s->soc->cpu_pll_mult * 100000000);
        tags[4] = 0;
        tags[5] = 0;
        rom_add_blob_fixed("mirari.tags", tags, sizeof(tags), HBL_TAGS_ADDR);
        rom_add_blob_fixed("mirari.cmdline", cmdline, len, HBL_CMDLINE_ADDR);
        s->have_cmdline = true;
    }
}

static void mirari_init_sata(MirariMachineState *s, MemoryRegion *ccsr)
{
    static const hwaddr offsets[2] = { CCSR_SATA1_OFFSET, CCSR_SATA2_OFFSET };
    int i, port = 0;

    for (i = 0; i < 2; i++) {
        DeviceState *dev = qdev_new(TYPE_FSL_SATA);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        char *name = g_strdup_printf("sata%d", i + 1);

        object_property_add_child(OBJECT(s), name, OBJECT(dev));
        g_free(name);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(s->mpic, MPIC_IRQ_SATA(i)));
        memory_region_add_subregion(ccsr, offsets[i],
                                    sysbus_mmio_get_region(sbd, 0));
        s->sata[i] = FSL_SATA(dev);
    }

    /* One drive per port, in -drive index order (units_per_default_bus 1) */
    for (i = 0; i <= drive_get_max_bus(IF_IDE); i++) {
        DriveInfo *dinfo = drive_get(IF_IDE, i, 0);

        if (!dinfo) {
            continue;
        }
        if (port == ARRAY_SIZE(s->sata)) {
            error_report("mirari: at most two IDE/SATA drives (sata1, sata2)");
            exit(1);
        }
        ide_bus_create_drive(&s->sata[port++]->bus, 0, dinfo);
    }
}

static void mirari_init_i2c(MirariMachineState *s, MemoryRegion *ccsr)
{
    static const hwaddr offsets[4] = {
        CCSR_I2C1_OFFSET, CCSR_I2C2_OFFSET, CCSR_I2C3_OFFSET, CCSR_I2C4_OFFSET
    };
    /*
     * 24AA256UID: EUI-48 at 0x7f7a, EUI-64 at 0x7fb8, ID + 32-bit serial
     * at 0x7ffa.  Locally administered addresses.
     */
    static const uint8_t eui48[6] = { 0x02, 0x4d, 0x49, 0x52, 0x41, 0x52 };
    static const uint8_t eui64[8] = { 0x02, 0x4d, 0x49, 0xff, 0xfe,
                                      0x52, 0x41, 0x52 };
    static const uint8_t serial[6] = { 0x29, 0x00, 0x00, 0x00, 0x00, 0x01 };
    uint8_t *rom;
    int i;

    /* All four controllers share one MPIC input; OR them together */
    object_initialize_child(OBJECT(s), "i2c-orirq", &s->i2c_orirq,
                            TYPE_OR_IRQ);
    object_property_set_int(OBJECT(&s->i2c_orirq), "num-lines", 4,
                            &error_fatal);
    qdev_realize(DEVICE(&s->i2c_orirq), NULL, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->i2c_orirq), 0,
                          qdev_get_gpio_in(s->mpic, MPIC_IRQ_I2C));

    for (i = 0; i < 4; i++) {
        DeviceState *dev = qdev_new("mpc-i2c");
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0,
                           qdev_get_gpio_in(DEVICE(&s->i2c_orirq), i));
        memory_region_add_subregion(ccsr, offsets[i],
                                    sysbus_mmio_get_region(sbd, 0));

        if (i == 0) {
            I2CBus *bus = I2C_BUS(qdev_get_child_bus(dev, "i2c"));

            rom = g_malloc0(32 * KiB);
            memcpy(rom + 0x7f7a, eui48, sizeof(eui48));
            memcpy(rom + 0x7fb8, eui64, sizeof(eui64));
            memcpy(rom + 0x7ffa, serial, sizeof(serial));
            at24c_eeprom_init_rom(bus, 0x57, 32 * KiB, rom, 32 * KiB);
            g_free(rom);

            /* DS1339 RTC: same register map as the DS1338 */
            i2c_slave_create_simple(bus, "ds1338", 0x68);

            /*
             * ADT7461 CPU/board temperature sensor.  The EMC1413 model
             * provides the same LM90-derived register map: local temp at
             * 0x00, remote high/low byte at 0x01/0x10, limits at
             * 0x05..0x08, manufacturer/revision at 0xfe/0xff.  Only the
             * manufacturer ID differs (SMSC 0x5d instead of ADI 0x41).
             */
            dev = DEVICE(i2c_slave_create_simple(bus, "emc1413", 0x4c));
            /* millidegrees: die (remote) and board (local) */
            object_property_set_int(OBJECT(dev), "temperature1", 48000,
                                    &error_abort);
            object_property_set_int(OBJECT(dev), "temperature0", 36000,
                                    &error_abort);
        }
    }
}

static void mirari_init_pex(MirariMachineState *s, MemoryRegion *ccsr)
{
    /* PCI-side memory windows */
    static const struct {
        uint32_t mem_base;
        uint32_t mem_size;
    } pex[MIRARI_NR_PEX] = {
        { 0xa0000000, 0x20000000 },
        { 0xc0000000, 0x10000000 },
        { 0xd0000000, 0x08000000 },
        { 0xd8000000, 0x08000000 },
    };
    int i;

    for (i = 0; i < MIRARI_NR_PEX; i++) {
        DeviceState *dev = qdev_new(TYPE_MIRARI_PEX_HOST);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        char *name = g_strdup_printf("pex%d", i);

        object_property_add_child(OBJECT(s), name, OBJECT(dev));
        g_free(name);
        qdev_prop_set_uint32(dev, "index", i);
        /* Only the first slot is populated: PEX1 is the graphics slot */
        qdev_prop_set_bit(dev, "link-up", i == 0);
        qdev_prop_set_uint64(dev, "io-cpu-base",
                             MIRARI_PEX_IO_BASE + i * 0x10000);
        qdev_prop_set_uint32(dev, "mem-base", pex[i].mem_base);
        qdev_prop_set_uint32(dev, "mem-size", pex[i].mem_size);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(s->mpic, MPIC_IRQ_PEX(i)));
        memory_region_add_subregion(ccsr,
                                    CCSR_PEX_OFFSET + i * CCSR_PEX_STRIDE,
                                    sysbus_mmio_get_region(sbd, 0));

        if (i == 0) {
            PCIBus *bus = PCI_BUS(qdev_get_child_bus(dev, "pex0"));
            PCIDevice *root = pci_find_device(bus, 0, PCI_DEVFN(0, 0));

            s->pci_bus = pci_bridge_get_sec_bus(PCI_BRIDGE(root));
        }
    }
}

/*
 * The devices the board carries, created by default so that no -device
 * lines are needed.  -nodefaults, -vga none, -usb off and -nic none opt
 * out of each piece.
 */
static void mirari_init_default_devices(MirariMachineState *s)
{
    MachineState *machine = MACHINE(s);
    MachineClass *mc = MACHINE_GET_CLASS(s);
    PCIBus *bus = s->pci_bus;

    /* Display: the SM501 is this machine's "std" VGA */
    switch (vga_interface_type) {
    case VGA_NONE:
    case VGA_DEVICE:
        break;
    case VGA_STD:
        pci_create_simple(bus, -1, "sm501");
        vga_interface_created = true;
        break;
    default:
        pci_vga_init(bus);
        break;
    }

    /*
     * -cdrom / -hda / -hdb / -drive if=ide go to the SoC SATA ports;
     * -drive if=scsi gets an lsi53c895a on the PCI bus.
     */
    if (defaults_enabled() && drive_get_max_bus(IF_SCSI) >= 0) {
        DeviceState *dev = DEVICE(pci_create_simple(bus, -1, "lsi53c895a"));

        lsi53c8xx_handle_legacy_cmdline(dev);
    }

    /* OHCI with a USB keyboard and mouse */
    machine->usb |= defaults_enabled() && !machine->usb_disabled;
    if (machine->usb) {
        USBBus *usb_bus;

        pci_create_simple(bus, -1, "pci-ohci");
        usb_bus = USB_BUS(object_resolve_type_unambiguous(TYPE_USB_BUS,
                                                          &error_abort));
        usb_create_simple(usb_bus, "usb-kbd");
        usb_create_simple(usb_bus, "usb-mouse");
    }

    pci_init_nic_devices(bus, mc->default_nic);
}

static const VMStateDescription vmstate_mirari = {
    .name = "mirari",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bstrh, MirariMachineState),
        VMSTATE_UINT32(bstrl, MirariMachineState),
        VMSTATE_UINT32(bstar, MirariMachineState),
        VMSTATE_UINT32_2DARRAY(law, MirariMachineState, MIRARI_NR_LAW, 3),
        VMSTATE_UINT32(brr, MirariMachineState),
        VMSTATE_UINT32(nap, MirariMachineState),
        VMSTATE_UINT32(l2csr0, MirariMachineState),
        VMSTATE_UINT32(l2csr1, MirariMachineState),
        VMSTATE_END_OF_LIST()
    },
};

static void mirari_init(MachineState *machine)
{
    MirariMachineState *s = MIRARI_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ccsr = &s->ccsr;
    unsigned int smp_cpus = machine->smp.cpus;
    IrqLines *irqs;
    DeviceState *dev;
    SysBusDevice *sbd;
    int i, j, k;

    if (machine->ram_size > MIRARI_MAX_RAM) {
        error_report("mirari: at most 4 GB of RAM (36-bit DDR window)");
        exit(1);
    }
    if (!QEMU_IS_ALIGNED(machine->ram_size, 16 * MiB)) {
        error_report("mirari: RAM size must be a multiple of 16 MB");
        exit(1);
    }
    if (mirari_ddr_laws(machine->ram_size, NULL, NULL, 0) >
        MIRARI_NR_DDR_LAW) {
        error_report("mirari: RAM size takes more than the %d local access "
                     "windows kept for DDR; pick a rounder size",
                     MIRARI_NR_DDR_LAW);
        exit(1);
    }

    /* SoC identity from the core type */
    {
        PowerPCCPUClass *pcc =
            POWERPC_CPU_CLASS(object_class_by_name(machine->cpu_type));

        s->soc = NULL;
        for (i = 0; i < ARRAY_SIZE(mirari_socs); i++) {
            if (pcc && mirari_socs[i].pvr_version == (pcc->pvr >> 16)) {
                s->soc = &mirari_socs[i];
            }
        }
        if (!s->soc) {
            error_report("mirari: -cpu must be e5500 (T1042) or e6500 "
                         "(T2081)");
            exit(1);
        }
    }

    /* CPUs */
    irqs = g_new0(IrqLines, smp_cpus);
    for (i = 0; i < smp_cpus; i++) {
        PowerPCCPU *cpu = POWERPC_CPU(object_new(machine->cpu_type));
        CPUState *cs = CPU(cpu);
        CPUPPCState *env = &cpu->env;

        /* Aux cores wait in boot hold-off until DCFG BRR releases them */
        object_property_set_bool(OBJECT(cs), "start-powered-off", i != 0,
                                 &error_abort);
        qdev_realize_and_unref(DEVICE(cs), NULL, &error_fatal);
        s->cpu[i] = cpu;

        irqs[i].irq[OPENPIC_OUTPUT_INT] =
            qdev_get_gpio_in(DEVICE(cpu), PPCE500_INPUT_INT);
        irqs[i].irq[OPENPIC_OUTPUT_CINT] =
            qdev_get_gpio_in(DEVICE(cpu), PPCE500_INPUT_CINT);
        irqs[i].irq[OPENPIC_OUTPUT_RESET] =
            qemu_allocate_irq(mirari_mpic_core_reset, s, i);

        /* PIR reads core# << 5 (T104x) or core# << 3 (T208x) out of reset */
        env->spr_cb[SPR_BOOKE_PIR].default_value = i << s->soc->pir_shift;
        env->spr_cb[SPR_E500_SVR].default_value = s->soc->svr;
        env->spr_cb[SPR_PVR].default_value = s->soc->pvr;
        if (i == 0) {
            /*
             * u-boot leaves the boot core with the decrementer interrupt
             * enabled (TCR[DIE]).  The aux cores come out of hard reset
             * with TCR = 0.
             */
            env->spr_cb[SPR_BOOKE_TCR].default_value = 0x04000000;
        }
        env->mpic_iack = MIRARI_CCSR_BASE + CCSR_MPIC_OFFSET + 0xa0;

        ppc_booke_timers_init(cpu, MIRARI_TB_FREQ, PPC_TIMER_E500);

        if (i == 0) {
            qemu_register_reset(mirari_cpu_reset, cpu);
        } else {
            qemu_register_reset(mirari_cpu_reset_sec, cpu);
        }
    }

    /* RAM */
    memory_region_add_subregion(sysmem, 0, machine->ram);

    /* CCSR */
    memory_region_init(ccsr, OBJECT(s), "mirari.ccsr", MIRARI_CCSR_SIZE);
    memory_region_add_subregion(sysmem, MIRARI_CCSR_BASE, ccsr);

    dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
    qdev_prop_set_string(dev, "name", "ccsr");
    qdev_prop_set_uint64(dev, "size", MIRARI_CCSR_SIZE);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion_overlap(ccsr, 0, sysbus_mmio_get_region(sbd, 0),
                                        -1000);

    memory_region_init_io(&s->ccm_mr, OBJECT(s), &mirari_ccm_ops, s,
                          "mirari.ccm", 0x1000);
    memory_region_add_subregion(ccsr, CCSR_CCM_OFFSET, &s->ccm_mr);

    memory_region_init_io(&s->ddr_mr[0], OBJECT(s), &mirari_ddr_ops, s,
                          "mirari.ddr", 0x2000);
    memory_region_add_subregion(ccsr, CCSR_DDR1_OFFSET, &s->ddr_mr[0]);

    memory_region_init_io(&s->dcfg_mr, OBJECT(s), &mirari_dcfg_ops, s,
                          "mirari.dcfg", 0x1000);
    memory_region_add_subregion(ccsr, CCSR_DCFG_OFFSET, &s->dcfg_mr);

    memory_region_init_io(&s->clk_mr, OBJECT(s), &mirari_clk_ops, s,
                          "mirari.clk", 0x1000);
    memory_region_add_subregion(ccsr, CCSR_CLK_OFFSET, &s->clk_mr);

    memory_region_init_io(&s->rcpm_mr, OBJECT(s), &mirari_rcpm_ops, s,
                          "mirari.rcpm", 0x1000);
    memory_region_add_subregion(ccsr, CCSR_RCPM_OFFSET, &s->rcpm_mr);

    if (s->soc->cluster_l2) {
        memory_region_init_io(&s->l2_mr, OBJECT(s), &mirari_l2_ops, s,
                              "mirari.l2", 0x1000);
        memory_region_add_subregion(ccsr, CCSR_L2_CLUSTER_OFFSET, &s->l2_mr);
    }

    /* Board CPLD */
    memory_region_init_io(&s->cpld_mr, OBJECT(s), &mirari_cpld_ops, s,
                          "mirari.cpld", 64 * KiB);
    memory_region_add_subregion(sysmem, MIRARI_CPLD_BASE, &s->cpld_mr);

    /* MPIC */
    dev = qdev_new(TYPE_OPENPIC);
    object_property_add_child(OBJECT(machine), "pic", OBJECT(dev));
    qdev_prop_set_uint32(dev, "model", OPENPIC_MODEL_FSL_MPIC_42);
    qdev_prop_set_uint32(dev, "nb_cpus", smp_cpus);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    k = 0;
    for (i = 0; i < smp_cpus; i++) {
        for (j = 0; j < OPENPIC_OUTPUT_NB; j++) {
            sysbus_connect_irq(sbd, k++, irqs[i].irq[j]);
        }
    }
    g_free(irqs);
    memory_region_add_subregion(ccsr, CCSR_MPIC_OFFSET,
                                sysbus_mmio_get_region(sbd, 0));
    s->mpic = dev;

    /* DUART: both UARTs drive the one MPIC input through an OR gate */
    object_initialize_child(OBJECT(s), "duart-orirq", &s->duart_orirq,
                            TYPE_OR_IRQ);
    object_property_set_int(OBJECT(&s->duart_orirq), "num-lines", 2,
                            &error_fatal);
    qdev_realize(DEVICE(&s->duart_orirq), NULL, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->duart_orirq), 0,
                          qdev_get_gpio_in(s->mpic, MPIC_IRQ_DUART));

    serial_mm_init(ccsr, CCSR_DUART1_OFFSET, 0,
                   qdev_get_gpio_in(DEVICE(&s->duart_orirq), 0), 1843200,
                   serial_hd(0), DEVICE_BIG_ENDIAN);
    serial_mm_init(ccsr, CCSR_DUART2_OFFSET, 0,
                   qdev_get_gpio_in(DEVICE(&s->duart_orirq), 1), 1843200,
                   serial_hd(1), DEVICE_BIG_ENDIAN);

    mirari_init_sata(s, ccsr);
    mirari_init_i2c(s, ccsr);
    mirari_init_pex(s, ccsr);
    mirari_init_default_devices(s);

    mirari_load_kernel(s);

    vmstate_register(NULL, 0, &vmstate_mirari, s);
}

static void mirari_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Mirari (QorIQ T1042, 4x e5500; -cpu e6500 for a T2081)";
    mc->init = mirari_init;
    mc->max_cpus = MIRARI_MAX_CPUS;
    mc->default_cpus = MIRARI_MAX_CPUS;     /* the T1042 has four cores */
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("e5500");
    mc->default_ram_size = 1 * GiB;
    mc->default_ram_id = "mirari.ram";
    mc->default_nic = "rtl8139";
    mc->default_display = "std";            /* SM501 */
    mc->no_floppy = 1;
    mc->no_cdrom = 1;           /* no empty default CD on a SATA port */
    mc->no_parallel = 1;
    mc->block_default_type = IF_IDE;         /* SoC SATA ports */
    mc->units_per_default_bus = 1;
}

/* The same board with the e6500 SoC as the default: -M mirari2 */
static void mirari2_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Mirari (QorIQ T2081, 4x e6500)";
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("e6500");
}

static const TypeInfo mirari_machine_types[] = {
    {
        .name          = TYPE_MIRARI_MACHINE,
        .parent        = TYPE_MACHINE,
        .instance_size = sizeof(MirariMachineState),
        .class_init    = mirari_machine_class_init,
    },
    {
        .name          = MACHINE_TYPE_NAME("mirari2"),
        .parent        = TYPE_MIRARI_MACHINE,
        .class_init    = mirari2_machine_class_init,
    },
};

DEFINE_TYPES(mirari_machine_types)
