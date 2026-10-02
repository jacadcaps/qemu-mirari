/*
 * QTest for the Mirari machine (Freescale QorIQ T1042)
 *
 * Copyright (c) 2026 Jacek Piszczek
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CCSR_BASE       0xffe000000ULL
#define CCM_LAWBARH(n)  (CCSR_BASE + 0xc00 + (n) * 0x10)
#define CCM_LAWBARL(n)  (CCM_LAWBARH(n) + 4)
#define CCM_LAWAR(n)    (CCM_LAWBARH(n) + 8)
#define DDR_CS0_BNDS    (CCSR_BASE + 0x8000)

#define LAWAR_EN        0x80000000
#define LAWAR_TRGT(v)   (((v) >> 20) & 0xff)
#define LAWAR_SIZE(v)   (2ULL << ((v) & 0x3f))
#define LAW_TRGT_DDR    0x10
#define NR_LAW          16

/*
 * The local access window table and the DDR controller's chip select
 * bounds both tell the guest where DDR ends, and the guest believes both.
 * u-boot sizes the DDR LAWs to the RAM actually fitted, so they always
 * agree; a fixed 2 GB entry left real RAM outside DDR at every larger -m,
 * which the guest was then free to reuse as unallocated address space.
 */
static void test_ddr_law(const void *opaque)
{
    const char *ramsize = opaque;
    QTestState *qts = qtest_initf("-M mirari -m %s", ramsize);
    uint32_t bnds = qtest_readl(qts, DDR_CS0_BNDS);
    uint64_t ram = ((uint64_t)(bnds & 0xffff) + 1) << 24;
    uint64_t covered = 0;
    int n;

    /* CS0 starts at 0: start address in 31:16, end address in 15:0, >> 24 */
    g_assert_cmphex(bnds >> 16, ==, 0);

    for (n = 0; n < NR_LAW; n++) {
        uint32_t attr = qtest_readl(qts, CCM_LAWAR(n));
        uint64_t base, size;

        if (!(attr & LAWAR_EN) || LAWAR_TRGT(attr) != LAW_TRGT_DDR) {
            continue;
        }
        base = ((uint64_t)qtest_readl(qts, CCM_LAWBARH(n)) << 32) |
               qtest_readl(qts, CCM_LAWBARL(n));
        size = LAWAR_SIZE(attr);

        /* each window is aligned to its own size ... */
        g_assert_cmphex(base & (size - 1), ==, 0);
        /* ... and together they tile DDR from 0 up with no gap or overlap */
        g_assert_cmphex(base, ==, covered);
        covered += size;
    }

    g_assert_cmphex(covered, ==, ram);
    qtest_quit(qts);
}

#define DCFG_SVR        (CCSR_BASE + 0xe00a4)
#define L2_L2CSR0       (CCSR_BASE + 0xc20000)
#define L2_L2CFG0       (CCSR_BASE + 0xc20008)

/*
 * -cpu selects the SoC: the T1042 for e5500 and the T2081 for e6500, the
 * latter with the e6500's memory-mapped cluster L2 (enabled, 2 MB).  The
 * unimplemented-device fallback reads zero for the block the T1042 has
 * not got.
 */
static void test_soc_identity(void)
{
    QTestState *qts;

    qts = qtest_init("-M mirari");
    g_assert_cmphex(qtest_readl(qts, DCFG_SVR), ==, 0x85200211);
    g_assert_cmphex(qtest_readl(qts, L2_L2CFG0), ==, 0);
    qtest_quit(qts);

    qts = qtest_init("-M mirari -cpu e6500");
    g_assert_cmphex(qtest_readl(qts, DCFG_SVR), ==, 0x85310011);
    g_assert_cmphex(qtest_readl(qts, L2_L2CSR0) & 0x80000000, ==, 0x80000000);
    g_assert_cmphex(qtest_readl(qts, L2_L2CFG0) & 0x3fff, ==, 0x20);
    /* flash invalidate completes at once, the enable bits stay */
    qtest_writel(qts, L2_L2CSR0, 0xc0200400);
    g_assert_cmphex(qtest_readl(qts, L2_L2CSR0), ==, 0xc0000000);
    qtest_quit(qts);

    qts = qtest_init("-M mirari2");
    g_assert_cmphex(qtest_readl(qts, DCFG_SVR), ==, 0x85310011);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const char *const sizes[] = {
        "1G", "2G", "3G", "4G", "1536M", "2064M"
    };
    int i;

    g_test_init(&argc, &argv, NULL);

    for (i = 0; i < ARRAY_SIZE(sizes); i++) {
        g_autofree char *name = g_strdup_printf("mirari/ddr-law/%s", sizes[i]);
        qtest_add_data_func(name, sizes[i], test_ddr_law);
    }
    qtest_add_func("mirari/soc-identity", test_soc_identity);

    return g_test_run();
}
