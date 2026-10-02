/*
 * The per-CPU TranslationBlock jump cache.
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ACCEL_TCG_TB_JMP_CACHE_H
#define ACCEL_TCG_TB_JMP_CACHE_H

#include "qemu/rcu.h"
#include "exec/cpu-common.h"
#include "qemu/atomic.h"

#define TB_JMP_CACHE_BITS 12
#define TB_JMP_CACHE_SIZE (1 << TB_JMP_CACHE_BITS)

/*
 * tb_jmp_cache_hash_func() splits the hash so that every guest page's
 * translations land in one aligned run of TB_JMP_CACHE_PAGE_SIZE entries --
 * the low bits of the hash come from within the page, the high bits from the
 * page number alone.  So a run can be retired as a unit.  Kept here rather
 * than with the hash itself in tb-hash.h, which includes this header.
 */
#define TB_JMP_CACHE_PAGE_BITS (TB_JMP_CACHE_BITS / 2)
#define TB_JMP_CACHE_PAGE_SIZE (1 << TB_JMP_CACHE_PAGE_BITS)
#define TB_JMP_CACHE_PAGES     (TB_JMP_CACHE_SIZE / TB_JMP_CACHE_PAGE_SIZE)

/*
 * Invalidated in parallel; all accesses to 'tb' must be atomic.
 * A valid entry is read/written by a single CPU, therefore there is
 * no need for qatomic_rcu_read() and pc is always consistent with a
 * non-NULL value of 'tb'.  Strictly speaking pc is only needed for
 * CF_PCREL, but it's used always for simplicity.
 *
 * An entry is live only while its 'gen' equals the generation of the run it
 * sits in.  Emptying a run -- which is what invalidating one guest page's
 * translations comes to -- is then an increment rather than 64 stores, and
 * emptying the whole cache is 64 increments rather than 4096 stores.  Both
 * matter for a guest that reloads its own TLB in software: it does one or
 * the other on every refill, and that was 10-16 % of the vCPU on a Book E
 * browser workload.  'gen' must be tested before 'tb' is dereferenced, since
 * a stale entry can hold a pointer to a block that has since been freed.
 */
typedef struct CPUJumpCache {
    struct rcu_head rcu;
    uint32_t gen[TB_JMP_CACHE_PAGES];
    struct {
        TranslationBlock *tb;
        vaddr pc;
        uint32_t gen;
    } array[TB_JMP_CACHE_SIZE];
} CPUJumpCache;

/* The generation an entry at @hash has to carry to be live. */
static inline uint32_t *tb_jmp_cache_gen(CPUJumpCache *jc, uint32_t hash)
{
    return &jc->gen[hash >> TB_JMP_CACHE_PAGE_BITS];
}

/*
 * Retire every entry in one run.  This can run on another CPU's thread
 * (tb_flush(), and invalidating a CF_PCREL block), so the increment has to
 * be atomic: two flushers that lost one of two increments would leave
 * entries from before them live.
 */
static inline void tb_jmp_cache_retire(CPUJumpCache *jc, int run)
{
    if (unlikely(qatomic_fetch_inc(&jc->gen[run]) == UINT32_MAX)) {
        /*
         * Wrapped, so a stamp from 2^32 retirements ago could match again.
         * Empty the run the long way; an entry whose stale stamp does match
         * is then still dead, because its block pointer is NULL.
         */
        int i0 = run * TB_JMP_CACHE_PAGE_SIZE;
        int i;

        for (i = 0; i < TB_JMP_CACHE_PAGE_SIZE; i++) {
            qatomic_set(&jc->array[i0 + i].tb, NULL);
        }
        qatomic_set(&jc->gen[run], 1);
    }
}

#endif /* ACCEL_TCG_TB_JMP_CACHE_H */
