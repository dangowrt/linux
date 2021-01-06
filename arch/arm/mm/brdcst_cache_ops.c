#include <linux/linkage.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <mach/rps-irq.h>
#include <mach/ipi.h>
#include <mach/rps-timer.h>

extern asmlinkage void local_v6_flush_kern_cache_all(void);
//extern asmlinkage void local_v6_flush_user_cache_all(void);
//extern asmlinkage void local_v6_flush_user_cache_range(unsigned long, unsigned long, unsigned int);
extern asmlinkage void local_v6_coherent_kern_range(unsigned long, unsigned long);
extern asmlinkage void local_v6_coherent_user_range(unsigned long, unsigned long);
extern asmlinkage void local_v6_flush_kern_dcache_area(void *, size_t);
extern asmlinkage void local_v6_dma_inv_range(const void *, const void *);
extern asmlinkage void local_v6_dma_clean_range(const void *, const void *);
extern asmlinkage void local_v6_dma_flush_range(const void *, const void *);

extern asmlinkage void local_cpu_v6_dcache_clean_area(void *, size_t);

extern int gic_set_cpu(unsigned int irq, const struct cpumask *mask_val);

#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
static inline int brdcst_null_fiq_begin(unsigned int cpu)
{
	cpumask_t callmap;
    struct fiq_coherency_communication_s* comms;
#ifdef CONFIG_FIQ_TIMEOUTS
	unsigned long start_ticks;
#endif // CONFIG_FIQ_TIMEOUTS
	int need_end = 0;

	// We can support a maximum of 2 CPUs
	BUG_ON(cpu > 1);

	// Find the other CPU
	callmap = cpu_online_map;
	cpu_clear(cpu, callmap);

    // Get the ipi work structure
    comms = &(per_cpu(fiq_coherency_communication, cpu));

	// If we're here when the other CPU is still processing a previous cache
	// coherency operation then something is wrong
	BUG_ON(comms->nents);

#ifdef CONFIG_FIQS_USE_ACKS
	// Protocol violation if the cache ops ack flag is already set
	BUG_ON(comms->ack);
#endif // CONFIG_FIQS_USE_ACKS

	// Only do this if there is another CPU active
	if (!cpus_empty(callmap)) {
		/* Prepare one memory range */
		comms->type = CACHE_COHERENCY;
		comms->message.cache_coherency.type = DMA_NONE;
		comms->nents = 1;
		smp_wmb();

#ifdef CONFIG_SERIALIZE_FIQS
		// Wait for exclusive access to the ability to raise a FIQ
		while (test_and_set_bit(CACHE_BRDCST_PRIV_FLAG, &cache_brdcst_priviledge));
#endif // CONFIG_SERIALIZE_FIQS

		// Inform the other CPU that there's per-CPU data to examine
		OX820_RPS_trigger_fiq((cpu == 0) ? 1 : 0);

#ifdef CONFIG_FIQS_USE_ACKS
		// Wait for the other CPU to enter the FIQ handler so no strex can occur
		// during the cache operations
#ifdef CONFIG_FIQ_TIMEOUTS
		start_ticks = *((volatile unsigned long*)TIMER2_VALUE);
#endif // CONFIG_FIQ_TIMEOUTS
		smp_rmb();
		while (!comms->ack) {
#ifdef CONFIG_FIQ_TIMEOUTS
			unsigned long elapsed_ticks;
			unsigned long now_ticks = *((volatile unsigned long*)TIMER2_VALUE);

			if (now_ticks > start_ticks) {
				elapsed_ticks = (TIMER_2_LOAD_VALUE - now_ticks) + start_ticks;
			} else {
				elapsed_ticks = start_ticks - now_ticks;
			}

			if (elapsed_ticks > TIMER_2_PRESCALED_CLK) {
				// Try to force a debug message out using serial interrupts on this
				// known functional CPU
				gic_set_cpu(55, cpumask_of(cpu));
				printk(KERN_WARNING "brdcst_null_fiq_begin() Wait for FIQ ack took longer "
					"than 1 second, giving up (CPU %d, ticks 0x%p)\n", cpu,
					(void*)elapsed_ticks);
				break;
			}
#endif // CONFIG_FIQ_TIMEOUTS
			smp_rmb();
		}
#endif // CONFIG_FIQS_USE_ACKS

		// Need to synchronise with other CPU after local cache ops finished
		need_end = 1;
	}
	
	return need_end;
}

static inline void brdcst_null_fiq_end(unsigned int cpu)
{
    struct fiq_coherency_communication_s* comms;
#ifdef CONFIG_FIQ_TIMEOUTS
	unsigned long start_ticks;
#endif // CONFIG_FIQ_TIMEOUTS

    // Get the ipi work structure
    comms = &(per_cpu(fiq_coherency_communication, cpu));

#ifdef CONFIG_FIQS_USE_ACKS
	// Signal the other CPU that we have completed our cache ops
	comms->ack = 0;
	smp_wmb();
#endif // CONFIG_FIQS_USE_ACKS

	// Rendezvous the two cpus here
#ifdef CONFIG_FIQ_TIMEOUTS
	start_ticks = *((volatile unsigned long*)TIMER2_VALUE);
#endif // CONFIG_FIQ_TIMEOUTS
	smp_rmb();
	while (comms->nents) {
#ifdef CONFIG_FIQ_TIMEOUTS
		unsigned long elapsed_ticks;
		unsigned long now_ticks = *((volatile unsigned long*)TIMER2_VALUE);

		if (now_ticks > start_ticks) {
			elapsed_ticks = (TIMER_2_LOAD_VALUE - now_ticks) + start_ticks;
		} else {
			elapsed_ticks = start_ticks - now_ticks;
		}

		if (elapsed_ticks > TIMER_2_PRESCALED_CLK) {
			// Try to force a debug message out using serial interrupts on this
			// known functional CPU
			gic_set_cpu(55, cpumask_of(cpu));
			printk(KERN_WARNING "brdcst_null_fiq_end() Wait for sync took longer "
				"than 1 second, giving up (CPU %d, ticks 0x%p)\n", cpu,
				(void*)elapsed_ticks);
			break;
		}
#endif // CONFIG_FIQ_TIMEOUTS
		smp_rmb();
	}

#ifdef CONFIG_SERIALIZE_FIQS
	// Relinquish exclusive access to the ability to raise a FIQ
	clear_bit(CACHE_BRDCST_PRIV_FLAG, &cache_brdcst_priviledge);
#endif // CONFIG_SERIALIZE_FIQS
}
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS

static asmlinkage void brdcst_v6_flush_kern_cache_all(void)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_flush_kern_cache_all();
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

static asmlinkage void brdcst_v6_flush_user_cache_all(void)
{
//	local_v6_flush_user_cache_all();	// No implementation for ARM v6
}

static asmlinkage void brdcst_v6_flush_user_cache_range(unsigned long arg1, unsigned long arg2, unsigned int arg3)
{
//	local_v6_flush_user_cache_range(arg1, arg2, arg3);	// No implementation for ARM v6
}

static asmlinkage void brdcst_v6_coherent_kern_range(unsigned long arg1, unsigned long arg2)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_coherent_kern_range(arg1, arg2);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

static asmlinkage void brdcst_v6_coherent_user_range(unsigned long arg1, unsigned long arg2)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_coherent_user_range(arg1, arg2);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

static asmlinkage void brdcst_v6_flush_kern_dcache_area(void *vaddr , size_t size)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_flush_kern_dcache_area(vaddr, size);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

static asmlinkage void brdcst_v6_dma_inv_range(const void *start_vaddr, const void *end_vaddr)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_dma_inv_range(start_vaddr, end_vaddr);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

static asmlinkage void brdcst_v6_dma_clean_range(const void *arg1, const void *arg2)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_dma_clean_range(arg1, arg2);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

static asmlinkage void brdcst_v6_dma_flush_range(const void *arg1, const void *arg2)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_v6_dma_flush_range(arg1, arg2);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}

struct cpu_cache_fns v6_cache_fns = {
	.flush_kern_all         = brdcst_v6_flush_kern_cache_all,
	.flush_user_all         = brdcst_v6_flush_user_cache_all,
	.flush_user_range       = brdcst_v6_flush_user_cache_range,
	.coherent_kern_range    = brdcst_v6_coherent_kern_range,
	.coherent_user_range    = brdcst_v6_coherent_user_range,
	.flush_kern_dcache_area = brdcst_v6_flush_kern_dcache_area, 
	.dma_inv_range          = brdcst_v6_dma_inv_range,
	.dma_clean_range        = brdcst_v6_dma_clean_range,
	.dma_flush_range        = brdcst_v6_dma_flush_range,
};

asmlinkage void brdcst_cpu_v6_dcache_clean_area(void *start, size_t size)
{
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	unsigned long flags;
	unsigned int  cpu;
	int need_end;

	local_irq_save(flags);
	cpu = get_cpu();

	need_end = brdcst_null_fiq_begin(cpu);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
	local_cpu_v6_dcache_clean_area(start, size);
#ifdef CONFIG_BRDCST_LOCAL_CACHE_OPS
	if (need_end) brdcst_null_fiq_end(cpu);

	put_cpu();
	local_irq_restore(flags);
#endif // CONFIG_BRDCST_LOCAL_CACHE_OPS
}