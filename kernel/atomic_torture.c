#include "linux/init.h"
#include "linux/module.h"
#include "linux/kthread.h"
#include "linux/delay.h"

#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <mach/prealloc_init.h>
#include <asm/uaccess.h>
#include "mach/hardware.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <agreen@plxtech.com>");


//#define PNT printk(KERN_DEBUG "PNT %s/%u\n", __FILE__,__LINE__)
#define PNT 

#define USE_PRIVATE_PROTOTYPES	1 //0 // 1		// use the private atomic/spin routines with logging. Otherwise, use the normal atomic_XX, spin_XX methods
#define PLX_LOG_CPU_ACTIVITY	1
#define USE_UNCACHED_MEMORY	0 // 1
#define APPLY_FIX		0 // 1 //0 // 1
#define SIM_ONLY		0
#define CLRINVAL		0//1
#define CLRINVAL2		1


#if SIM_ONLY

#define PRINT(fmt, args...)
#define MY_BAD(fmt, args...)		// trigger sim event?
#define ALERT_IF(arg)  if (arg) MY_BAD("Ping!")

#else

#define PRINT(fmt, args...) printk(KERN_INFO fmt, ## args)
#define MY_BAD PRINT
#define ALERT_IF WARN_ON

#endif


// **** copied and adapted from arch/arm/include/asm/atomic.h:-
//   - basically the same methods, just return what the value was before we modified it

#if APPLY_FIX
static int junk[16384];
static inline void InsertFix(void * addr)
{
	int a,b;
	
	addr = &junk[raw_smp_processor_id() * 8192];

	__asm__ __volatile__("@ InsertFix\n"
#if 1
"	mcr p15, 0, %2, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#elif 0
"	ldr	%0, [%2]\n"
"	str	%0, [%2]\n"
"	ldr	%0, [%2]\n"
"	str	%0, [%2]\n"
#elif 0
"	strex	%1, %0, [%2]\n"
"	strex	%1, %0, [%2]\n"
#elif 0
"	ldrex	%0, [%2]\n"
"	strex	%1, %0, [%2]\n"
"	ldrex	%0, [%2]\n"
"	strex	%1, %0, [%2]\n"
#elif 0
"	nop\n"
"	nop\n"
"	nop\n"
"	nop\n"
#else
"	ldrex	%0, [%2]\n"
"	clrex\n"
"	ldrex	%0, [%2]\n"
"	clrex\n"
#endif
	: "=&r"(a), "=&r"(b)
	: "r"(addr)
	: "cc");
}
#else
#define InsertFix(a);
#endif

static inline void atomic_set_was(atomic_t *v, int i, int * was)
{
	int tmp;

	InsertFix(v);

	__asm__ __volatile__("@ atomic_set_was\n"
"1:\n"
#if CLRINVAL
"	mcr p15, 0, %1, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#endif
"	ldrex	%0, [%1]\n"
"	strex	%0, %2, [%1]\n"
"	teq	%0, #0\n"
"	bne	1b\n"
#if CLRINVAL2
"	mcr p15, 0, %1, c7, c14, 1\n"
#endif
	: "=&r" (tmp)
	: "r" (&v->counter), "r" (i)
	: "cc");
	
	*was = tmp;
}

static inline int atomic_get_safe(atomic_t * v)
{
	int tmp;
	__asm__ __volatile__("@ atomic_get_safe\n"
"	ldrex	%0, [%1]\n"
"	clrex\n"
	: "=&r" (tmp)
	: "r" (&v->counter)
	: "cc");

	return tmp;
}


static inline void atomic_add_was(int i, atomic_t *v, int * was)
{
	unsigned long tmp;
	int result;

	InsertFix(v);

	__asm__ __volatile__("@ atomic_add_was\n"
"1:\n"
#if CLRINVAL
"	mcr p15, 0, %3, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#endif
"	ldrex	%0, [%3]\n"
"	str	%0, [%2]\n"
"	add	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b\n"
#if CLRINVAL2
"	mcr p15, 0, %3, c7, c14, 1\n"
#endif
	: "=&r" (result), "=&r" (tmp)
	: "r"(was), "r" (&v->counter), "Ir" (i)
	: "cc");
}

static inline int atomic_add_return_was(int i, atomic_t *v, int * was)
{
	unsigned long tmp;
	int result;

	smp_mb();

	InsertFix(v);

	__asm__ __volatile__("@ atomic_add_return_was\n"
"1:\n"
#if CLRINVAL
"	mcr p15, 0, %3, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#endif
"	ldrex	%0, [%3]\n"
"	str	%0, [%2]\n"
"	add	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b\n"
#if CLRINVAL2
"	mcr p15, 0, %3, c7, c14, 1\n"
#endif
	: "=&r" (result), "=&r" (tmp)
	: "r"(was), "r" (&v->counter), "Ir" (i)
	: "cc");

	smp_mb();

	return result;
}

static inline void atomic_sub_was(int i, atomic_t *v, int * was)
{
	unsigned long tmp;
	int result;

	InsertFix(v);

	__asm__ __volatile__("@ atomic_sub_was\n"
"1:\n"
#if CLRINVAL
"	mcr p15, 0, %3, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#endif
"	ldrex	%0, [%3]\n"
"	str	%0, [%2]\n"
"	sub	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b\n"
#if CLRINVAL2
"	mcr p15, 0, %3, c7, c14, 1\n"
#endif
	: "=&r" (result), "=&r" (tmp)
	: "r"(was), "r" (&v->counter), "Ir" (i)
	: "cc");
}

static inline int atomic_sub_return_was(int i, atomic_t *v, int * was)
{
	unsigned long tmp;
	int result;

	smp_mb();

	InsertFix(v);

	__asm__ __volatile__("@ atomic_sub_return_was\n"
"1:\n"
#if CLRINVAL
"	mcr p15, 0, %3, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#endif
"	ldrex	%0, [%3]\n"
"	str	%0, [%2]\n"
"	sub	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b\n"
#if CLRINVAL2
"	mcr p15, 0, %3, c7, c14, 1\n"
#endif
	: "=&r" (result), "=&r" (tmp)
	: "r"(was), "r" (&v->counter), "Ir" (i)
	: "cc");

	smp_mb();

	return result;
}


static inline int atomic_cmpxchg_was(atomic_t *ptr, int old, int new, int * was)
{
	unsigned long oldval, res;

	smp_mb();

	InsertFix(ptr);

	do {
		__asm__ __volatile__("@ atomic_cmpxchg\n"
#if CLRINVAL
		"mcr p15, 0, %3, c7, c14, 1\n"	// clean and invalidate cache line using MVA
#endif
		"ldrex	%1, [%3]\n"
		"str	%1, [%2]\n"
		"mov	%0, #0\n"
		"teq	%1, %4\n"
		"strexeq %0, %5, [%3]\n"
#if CLRINVAL2
"	mcr p15, 0, %3, c7, c14, 1\n"
#endif
		    : "=&r" (res), "=&r" (oldval)
		    : "r"(was), "r" (&ptr->counter), "Ir" (old), "r" (new)
		    : "cc");
	} while (res);

	smp_mb();

	return oldval;
}

static inline int atomic_add_unless_was(atomic_t *v, int a, int u, int * was)
{
	int c, old;

	c = atomic_read(v);
	while (c != u && (old = atomic_cmpxchg_was((v), c, c + a, was)) != c)
		c = old;
	return c != u;
}


static inline void atomic_add_safe(int i, atomic_t * v)
{
	int x;
	atomic_add_was(i,v,&x);
}
static inline int atomic_add_return_safe(int i, atomic_t * v)
{
	int x;
	return atomic_add_return_was(i,v,&x);
}
static inline void atomic_sub_safe(int i, atomic_t * v)
{
	int x;
	atomic_sub_was(i,v,&x);
}
static inline int atomic_sub_return_safe(int i, atomic_t * v)
{
	int x;
	return atomic_sub_return_was(i,v,&x);
}

#define atomic_inc_not_zero_was(v,was) atomic_add_unless_was((v), 1, 0, was)
#define atomic_dec_and_test_was(v,was)	(atomic_sub_return_was(1, v, was) == 0)



// ****** logging wrappers

// PLX monitor hardware.
#if PLX_LOG_CPU_ACTIVITY

typedef struct _ETM_MON {
    volatile unsigned int ctrl;
    volatile unsigned int address_lo;
    volatile unsigned int address_hi;
    volatile unsigned int clock_count;
    volatile unsigned int hit_count;
    volatile unsigned int miss_count;
    volatile unsigned int zeroing_clock_count;
} ETM_MON, *PETM_MON;

ETM_MON * etmArmA = (ETM_MON *)AHB_MON_ETM_ARMA;
ETM_MON * etmArmB = (ETM_MON *)AHB_MON_ETM_ARMB;

static inline void InitETMs(void)
{
	etmArmA->address_lo = 0;
	etmArmA->address_hi = 0xFFFFFFFF;
	etmArmA->ctrl = 3; // zero, enable
	etmArmB->address_lo = 0;
	etmArmB->address_hi = 0xFFFFFFFF;
	etmArmB->ctrl = 3; // zero, enable
}


static inline __u32 SampleETM(int cpu)
{
	ETM_MON * etm = (cpu) ? etmArmB : etmArmA;

	int ret = etm->hit_count;
	if (ret >= 0xFFFFFFE0) {
		etm->ctrl = 3;
	}
	return ret;
}

static inline __u32 CycleCount(int cpu)
{
	ETM_MON * etm = (cpu) ? etmArmB : etmArmA;

	int ret = etm->clock_count;
	if (ret >= 0xFFFFFFE0) {
		etm->ctrl = 3;
	}
	return ret;
}

#else
#define InitETMs()
#define SampleETM(a) 	0
#define CycleCount(a)	0
#endif


struct LogEntry
{
	__u32 quad[8];
};

#define ATOMICLOGSIZE 16384
#if !USE_PRIVATE_PROTOTYPES // no logging, so keep output small
 #undef  ATOMICLOGSIZE
 #define ATOMICLOGSIZE 512
#endif


#define ENTRIES_IN_LOG (((ATOMICLOGSIZE / sizeof(struct LogEntry)) - 1))

struct PerCpuLog
{
	struct LogEntry entry[ENTRIES_IN_LOG];
	int index;
	long iterations;
} __attribute__((__aligned__( ATOMICLOGSIZE )));


static struct PerCpuLog * atomicLogs[CONFIG_NR_CPUS] = { NULL };

static inline void InitLogs(void)
{
	int i;
	for (i=0 ; i<CONFIG_NR_CPUS ; ++i)
	{
		atomicLogs[i]->index = 0;
		atomicLogs[i]->iterations = 0LL;
	}
}

static inline struct LogEntry * Next(int cpu)
{
	return &atomicLogs[cpu]->entry[atomicLogs[cpu]->index];
}

static inline void MoveOn(int cpu)
{
	if (++atomicLogs[cpu]->index >= ENTRIES_IN_LOG)
	{
		atomicLogs[cpu]->index = 0;
	}
}

inline void PostLog(struct LogEntry * loc, char a, char b, __u32 was, __u32 now, void * p, __u32 starta, __u32 enda, __u32 startb, __u32 endb)
{
	// log in 8-quadlet code:
	// OPopWSnw pppppppp startIns endInstr otherStT otherEnd cycleStt cycleEnd
	// OPop = magic character code, e.g. '+Z' == add if non-zero
	// pppp = pointer to shared memory address
	// startIns..endInstr = etm instruction count at start and end points
	// otherStT..otherEnd = other CPU instruction count for cross-reference
	// cycleStt..cycleEnd = clock tick count from cpu monitor
	register __u32 quad0;
	quad0  = a; quad0 <<= 8;
	quad0 |= b; quad0 <<= 8;
	quad0 |= (was & 0xFF); quad0 <<= 8;
	quad0 |= (now & 0xFF);
	loc->quad[0] = quad0;
	loc->quad[1] = (__u32)p;
	loc->quad[2] = starta;
	loc->quad[3] = enda;
	loc->quad[4] = startb;
	loc->quad[5] = endb;
	loc->quad[6] = CycleCount(0);
	loc->quad[7] = CycleCount(1);
}

inline void DumpLog(struct LogEntry * loc, int star)
{
	PRINT( "%08x %08x %08x %08x %08x %08x %08x %08x%s\n", 
			loc->quad[0], loc->quad[1], loc->quad[2], loc->quad[3],
			loc->quad[4], loc->quad[5], loc->quad[6], loc->quad[7],
			star ? " *" : "");
}

inline void DumpActivityForCpu(int cpu)
{
//	int i;
	struct PerCpuLog * log = atomicLogs[cpu];

	PRINT( "CPU%d (%ld iterations):-\n", cpu, log->iterations);
//	for (i=0 ; i<ENTRIES_IN_LOG ; ++i)
//	{
//		DumpLog(&log->entry[i], i==log->index);
//	}
}

inline void DumpActivity(void)
{
	DumpActivityForCpu(0);
	DumpActivityForCpu(1);
}

inline void LogIteration(int cpu)
{
	++atomicLogs[cpu]->iterations;
}


#define PROLOGUE() \
	int cpu = raw_smp_processor_id();	\
	int endcpu;				\
	struct LogEntry * entry = Next(cpu);	\
	__u32 startOther = SampleETM(!cpu);	\
	__u32 start = SampleETM(cpu);		\
	__u32 end;				\
	__u32 endOther;				\
	int was=0

#define EPILOGUE(a,b,was,now,p,start,end) 					\
	end = SampleETM(cpu);							\
	endOther = SampleETM(!cpu);						\
	endcpu = raw_smp_processor_id();					\
	if (cpu == endcpu)							\
	{									\
		PostLog(entry, a, b, was, now, (void*)p, start, end, startOther, endOther);		\
		MoveOn(cpu);							\
		if (cpu != raw_smp_processor_id())				\
			PRINT( "switched cpu->%d in %s while logging %p\n", !cpu, __FUNCTION__, p);\
	}									\
	else									\
	{									\
		PRINT( "switched cpu during %s\n", __FUNCTION__);	\
	}

#define UNKNOWN 255



#if USE_PRIVATE_PROTOTYPES

#define atomic_readl		atomic_read
#define ATOMIC_INITL		ATOMIC_INIT
#define atomic_setl		atomic_set
#define atomic_getl		atomic_get_safe

inline void atomic_addl(int i, atomic_t *v)
{
	PROLOGUE();
	atomic_add_was(i, v, &was);
	EPILOGUE('+', 'a', was, UNKNOWN, &v->counter, start, end);
}

inline void atomic_subl(int i, atomic_t *v)
{
	PROLOGUE();
	atomic_sub_was(i, v, &was);
	EPILOGUE('-', 'a', was, UNKNOWN, &v->counter, start, end);
}

inline int atomic_add_returnl(int i, atomic_t *v)
{
	PROLOGUE();
	int ret = atomic_add_return_was(i, v, &was);
	EPILOGUE('+', 'r', was, ret, &v->counter, start, end);
	return ret;
}

inline int atomic_sub_returnl(int i, atomic_t *v)
{
	PROLOGUE();
	int ret = atomic_sub_return_was(i, v, &was);
	EPILOGUE('-', 'r', was, ret, &v->counter, start, end);
	return ret;
}

inline int atomic_inc_not_zerol(atomic_t *v)
{
	PROLOGUE();
	int ret;
	ret  = atomic_inc_not_zero_was(v, &was);
	EPILOGUE('+', 'Z', was, ret, &v->counter, start, end);
	return ret;
}

inline int atomic_dec_and_testl(atomic_t *v)
{
	PROLOGUE();
	int ret = atomic_dec_and_test_was(v, &was);
	EPILOGUE('-', 't', was, ret, &v->counter, start, end);
	return ret;
}

inline int atomic_add_unlessl(atomic_t *v, int a, int u)
{
	PROLOGUE();
	int ret = atomic_add_unless_was(v, a, u, &was);
	EPILOGUE('+', 'u', was, ret, &v->counter, start, end);
	return ret;
}

#else // USE_PRIVATE_PROTO


// add a function so we can read exclusive value without just getting local cached value
static inline int atomic_get(atomic_t * v)
{
	int tmp;
	__asm__ __volatile__("@ atomic_get\n"
"1:	ldrex	%0, [%1]\n"
"	clrex\n"
	: "=&r" (tmp)
	: "r" (&v->counter)
	: "cc");

	return tmp;
}

#define atomic_readl		atomic_read
#define ATOMIC_INITL		ATOMIC_INIT
#define atomic_setl		atomic_set
#define atomic_getl		atomic_get
#define atomic_addl 		atomic_add
#define atomic_subl 		atomic_sub
#define atomic_add_returnl	atomic_add_return
#define atomic_sub_returnl	atomic_sub_return
#define atomic_inc_not_zerol	atomic_inc_not_zero
#define atomic_dec_and_testl	atomic_dec_and_test
#define atomic_add_unlessl	atomic_add_unless
#define spin_lockl		spin_lock
#define spin_trylockl		spin_trylock
#define spin_unlockl		spin_unlock
#define down_readl		down_read
#define up_readl		up_read

#endif



// ****** the guts of the test
// 


//#define TCOUNT  160 //8 // 400 // 4 // 1025 // 200
#define TCOUNT 	1
#define TALIGN	0x2000

struct Page
{
	unsigned long	pad[(TALIGN / sizeof(unsigned long)) - 1];
	atomic_t	count;
} __attribute__((__aligned__( TALIGN )));


static inline int PageCount(struct Page *page)
{
	return atomic_readl(&page->count);
}


static atomic_t gStop  = ATOMIC_INITL(0);
static spinlock_t waitForDump;

static struct Page * pages = NULL;
static struct Page * otherPages = NULL;



#include <linux/dma-mapping.h>
#if USE_UNCACHED_MEMORY
#define DECLARE_DMA_DUMMY()	dma_addr_t dma_addr
#define MALLOC(size,flags) 	dma_alloc_coherent(NULL, size, &dma_addr, flags)
#else
#define DECLARE_DMA_DUMMY();
#define MALLOC(size,flags) kzalloc(size, flags)
#endif


static int allocate_memory(void)
{
	int i;
	DECLARE_DMA_DUMMY();

	pages = MALLOC(TCOUNT * 2 * sizeof(struct Page), GFP_KERNEL);
	if (pages == NULL)
	{
		PRINT( "page alloc fails torture\n");
		return -ENOMEM;
	}

	otherPages = MALLOC(TCOUNT * 2 * sizeof(struct Page), GFP_KERNEL);
	if (otherPages == NULL)
	{
		PRINT( "page alloc fails torture\n");
		return -ENOMEM;
	}
	PRINT( "Allocated %d bytes for page blocks @ %p/%p\n", TCOUNT * 2 * sizeof(struct Page), pages, otherPages);

	PRINT( "Allocate %d bytes for page blocks...\n", TCOUNT * 2 * sizeof(struct Page));
	for (i=0 ; i<CONFIG_NR_CPUS ; ++i)
	{
		dma_addr_t dummy;  atomicLogs[i] = (struct PerCpuLog *)dma_alloc_coherent(NULL, sizeof(struct PerCpuLog), &dummy, GFP_KERNEL);
		atomicLogs[i] = (struct PerCpuLog *)MALLOC(sizeof(struct PerCpuLog), GFP_KERNEL);
		PRINT( "per-cpu log located at %p\n", atomicLogs[i]);
	}
	return 0;
}

static void init_memory(void)
{
	int i;
#if !SIM_ONLY
	atomic_setl(&gStop, 0);
	spin_lock_init(&waitForDump);
#endif

	for (i=0 ; i<(TCOUNT*2) ; ++i){
		atomic_setl(&pages[i].count, 1);
		atomic_setl(&otherPages[i].count, 1);
	}

	InitLogs();
	InitETMs();
	PRINT( "ready for torture\n");
	smp_mb();
}



static int do_test(struct Page * pageBlock1, struct Page * pageBlock2, int cpu)
{
	int i;
	int err;
	int ret = 0;
	for (i=0 ; i<TCOUNT ; ++i)
	{
		struct Page * page = &pageBlock1[i];
		struct Page * page2 = page + TCOUNT;
		struct Page * page3 = &pageBlock2[i];
		struct Page * page4 = page3 + TCOUNT;

#define IncAndCheck(page,err,ret) \
ALERT_IF(PageCount(page) == 0); \
err = atomic_inc_not_zerol(&page->count); \
if (!err) { \
  MY_BAD( "page %p err%d line %u\n", &page->count.counter, err, __LINE__); \
  ret = 1; \
  break; \
} \
if (unlikely(PageCount(page) <= 1)) { \
  MY_BAD( "bad page  %p %u\n", &page->count.counter, __LINE__); \
  ret = 1; \
  break; \
}

#define DecAndCheck(page,err,ret) \
if ((err = PageCount(page)) <= 1) { \
  MY_BAD( "page--  %p now %d line %u\n", &page->count.counter, err, __LINE__); \
  ret = 1; \
  break; \
} \
err = atomic_dec_and_testl(&page->count); \
if (err) { \
  MY_BAD( "page %p err%d line %u\n", &page->count.counter, err, __LINE__); \
  ret = 1; \
  break; \
}
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		IncAndCheck(page,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		IncAndCheck(page2,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		IncAndCheck(page3,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		IncAndCheck(page4,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));

		DecAndCheck(page,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		DecAndCheck(page2,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		DecAndCheck(page3,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));
		DecAndCheck(page4,err,ret);
		//printk(KERN_DEBUG "counts: %d %d %d %d\n", PageCount(page), PageCount(page2), PageCount(page3), PageCount(page4));

#if !SIM_ONLY
		if (atomic_getl(&gStop))
		{
			spin_lock(&waitForDump);
			PRINT( "***Stopping cpu%d: other thread has signalled stop\n", cpu);
			spin_unlock(&waitForDump);
			return 1;
		}
#endif
		LogIteration(cpu);
		//msleep(1000);
	}

#if !SIM_ONLY
	if (ret)
	{
		// dump the other CPU first so we stand most chance of catching it
		int cpu = raw_smp_processor_id();
		spin_lock(&waitForDump);
		atomic_setl(&gStop, 1);
		smp_mb();
		PRINT( "Detected failure on cpu%d, dump follows:-\n\n", cpu);
		DumpActivityForCpu(!cpu);
		DumpActivityForCpu(cpu);
		spin_unlock(&waitForDump);
	}
#endif
	return ret;
}




// ****** end of guts


int thread_main(void * context)
{
	int arg = (int)context;
	int err = 0;

	PRINT( "Starting torture thread(%i)...\n", arg);

	while (1)
	{
		int cpu = raw_smp_processor_id();
		err = do_test(pages, otherPages, cpu);
		if ( err || kthread_should_stop())
			break;
		schedule();
	}
	return 0;
}

#define NUM_THRASHERS 2

struct task_struct * ts[NUM_THRASHERS];

static int torture_init(void)
{
	int i;
	int err;
	char buf[128];
	PRINT( "torture_init %s/%s\n", __DATE__, __TIME__);
//	PRINT( "torture_init %s/%s TCOUNT %d\n", __DATE__, __TIME__, TCOUNT);
	// initialise
	err = allocate_memory(); if (err) return err;
	init_memory();
	// run
	for (i=0 ; i<NUM_THRASHERS ; ++i)
	{
		sprintf(buf, "torturer%d", i);
		ts[i] = kthread_run(thread_main, (void *)i, buf);
	}
	return 0;
}

static void torture_exit(void)
{
	int i;
	PRINT( "torture_exit\n");
	for (i=0 ; i<NUM_THRASHERS ; ++i)
	{
		kthread_stop(ts[i]);
	}
	PRINT( "It's all over\n");
	DumpActivity();
	PRINT( "-----------------\n");
}

module_init(torture_init);
module_exit(torture_exit);

// **** end of file

