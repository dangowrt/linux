/*********************************************************************************
 * Filename:  iscsi_linux_defs.h
 *
 * This file contains wrapper definies related to LINUX functions.
 *
 * $HeadURL: svn://subversion.sbei.com/pyx/target/branches/2.9-linux-iscsi.org/pyx-target/include/iscsi_linux_defs.h $
 *   $LastChangedRevision: 7131 $
 *   $LastChangedBy: nab $
 *   $LastChangedDate: 2007-08-25 17:03:55 -0700 (Sat, 25 Aug 2007) $
 *
 * Copyright (c) 2003 PyX Technologies, Inc.
 * Copyright (c) 2006-2007 SBE, Inc.  All Rights Reserved.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * For further information, contact via email: support@sbei.com
 * SBE, Inc.  San Ramon, California  U.S.A.
 *********************************************************************************/


#ifndef ISCSI_LINUX_DEFS_H
#define ISCSI_LINUX_DEFS_H

#include <linux/kallsyms.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
/*
 * Used for utsname()-> access
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
#include <linux/syscalls.h>
#endif
#include <linux/highmem.h>

/*
 * Userspace access.
 */
#define CALL_USERMODEHELPER(a, b, c)   call_usermodehelper(a, b, c, 1)

#ifdef MEMORY_DEBUG
#define inline
#endif

/*
 * 2.6.24 provides an updated struct scatterlist API.  Use macros for the new
 * code, and use inline functions for legacy operation. 
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
# define SET_SG_TABLE(sg, cnt)		sg_init_table((struct scatterlist *)sg, cnt);
# define GET_ADDR_SG(sg)		sg_virt(sg)
# define GET_PAGE_SG(sg)		sg_page(sg)
# define SET_PAGE_SG(sg, page)		sg_assign_page(sg, page)

#define SCSI_SG_COUNT(cmd)		scsi_sg_count(cmd)
#define SCSI_SGLIST(cmd)		scsi_sglist(cmd)
#define SCSI_BUFFLEN(cmd)		scsi_bufflen(cmd)

#else
#include <linux/scatterlist.h>
#define SET_SG_TABLE(sg, cnt)	
static inline void *GET_ADDR_SG(struct scatterlist *sg)
{
	return(page_address(sg->page) + sg->offset);
}
static inline struct page *GET_PAGE_SG(struct scatterlist *sg)
{
	return(sg->page);
}
static inline void SET_PAGE_SG(struct scatterlist *sg, struct page *page)
{
	sg->page = page;
	return;
}

#define SCSI_SG_COUNT(cmd)		((cmd)->use_sg)
#define SCSI_SGLIST(cmd)		((struct scatterlist *)(cmd)->request_buffer)
#define SCSI_BUFFLEN(cmd)		((cmd)->request_bufflen)

#endif

/*
 * kernel -- userspace copy commands
 */
#define COPY_FROM_USER(dest, src, len)		\
	copy_from_user((void *)(dest), (void *)(src), (len))
#define COPY_TO_USER(dest, src, len)		\
	copy_to_user((void *)(dest), (void *)(src), (len))

/*
 * Sockets.
 */
#define iscsi_sock_create(sock, f, t, p, uc, td) sock_create(f, t, p, sock)
#define iscsi_sock_create_lite(sock, f, t, p, uc, td) sock_create_lite(f, t, p, sock)
#define iscsi_sock_connect(sock, s_in, size, td) sock->ops->connect(sock, s_in, size, 0)
#define iscsi_sock_bind(sock, s_in, size, td) sock->ops->bind(sock, s_in, size)
#define iscsi_sock_listen(sock, backlog, td) sock->ops->listen(sock, backlog)
#define iscsi_sock_accept(sock, newsock, td) sock->ops->accept(sock, newsock, 0)
#define iscsi_sock_sockopt_off(sock, p, o) \
	{ \
	int value = 0; \
	sock->ops->setsockopt(sock, p, o, (char *)&value, sizeof(value)); \
	}
#define iscsi_sock_sockopt_on(sock, p, o) \
	{ \
	int value = 1; \
	sock->ops->setsockopt(sock, p, o, (char *)&value, sizeof(value)); \
	}
#define iscsi_sock_sockopt_bindtodev(sock, dev) \
	{ \
	sock->ops->setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, dev, strlen(dev)); \
	}

#include <linux/net.h>
#include <linux/inet.h>
#include <linux/socket.h>
#include <linux/module.h>
#include <net/sock.h>

static inline int lio_kernel_bind(struct socket *sock, struct sockaddr *addr, int addrlen)
{
        return sock->ops->bind(sock, addr, addrlen);
}

static inline int lio_kernel_listen(struct socket *sock, int backlog)
{
        return sock->ops->listen(sock, backlog);
}

static inline int lio_kernel_accept(struct socket *sock, struct socket **newsock, int flags)
{
        struct sock *sk = sock->sk;
        int err;

        err = sock_create_lite(sk->sk_family, sk->sk_type, sk->sk_protocol,
                               newsock);
        if (err < 0)
                goto done;

        err = sock->ops->accept(sock, *newsock, flags);
        if (err < 0) {
                sock_release(*newsock);
                *newsock = NULL;
                goto done;
        }

        (*newsock)->ops = sock->ops;
        __module_get((*newsock)->ops->owner);

done:
        return err;
}

static inline int lio_kernel_setsockopt(struct socket *sock, int level, int optname,
                        char *optval, int optlen)
{
        mm_segment_t oldfs = get_fs();
        int err;

        set_fs(KERNEL_DS);
        if (level == SOL_SOCKET)
                err = sock_setsockopt(sock, level, optname, optval, optlen);
        else
                err = sock->ops->setsockopt(sock, level, optname, optval,
                                            optlen);
        set_fs(oldfs);
        return err;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
# define DEV_GET_BY_NAME(name)	dev_get_by_name(&init_net, name)
#else
# define DEV_GET_BY_NAME(name)	dev_get_by_name(name)
#endif

/*
 * Threads.
 */
#define iscsi_daemon(thread, name, sigs) \
	daemonize(name); \
	current->policy = SCHED_NORMAL; \
	set_user_nice(current, -20); \
	spin_lock_irq(&current->sighand->siglock); \
	siginitsetinv(&current->blocked, (sigs)); \
	recalc_sigpending(); \
	(thread) = current; \
	spin_unlock_irq(&current->sighand->siglock);

/*
 * Timers and Time
 */
#define MOD_TIMER(timer, expires)	mod_timer(timer, (get_jiffies_64() + expires * HZ))
#define SETUP_TIMER(timer, t, d, func) \
	timer.expires	= (get_jiffies_64() + t * HZ); \
	timer.data	= (unsigned long) d; \
	timer.function	= func;

/*
 * Other misc stuff.
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
# define TCM_UTS_SYSNAME      utsname()->sysname
# define TCM_UTS_MACHINE      utsname()->machine
#else
# define TCM_UTS_SYSNAME      system_utsname.sysname  
# define TCM_UTS_MACHINE      system_utsname.machine
#endif

#ifndef SCSI_DATA_UNKNOWN
#define SCSI_DATA_UNKNOWN       (DMA_BIDIRECTIONAL)
#endif
#ifndef SCSI_DATA_WRITE
#define SCSI_DATA_WRITE         (DMA_TO_DEVICE)
#endif
#ifndef SCSI_DATA_READ
#define SCSI_DATA_READ          (DMA_FROM_DEVICE)
#endif
#ifndef SCSI_DATA_NONE
#define SCSI_DATA_NONE          (DMA_NONE)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
#define BLKDEV_GET(bd, fmode, mode)	blkdev_get(bd, fmode)
#define BLKDEV_PUT(bd, fmode)		blkdev_put(bd, fmode)
#else
#define BLKDEV_GET(bd, fmode, mode)	blkdev_get(bd, fmode, mode)
#define BLKDEV_PUT(bd, fmode)		blkdev_put(bd)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
#define KMEM_CACHE_CREATE(name, size, align, flags, ctor0)	\
	kmem_cache_create(name, size, align, flags, ctor0)
#else
#define KMEM_CACHE_CREATE(name, size, align, flags, ctor0)       \
	kmem_cache_create(name, size, align, flags, ctor0, NULL)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#include <linux/list.h>

static inline struct list_head *seq_list_start(struct list_head *head, loff_t pos)
{
        struct list_head *lh;

        list_for_each(lh, head)
                if (pos-- == 0)
                        return lh;

        return NULL;
}

static inline struct list_head *seq_list_start_head(struct list_head *head, loff_t pos)
{
        if (!pos)
                return head;

        return seq_list_start(head, pos - 1);
}

static inline struct list_head *seq_list_next(void *v, struct list_head *head, loff_t *ppos)
{
        struct list_head *lh;

        lh = ((struct list_head *)v)->next;
        ++*ppos;
        return lh == head ? NULL : lh;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)

static inline int tcm_strict_strtoul(const char *cp, unsigned int base, unsigned long *res)
{
        char *tail;
        unsigned long val;
        size_t len;

        *res = 0;
        len = strlen(cp);
        if (len == 0)
                return -EINVAL;

        val = simple_strtoul(cp, &tail, base);
#if 0
        if (tail == cp)
                return -EINVAL;
        if ((*tail == '\0') ||
                ((len == (size_t)(tail - cp) + 1) && (*tail == '\n'))) {
                *res = val;
                return 0;
        }

        return -EINVAL;
#else
	*res = val;
	return 0;
#endif
}

static inline int tcm_strict_strtoull(const char *cp, unsigned int base, unsigned long long *res)
{
        char *tail;
        unsigned long long val;
        size_t len;

        *res = 0;
        len = strlen(cp);
        if (len == 0)
                return -EINVAL;

        val = simple_strtoull(cp, &tail, base);
#if 0
        if (tail == cp)
                return -EINVAL;
        if ((*tail == '\0') ||
                ((len == (size_t)(tail - cp) + 1) && (*tail == '\n'))) {
                *res = val;
                return 0;
        }

        return -EINVAL;
#else
	*res = val;
	return 0;
#endif
}

static inline int tcm_strict_strtol(const char *cp, unsigned int base, long *res)
{
        int ret;
        if (*cp == '-') {
                ret = strict_strtoul(cp + 1, base, (unsigned long *)res);
                if (!ret)
                        *res = -(*res);
        } else {
                ret = strict_strtoul(cp, base, (unsigned long *)res);
        }

        return ret;
}

#else

#define tcm_strict_strtoul(cp, base, res)	strict_strtoul(cp, base, res)
#define tcm_strict_strtoull(cp, base, res)	strict_strtoull(cp, base, res)
#define tcm_strict_strtol(cp, base, res)	strict_strtol(cp, base, res)

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)

static inline u64 get_unaligned_be64(const void *p)
{
	return be64_to_cpup((__be64 *)p);
}

#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)

# define init_MUTEX(sem)	sema_init(sem, 1)
# define init_MUTEX_LOCKED(sem)	sema_init(sem, 0)

#endif

#ifndef MAINTENANCE_IN
# define MAINTENANCE_IN		0xa3
#endif
#ifndef MAINTENANCE_OUT
# define MAINTENANCE_OUT	0xa4
#endif

#define SECURITY_PROTOCOL_OUT 0xb5
#define SECURITY_PROTOCOL_IN  0xa2
#define EXTENDED_COPY         0x83
#define READ_ATTRIBUTE        0x8c
#define RECEIVE_COPY_RESULTS  0x84
#define WRITE_ATTRIBUTE       0x8d
#define VARIABLE_LENGTH_CMD   0x7f

#define MI_MANAGEMENT_PROTOCOL_IN 0x10
/* values for maintenance in */
#define MI_REPORT_IDENTIFYING_INFORMATION 0x05
#define MI_REPORT_TARGET_PGS  0x0a
#define MI_REPORT_ALIASES     0x0b
#define MI_REPORT_SUPPORTED_OPERATION_CODES 0x0c
#define MI_REPORT_SUPPORTED_TASK_MANAGEMENT_FUNCTIONS 0x0d
#define MI_REPORT_PRIORITY   0x0e
#define MI_REPORT_TIMESTAMP  0x0f
#define MI_MANAGEMENT_PROTOCOL_IN 0x10

#define MO_SET_TARGET_PGS	0x0a

#define ACCESS_CONTROL_IN     0x86
#define ACCESS_CONTROL_OUT    0x87
#define READ_MEDIA_SERIAL_NUMBER 0xab

#endif    /*** ISCSI_LINUX_DEFS_H ***/
