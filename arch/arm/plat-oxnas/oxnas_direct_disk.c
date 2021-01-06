/*
 * arch/arm/plat-oxnas/oxnas_direct_disk.c
 *
 * Copyright (C) 2010 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/oxnas_direct_disk.h>
#include <mach/desc_alloc.h>
#include <mach/oxnas_errors.h>
#include <mach/direct_writes.h>
#include <mach/ox820sata.h>
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
#include <mach/incoherent_sendfile.h>
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

//#define SHOW_WAITS

extern irqreturn_t (*ox810sata_isr_callback)(int, unsigned long);
extern unsigned long ox810sata_isr_arg;

/* end of fs helpers */

void fast_writes_isr(int error, void *arg)
{
	oxnas_direct_disk_context_t *context = (oxnas_direct_disk_context_t*)arg;
	struct inode *inode = context->inode;
	direct_access_context_t *sata_context = inode->writer_filemap_info.direct_access_context;
	void (*release_fn)(int) = sata_context->release;

	if (error) {
		// Record that the transfer failed
		printk(KERN_WARNING "fast_writes_isr() Transfer failed for file %s, error %d\n",
			context->file->f_path.dentry->d_name.name, error);
		set_write_error(inode);
	}

	// Relinquish ownership of the SATA core now we've finished touching it
	(*release_fn)(0);

	// Sort out DMA SG/PRD list used for the completed SATA transfer. The
	// variables for list handling are modified at task level when not under
	// the SATA completion spinlock
	smp_rmb();

	if(atomic_read(&context->free_sg)) {
		int cur_idx = atomic_read(&context->cur_transfer_idx);
		if(cur_idx != -1) {
			if (context->prd_list[cur_idx] != NULL) {
				odrb_free_prd_array(context->prd_list[cur_idx]);
				context->prd_list[cur_idx] = NULL;
				atomic_set(&context->cur_transfer_idx, -1);
				atomic_set(&context->cur_sg_status[cur_idx], 0);
			}
		}
		atomic_set(&context->free_sg, 0);
	}

	// Make sure DMA related changes above will be seen by the woken task
	// when it awakes due to seeing sata_in_progress become zero
	smp_wmb();

//printk("fast_writes_isr() Up for context %p\n", context);
	up(&context->sata_active_sem);
}
