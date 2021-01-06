/*
 * arch/arm/plat-oxnas/fast_open_filter.c
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

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/net.h>
#include <linux/spinlock.h>
#include <mach/prealloc_init.h>
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
#include <mach/fast_open_filter.h>
#include <mach/incoherent_sendfile.h>
#include <linux/sched.h>
#include <linux/wait.h>
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
#ifdef CONFIG_OXNAS_FAST_WRITES
#include <mach/direct_writes.h>
#endif // CONFIG_OXNAS_FAST_WRITES

const int FAST_NUM_EXTENTS_LIMIT = 146;	// 8K of EXT4 internal filemap entries

/* Returns true if non-fast access to the file is denied */
int non_fast_access_denied(struct file *file)
{
	int denied = 0;

	if (!file->inode)
		// Not a regular file that could be open for fast access
		return 0;

	if (!supports_fast_mode(file, file->inode))
		// Can never be open for fast access so nothing more to check
		return 0;

	// File could be open for fast access so need to check whether currently
	// open for fast access
	denied = file->f_flags & O_FAST;
	WARN_ON(denied);
	return denied;
}

static void lightweight_filter_open(
	struct file  *file,
	struct inode *inode)
{
	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("lightweight_filter_open() A second has elapsed while waiting, inode %p, file %s\n", inode, file->f_path.dentry->d_name.name);
	}

 	if (!inode->normal_open_count++) {
 		if (inode->i_op->get_extents &&
 			(inode->i_op->get_extents(inode) > FAST_NUM_EXTENTS_LIMIT)) {
//			printk(KERN_INFO "lightweight_filter_open() Attempt to fast open a "
//				"file %s composed of too many extents so disabling preallocation\n",
//				file->f_path.dentry->d_name.name);

 			file->f_flags &= ~O_PREALLOC;
 		}
 	}

	up(&inode->writer_sem);

	file->inode = inode;
}

static void lightweight_filter_close(struct file *file)
{
	struct inode *inode = file->inode;

	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("lightweight_filter_close() A second has elapsed while waiting, inode %p, file %s\n", inode, file->f_path.dentry->d_name.name);
	}

	if (--inode->normal_open_count == 0) {
		do_unpreallocation(inode, file);
	}

	up(&inode->writer_sem);

	file->inode = NULL;
}

int wait_for_truncate(struct inode *inode)
{
	int lock_dropped = 0;

	if (unlikely(inode->truncate_count)) {
		DEFINE_WAIT(wait);
		for (;;) {
			prepare_to_wait(&inode->truncate_wait_queue, &wait, TASK_UNINTERRUPTIBLE);
			if (!inode->truncate_count) {
				break;
			}
			spin_unlock(&inode->fast_lock);
			lock_dropped = 1;
			if (!schedule_timeout(HZ)) {
				printk(KERN_INFO "wait_for_truncate() A second has passed while waiting inode %p\n", inode);
			}
			spin_lock(&inode->fast_lock);
		}
		finish_wait(&inode->truncate_wait_queue, &wait);
	}

	return lock_dropped;
}

int begin_truncate(struct inode *inode)
{
	int ret = 0;

	if (inode_supports_fast_mode(inode)) {
		spin_lock(&inode->fast_lock);
//printk("begin_truncate() For fast supporting inode 0x%p\n", LENGTH, inode);

		BUG_ON(inode->fast_open_count && inode->truncate_count);

		// Can avoid falling back if there are no fast reads or writes currently
		// in progress and there has been no fast reader and there currently is
		// no fast writer active on this inode
		if ((inode->fast_reads_in_progress_count ||
			inode->fast_writes_in_progress_count) ||
			(inode->filemap_info.map || inode->writer_file_context)) {
				// Can only proceed with truncate operations once the file is no longer
				// open for fast access
//printk("begin_truncate() Need to fallback for inode 0x%p\n", inode);
				while (inode->fast_open_count > 0) {
					// The fast lock may have been dropped in order to wait for
					// fallback to complete so must re-check all the open state of
					// the inode now that we have re-gained the lock
//printk("begin_truncate() Falling back for inode 0x%p\n", inode);
					fast_fallback(inode);
				}
			}

		// Record that the file is in use for normal access so that subsequent
		// open/close and fast reads/writes are held off
		++inode->truncate_count;

//printk("begin_truncate() Leaving for inode 0x%p\n", inode);
		spin_unlock(&inode->fast_lock);
	}

	return ret;
}

void end_truncate(struct inode *inode)
{
	if (inode_supports_fast_mode(inode)) {
		spin_lock(&inode->fast_lock);
//printk("end_truncate() For fast supporting inode 0x%p\n", inode);

		BUG_ON(inode->truncate_count <= 0);

		if (!--inode->truncate_count) {
//printk("end_truncate() Issuing wake ups for inode 0x%p\n", inode);
			wake_up(&inode->truncate_wait_queue);
		}

//printk("end_truncate() Leaving for fast supporting inode 0x%p\n", inode);
		spin_unlock(&inode->fast_lock);
	}
}

int fast_open_filter(
	struct file  *file,
	struct inode *inode)
{
	int ret = 0;

//printk("fast_open_filter() File %p, inode %p\n", file, inode);
	BUG_ON(file->inode);

	if (!supports_fast_mode(file, inode)) {
//printk("fast_open_filter() File %p, inode %p FAST mode is not supported\n", file, inode);
		if (file->f_flags & O_FAST) {
			/* The filesystem on which the file resides does not support FAST
			 * mode so force to NORMAL mode
			 */
			file->f_flags &= ~O_FAST;
		}

		// Still need to track file open/close for prealloc/unprealloc support
		lightweight_filter_open(file, inode);
	} else {
		spin_lock(&inode->fast_lock);

retry_open:
		/* If fallback from fast to normal mode is in progress, wait for it to
		 * complete */
		if (unlikely(inode->fallback_in_progress)) {
//printk("fast_open_filter() File %p, inode %p, waiting for fallback to complete\n", file, inode);
			wait_fallback_complete(inode);
		}
 
		if (wait_for_truncate(inode)) {
			// The fast lock was dropped in order to wait so must re-check all
			// the open state of the inode now that we have re-gained the lock
			goto retry_open;
		}

		// If some close processing that involves the entire inode is in progress
		// then wait until it is complete before opening the file again
		if (unlikely(inode->close_in_progress)) {
			static const int MAX_WAIT_LOOPS = 10;
			int wait_loops = 0;
			DEFINE_WAIT(wait);

			while (wait_loops++ < MAX_WAIT_LOOPS) {
				int timed_out = 0;

				prepare_to_wait(&inode->close_wait_queue, &wait, TASK_UNINTERRUPTIBLE);
				if (!inode->close_in_progress) {
					break;
				}

				spin_unlock(&inode->fast_lock);
				timed_out = !schedule_timeout(HZ);
				spin_lock(&inode->fast_lock);
				
				if (timed_out) {
					printk(KERN_INFO "wait_for_close_in_progress() A second has passed while waiting inode %p, close progress %d, close step %d\n",
						inode, inode->close_in_progress, inode->close_step);
				}
			}
			finish_wait(&inode->close_wait_queue, &wait);
			
			if (wait_loops >= MAX_WAIT_LOOPS) {
				printk(KERN_INFO "wait_for_close_in_progress() Max wait loops exceeded for inode %p, close progress %d, close step %d: forcing close-in-progress reset\n",
					inode, inode->close_in_progress, inode->close_step);

				// Something has gone wrong with file close handling, try to
				// ignore and continue
				inode->close_in_progress = 0;
				inode->close_step = -1;
				wake_up(&inode->close_wait_queue);
			}

			// The fast lock was probably dropped in order to wait so must
			// re-check all the open state of the inode now that we have
			// re-gained the lock
			goto retry_open;
		}

//printk("fast_open_filter() File %s, size %lld\n", file->f_path.dentry->d_name.name, i_size_read(inode));
		if (file->f_flags & O_FAST) {
//printk("fast_open_filter() File %s, fp %p, inode %p FAST open request (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
			if (inode->normal_open_count > 0) {
//printk("fast_open_filter() File%s, fp %p, inode %p already open for NORMAL read so force back to NORMAL (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
				file->f_flags &= ~O_FAST;
				++inode->normal_open_count;
			} else {
				if (!inode->fast_open_count) {
					// Reset the record of the file size including any
					// not-yet-commited accumulated writes here rather
					// than on close in case there can be a possibility
					// of stat() not seeing the full size if it races
					// with close
					i_tent_size_write(inode, 0);

					// Clear inode error record on open, rather than on
					// close when could lose the information for the
					// cleanup operations
					clear_write_error(inode);

					// On first open of the file extent count and disable
					// fast mode and preallocation if too many extents that
					// might lead to filemap memory allocation problems
					if (inode->i_op->get_extents &&
						(inode->i_op->get_extents(inode) > FAST_NUM_EXTENTS_LIMIT)) {
//						printk(KERN_INFO "fast_open_filter() Attempt to fast open "
//							"a file %s composed of too many extents so, falling "
//							"back to normal access\n", file->f_path.dentry->d_name.name);

						// Cannot do fast access without preallocation
						file->f_flags &= ~(O_PREALLOC | O_FAST);
						inode->normal_open_count = 1;
					}
				}

				if (!inode->normal_open_count) {
					/* Remember that we haven't yet allocated any context for this
					 * new fast reader
					 */
					file->fast_context = NULL;
#ifdef CONFIG_OXNAS_FAST_WRITES
					file->fast_write_context = NULL;
#endif // CONFIG_OXNAS_FAST_WRITES

					/* Record this fast open file with the inode */
					INIT_LIST_HEAD(&file->fast_head);
					write_lock(&inode->fast_files_lock);
					list_add_tail(&file->fast_head, &inode->fast_files);
					write_unlock(&inode->fast_files_lock);
					++inode->fast_open_count;
//printk("fast_open_filter() File %s, fp %p, inode %p sucessfully opened for FAST read (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
				}
			}
		} else {
//printk("fast_open_filter() File %s, fp %p, inode %p NORMAL open request (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
			if (inode->fast_open_count > 0) {
				/* Force the FAST mode users to fallback to normal mode so that
				 * we can then allow the new normal open to proceed
				 */
//printk("fast_open_filter() File %s, fp %p, inode %p already open for FAST read, so causing fallback (normal %d, fast %d)", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
				fast_fallback(inode);
			}

			// On first open of the file check extent count and disable
			// preallocation if too many that might lead to filemap memory
			// allocation problems
			if (!inode->normal_open_count++) {
				if (inode->i_op->get_extents &&
					(inode->i_op->get_extents(inode) > FAST_NUM_EXTENTS_LIMIT)) {
//					printk(KERN_INFO "fast_open_filter() Attempt to normal open "
//							"a file %s composed of too many extents so falling "
//							"back to normal access\n", file->f_path.dentry->d_name.name);

					file->f_flags &= ~O_PREALLOC;
				}
			}
//printk("fast_open_filter() File %s, fp %p, inode %p sucessfully opened for NORMAL read (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
		}

		spin_unlock(&inode->fast_lock);

		if (!ret) {
			/* Successfully opened a file on a filesystem capable of FAST mode */
			file->inode = inode;
		}
	}

	return ret;
}

void fast_close_filter(struct file* file)
{
	struct inode *inode = file->inode;
	int wait_for_close_set = 0;

//	WARN(!inode, "fast_close_filter() fp %p, inode %p, file %s\n", file, file->inode, file->f_path.dentry->d_name.name);
	if (!inode) {
		// Probably a shared-memory, semaphore or message queue object
		return;
	}

//printk("fast_close_filter() File %p, inode %p, f_count = %ld\n", file, file->inode, atomic_long_read(&file->f_count));
	if (!supports_fast_mode(file, inode)) {
		// For filesystems without fast access support still need to track file
		// open/close for prealloc/unprealloc support
		lightweight_filter_close(file);
	} else {
		int final_close = 0;
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		int free_filemap = 0;
		incoherent_sendfile_context_t *context = 0;
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
#ifdef CONFIG_OXNAS_FAST_WRITES
		int reset_prealloc = 0;
		void *write_context = 0;
#endif // CONFIG_OXNAS_FAST_WRITES

		spin_lock(&inode->fast_lock);

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
retry_close:
		/* If fallback from fast to normal mode is in progress, wait for it to
		 * complete */
		if (unlikely(inode->fallback_in_progress)) {
//printk("fast_close_filter() File %p, inode %p, waiting for fallback to complete\n", file, inode);
			wait_fallback_complete(inode);
		}

		if (wait_for_truncate(inode)) {
			// The fast lock was dropped in order to wait so must re-check all
			// the open state of the inode now that we have re-gained the lock
			goto retry_close;
		}

		if (file->f_flags & O_FAST) {
//printk("fast_close_filter() File %s, fp %p, inode %p FAST close request (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);

			if (!(--inode->fast_open_count)) {
				final_close = 1;

				WARN_ON(inode->close_in_progress);
				inode->close_in_progress = 1;
				inode->close_step = 0;

				wait_for_close_set = 1;

				free_filemap = 1;
#ifdef CONFIG_OXNAS_FAST_WRITES
				reset_prealloc = 1;
#endif // CONFIG_OXNAS_FAST_WRITES
//printk("fast_close_filter() File %s, fp %p, inode %p FAST open count now zero (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
			}

			/* Was a fast write context allocated for this file? */
			if (file->fast_write_context) {
				/*
				 * Cleaning up a file handle's fast write context involves
				 * the entire inode, so need to hold off subsequent opens on
				 * the inode
				 */
				write_context = file->fast_write_context;

				if (!wait_for_close_set) {
					WARN_ON(inode->close_in_progress);
					inode->close_in_progress = 1;
					inode->close_step = 0;

					wait_for_close_set = 1;
				}
			}

			/* Was a fast read context reserved for this file? */
			if (file->fast_context) {
				/*
				 * Want context deallocated once inode fast lock dropped
				 * No need to worry about subsequent open calls on the inode as
				 * the read context is local to the file handle being closed
				 * for the last time
				 */
				context = file->fast_context;
			}

			/* Remove the reference from the inode to this fast file */
			write_lock(&inode->fast_files_lock);
			list_del(&file->fast_head);
			write_unlock(&inode->fast_files_lock);
		} else {
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
//printk("fast_close_filter() File %s, fp %p, inode %p NORMAL close request (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
			BUG_ON(--inode->normal_open_count < 0);
			if (inode->normal_open_count == 0) {
				final_close = 1;

				WARN_ON(inode->close_in_progress);
				inode->close_in_progress = 1;
				inode->close_step = 0;

				wait_for_close_set = 1;
			}
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		}
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

		spin_unlock(&inode->fast_lock);

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		if (context) {
//printk("fast_close_filter() File %s, fp %p, inode %p freeing context %p\n", file->f_path.dentry->d_name.name, file, inode, context);
if (wait_for_close_set) inode->close_step = 1;
			incoherent_sendfile_free_context(context);
if (wait_for_close_set) inode->close_step = 2;
		}

#ifdef CONFIG_OXNAS_FAST_WRITES
		if (write_context) {
//printk("fast_close_filter() File %s, fp %p, inode %p complete_fast_write\n", file->f_path.dentry->d_name.name, file, inode);
if (wait_for_close_set) inode->close_step = 3;
			complete_fast_write(file);
if (wait_for_close_set) inode->close_step = 4;
			if (reset_prealloc) {
				/* last file - reset prealloc */
if (wait_for_close_set) inode->close_step = 5;
				writer_reset_prealloc(file);
if (wait_for_close_set) inode->close_step = 6;
			}
		}
#endif // CONFIG_OXNAS_FAST_WRITES
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

		// Need to clean up preallocation on the last file close on the inode
		if (final_close) {
			while (down_timeout(&inode->writer_sem, HZ)) {
				printk("fast_close_filter() A second has elapsed while waiting, inode %p, file %s\n", inode, file->f_path.dentry->d_name.name);
			}

			do_unpreallocation(inode, file);
			up(&inode->writer_sem);
		}

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		if (free_filemap) {
//printk("fast_close_filter() File %s, fp %p, inode %p freeing filemap\n", file->f_path.dentry->d_name.name, file, inode);
if (wait_for_close_set) inode->close_step = 9;
			incoherent_sendfile_check_and_free_filemap(inode);
if (wait_for_close_set) inode->close_step = 10;
#ifdef CONFIG_OXNAS_FAST_WRITES
			fast_write_check_and_free_filemap(inode);
if (wait_for_close_set) inode->close_step = 11;
#endif // CONFIG_OXNAS_FAST_WRITES
		}

		file->inode = NULL;
		file->fast_context = NULL;
#ifdef CONFIG_OXNAS_FAST_WRITES
		file->fast_write_context = NULL;
#endif // CONFIG_OXNAS_FAST_WRITES

		if (wait_for_close_set) {
			// Wake anyone waiting to open the inode due to close cleanup being
			// in-progress
			spin_lock(&inode->fast_lock);

			WARN_ON(!inode->close_in_progress);
			inode->close_in_progress = 0;
			inode->close_step = -1;
			wake_up(&inode->close_wait_queue);

			spin_unlock(&inode->fast_lock);
		}
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
	}
}
