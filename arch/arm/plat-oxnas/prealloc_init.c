/*
 * arch/arm/plat-oxnas/prealloc_init.c
 *
 * Copyright (C) 2008, 2009, 2010 Oxford Semiconductor Ltd
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
#include <mach/prealloc_init.h>

const loff_t PREALLOC_SIZE_LIMIT = ((loff_t)2)*1024*1024*1024;

int is_ext4_file(struct inode *inode)
{
	return !strncmp(inode->i_sb->s_type->name, "ext4", 4);
}

/*
 * writer filemap semaphore should be held around calls to this function
 */
void do_prealloc_init(
	struct inode *inode,
	struct file  *file)
{
	if (unlikely(!inode->prealloc_initialised)) {
		int use_prealloc = file->f_flags & O_PREALLOC;
		int use_fast     = file->f_flags & O_FAST;
		int fast_capable = supports_fast_mode(file, inode);

		inode->prealloc_initialised = 1;

		WARN_ON(use_prealloc && !file->f_op->preallocate);
		WARN_ON(use_fast && !fast_capable);
		WARN_ON(use_fast && !use_prealloc);

		if (use_prealloc) {
			inode->prealloc_size = i_size_read(inode);

			inode->do_space_reserve = 1;

			if (is_ext4_file(inode)) {
				// Implies preallocate space gets reset on truncate
				inode->truncate_space_reset = 1;
			} else {
				inode->truncate_space_reset = 0;
			}
		} else {
			inode->do_space_reserve = 0;
			inode->prealloc_size = 0;
			inode->truncate_space_reset = 0;
		}
	}
}

/*
 * writer filemap semaphore should be held around calls to this function
 */
void trim_preallocation(
	struct inode *inode,
	struct file  *file,
	loff_t        offset)
{
	loff_t length = 0;

	// Calculate unused preallocated length at end of file
	length = inode->prealloc_size;
	length -= offset;

	if ((length > 0) && (offset >= 0)) {
//printk(KERN_INFO "trim_preallocation() File %s, i_size %lld, acc_size %lld, unprealloc from %lld for %lld bytes\n", file->f_path.dentry->d_name.name, i_size_read(inode), acc_size_read(inode), offset, length); 
		file->f_op->unpreallocate(file, offset, length);
	}

	// Set preallocated size to match the unpreallocation just performed
	inode->prealloc_size = offset;
	inode->space_reserve = 0;
//printk(KERN_INFO "do_unpreallocation() File %s, set prealloc_size to %lld bytes\n", file->f_path.dentry->d_name.name, inode->prealloc_size);
}

void disable_preallocation(
	struct inode *inode,
	struct file  *file)
{
	file->f_flags &= ~O_PREALLOC;

	inode->prealloc_initialised = 0;
	if (is_preallocated_space(inode)) {
		inode->do_space_reserve = 0;
	}
}

void do_unpreallocation(
	struct inode *inode,
	struct file  *file)
{
//printk(KERN_INFO "do_unpreallocation() File %s, fp %p, inode %p, i_size %lld, acc_size %lld\n", file->f_path.dentry->d_name.name, file, inode, i_size_read(inode), acc_size_read(inode));
	inode->prealloc_initialised = 0;
	if (is_preallocated_space(inode)) {
		trim_preallocation(inode, file, i_size_read(inode));
		inode->do_space_reserve = 0;
	}
}
