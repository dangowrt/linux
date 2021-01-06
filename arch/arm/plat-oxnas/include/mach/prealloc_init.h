#ifndef PREALLOC_INIT_H
#define PREALLOC_INIT_H

/*
 * arch/arm/plat-oxnas/include/mach/prealloc_init.h
 *
 * Copyright (C) 2009, 2010 Oxford Semiconductor Ltd
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

extern const loff_t PREALLOC_SIZE_LIMIT;

extern void do_prealloc_init(struct inode *inode, struct file *file);
extern void trim_preallocation(struct inode *inode, struct file *file, loff_t offset);
extern void do_unpreallocation(struct inode *inode, struct file *file);
extern void disable_preallocation(struct inode *inode, struct file *file);

static inline int inode_supports_fast_mode(struct inode *inode)
{
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
	return inode->i_op->get_extents &&
	       inode->i_op->getbmapx &&
	       inode->i_fop->resetpreallocate;
#else // CONFIG_OXNAS_FAST_READS_AND_WRITES
	return 0;
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
}

static inline int supports_fast_mode(
	struct file  *file,
	struct inode *inode)
{
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
	int inode_fast_capable = inode_supports_fast_mode(inode);
	int file_fast_capable = file->f_op->preallocate &&
	                        file->f_op->incoherent_sendfile;
	
	BUG_ON(file_fast_capable && !inode_fast_capable);
	BUG_ON(!file_fast_capable && inode_fast_capable);

	return file_fast_capable && inode_fast_capable;
#else // CONFIG_OXNAS_FAST_READS_AND_WRITES
	return 0;
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
}

static inline int is_preallocated_space(struct inode *inode)
{
	return inode->space_reserve;
}

#endif        //  #ifndef PREALLOC_INIT_H
