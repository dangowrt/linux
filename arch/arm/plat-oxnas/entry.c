/*
 * arch/arm/plat-oxnas/entry.c
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
 
#include <linux/syscalls.h>
#include <linux/security.h>
#include <linux/file.h>
#include <linux/genhd.h>
#include <linux/pagemap.h>
#include <mach/oxnas_errors.h>

#ifdef CONFIG_OXNAS_FAST_WRITES

#include <mach/direct_writes.h>

SYSCALL_DEFINE4(direct_disk_write, int, sock_in_fd, int, file_out_fd, loff_t __user *,offset, size_t, count)
{
	struct file   *in_file;
	int            fput_needed_in;
	struct file   *out_file = 0;
	int            fput_needed_out;
	long           retval;
	struct socket *socket;
	loff_t         start_offset = 0;
	loff_t         file_offset = 0;

	/*
	 * Get input socket, and verify that it is ok
	 */
	retval = -EBADF;
	in_file = fget_light(sock_in_fd, &fput_needed_in);
	if (unlikely(!in_file)) {
		printk(KERN_INFO "sys_write_direct_disk - No infile returning\n");
		goto out;
	}

	/*
	 * Get output file, and verify that it is ok
	 */
	retval = -EBADF;
	out_file = fget_light(file_out_fd, &fput_needed_out);
	if (unlikely(!out_file)) {
		printk(KERN_INFO "sys_write_direct_disk - Failed no output file\n");
		goto fput_in;
	}

	if(!(out_file->f_flags & O_FAST)) {
//		printk(KERN_INFO "Fast write called without FAST Open\n");
		goto net_to_cache;
	}

	socket = in_file->private_data;

	if (unlikely(copy_from_user(&start_offset, offset, sizeof(loff_t)))) {
		retval = -EFAULT;
		printk(KERN_INFO "sys_write_direct_disk - Copy from user failed on offset \n");
		goto fput_out;
	}

	file_offset = in_file->f_pos;

	retval = rw_verify_area(READ, in_file, &file_offset, count);
	if (retval < 0) {
		printk(KERN_INFO "sys_write_direct_disk - read verify area returned less than 0\n");
		goto fput_in;
	}
	count = retval;

	file_offset = start_offset;
	retval = rw_verify_area(WRITE, out_file, &file_offset, count);
	if (retval < 0) {
		printk(KERN_INFO "sys_write_direct_disk - verify area failed on output file\n");
		goto fput_out;
	}
	count = retval;

//	printk(KERN_INFO "Values received from samba offset - %lld, size - %lld \n", start_offset, total_count);

	retval = oxnas_do_direct_disk_write(socket, out_file, start_offset, count);

	if (retval == OXNAS_FALLBACK) {
//		printk(KERN_INFO "Fallback - calling net to cache writes\n");
net_to_cache:
		retval = sys_direct_netrx_write(sock_in_fd, file_out_fd, offset, count);
	}

fput_out:
	fput_light(out_file, fput_needed_out);
fput_in:
	fput_light(in_file, fput_needed_in);
out:
	return retval;

}

#else /* CONFIG_OXNAS_FAST_WRITES */

SYSCALL_DEFINE4(direct_disk_write, int, sock_in_fd, int, file_out_fd, loff_t __user *,offset, size_t, count)
{
	return -EPERM;
}
 
#endif /* CONFIG_OXNAS_FAST_WRITES */

/* End of File */
