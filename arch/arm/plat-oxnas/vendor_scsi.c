/*
 * vendor_scis.c -- the vend_scsi_cmd char module
 *
 * Copyright (C) 2010 J Larkworthy PLX Technology Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <asm/uaccess.h>

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/vend_scsi.h>        /* io-ctl shared definitions */
#include <linux/vend_scsi_cmd.h>	/* kernel shared definitions */



/*
 * Our parameters which can be set at load time.
 */

#define VEND_SCSI_TTY_MAJOR		  240	/* experimental range */
#define VEND_SCSI_TTY_MINORS		4	/* only have 4 devices */

int vend_scsi_cmd_major =   0;
int vend_scsi_cmd_minor =   0;

module_param(vend_scsi_cmd_major, int, S_IRUGO);
module_param(vend_scsi_cmd_minor, int, S_IRUGO);

MODULE_AUTHOR("J Larkworthy");
MODULE_LICENSE("GPL");

#define DRIVER_DESC "Vendor Specific SCSI Command interface"
#define DRIVER_VERSION "v0.2"

static struct tty_driver *vend_scsi_tty_driver;
static struct tty_struct * control_tty;

/* reply data handling structures */
static char * active_buffer;
static int buffer_size;
static struct semaphore buffer_ready_sema, buffer_avail_sema;

/* general command recognistion flag */
static char handle_commands;


/* this implementation - for simplicity - handles LIST_COMMAND_SIZE maximum
 * user defined commands for forwarding for handling
 * TODO: use linked list and variable number of commands - sorted to avoid 
 * linear look up timing issues?
 */
#define LIST_COMMAND_SIZE     20

struct cmd_lst {
	unsigned char command;
	unsigned char sub_command;
	unsigned char direction;
	unsigned char data_type;
	unsigned int  length;
	unsigned char msb_index;
	unsigned char lsb_index;
	unsigned char cmd_length;
} command_list[LIST_COMMAND_SIZE]; //TODO: convert to a dynamic list.

static unsigned char registered_commands_count; // = LIST_COMMAND_SIZE;
static u32 user_sense_data;
static u32 user_sense_info;

static int do_scsi_cmds(int data_dir, char * buffer, int size);
static int command_size_cmds(char cmd[], int *direction);
static void do_get_sense(u32 *data, u32 *info);

static struct vendor_cmds vendor_ops_all = {
	.vendor_cmd_data_size = command_size_cmds,
	.do_vendor_cmd = do_scsi_cmds,
	.get_sense_data=do_get_sense,
	
};

static void do_get_sense(u32 * data, u32 * info){
	*data= user_sense_data;
	*info = user_sense_info;
}

/* use a linear search to find the command received. Proviced the 
 * number of commands is small this is a good solution. If the 
 * search time becomes protracted it may be worth using a different
 * organisation
 */
static int find_cmd(unsigned char cmd, unsigned char sub_cmd)
{
	int i;
	struct cmd_lst *curnt = command_list;

	for(i=0; i< registered_commands_count; i ++, curnt++) {
		if ((cmd == curnt->command) && (sub_cmd == curnt->sub_command))
			return i;
	}
	return -1;
}

/* The primary action routine. The data direction dictates whether this 
 * routine forwards or receives data to or from the user 
 * The size is the data transfered by the usb buffer. this function 
 * may be called multiple times and will continue to append data until
 * all data is transfered. This puts an obligation on the user read 
 * operation to accept multiple blocks and not throw a hissy fit if all
 * the data does not arrive in a single read.
 */
static int do_scsi_cmds(int data_dir, char * buffer, int size){
	int copied = 0; 
	int status;

	if (data_dir == SCSI_DATA_DIR_FROM_HOST) {
		status = tty_buffer_request_room (control_tty, size);
	    if(status!= size) printk(KERN_ERR "vend scsi:insufficient buffer space found\n");
		copied = tty_insert_flip_string(control_tty, buffer, size);
		tty_flip_buffer_push(control_tty);
		
	} else if (data_dir == SCSI_DATA_DIR_TO_HOST){
		/* make buffer available for writer */
		copied = size;
		active_buffer = buffer;
		buffer_size=size;
		up (&buffer_avail_sema);
		
		/* wait here for data to be placed in buffer. */
		if(down_interruptible(&buffer_ready_sema))
			return -ERESTARTSYS;
		copied = size - buffer_size;
		active_buffer=NULL;
		buffer_size=0;
	}
	return copied;
}

/* the function to establish if this is a vendor specific command
 * when the command is found in the list of acceptable commands
 * the details of bulk data size and direction is also established.
 * Should a command be missing or command handling not enabled the 
 * standard error handling is allowed to be invoked.
 */
static int command_size_cmds(char cmd[], int *direction){
	int index;
	int length;
	int status;
	unsigned char *p_cmd=cmd;

	if (!handle_commands) {
		*direction = SCSI_DATA_DIR_UNKNOWN;
		return 0;
	}

	if (p_cmd == NULL){
		printk(KERN_ERR "vend_scsi:matched on null command?\n");
		*direction = SCSI_DATA_DIR_UNKNOWN;
		return 0;
	}
	
	index = find_cmd(cmd[0], cmd[1]);
	if (index < 0)  {
		*direction = SCSI_DATA_DIR_UNKNOWN;
		return 0;
	}

	length = command_list[index].cmd_length;
	
	status = tty_buffer_request_room (control_tty, length);
	if(status != length) printk(KERN_ERR "vend scsi:insufficient buffer space found\n");
	status = tty_insert_flip_string(control_tty, p_cmd, length);
	if(status != length) printk(KERN_ERR "vend scsi:failed to forward all bytes\n");
	tty_flip_buffer_push(control_tty);
	
	if (command_list[index].data_type == LENGTH_TYPE_FIXED) {
		length = command_list[index].length;
	} else {
		length = ((cmd[command_list[index].msb_index]<<8) + (cmd[command_list[index].lsb_index]));
	}
	
	*direction = command_list[index].direction == BULK_DATA_DIR_TO_HOST ? 
			SCSI_DATA_DIR_TO_HOST : SCSI_DATA_DIR_FROM_HOST;
	if(!length) *direction = SCSI_DATA_DIR_NONE;
	
	return length;
	
}

/* The serial driver structure which contains the state data for the 
 * driver
 */
struct vend_scsi_serial {
	struct tty_struct	*tty;		/* pointer to the tty for this device */
	int			open_count;	/* number of times this port has been opened */
	struct semaphore	sem;		/* locks this structure */
};

static struct vend_scsi_serial *vend_scsi_table[VEND_SCSI_TTY_MINORS];	/* initially all NULL */

/* open function called from user space to establish the connection
 * to the user space handler. Currently this driver is aimed at supporting
 * only one handler so multiple opens and multiple tty controls will 
 * cause confusion. 
 */
static int vend_scsi_open(struct tty_struct *tty, struct file *file)
{
	struct vend_scsi_serial *vend_scsi;
	int index;
	
	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	/* get the serial object associated with this tty pointer */
	index = tty->index;
	
	if (index != 0) {
		printk(KERN_ERR "only one tty supported\n");
		return -EACCES;
	}
	
	vend_scsi = vend_scsi_table[index];
	if (vend_scsi == NULL) {
		/* first time accessing this device, let's create it */
		vend_scsi = kmalloc(sizeof(*vend_scsi), GFP_KERNEL);
		if (!vend_scsi)
			return -ENOMEM;

		init_MUTEX(&vend_scsi->sem);
		vend_scsi->open_count = 0;

		vend_scsi_table[index] = vend_scsi;
	}
	if (vend_scsi->open_count != 0) {
		printk(KERN_ERR "only one handler supported\n");
		return -EACCES;
	}
	
	control_tty = tty;
	down(&vend_scsi->sem);

	/* save our structure within the tty structure */
	tty->driver_data = vend_scsi;
	vend_scsi->tty = tty;
	tty->low_latency = 1;

	++vend_scsi->open_count;
	if (vend_scsi->open_count == 1) {
		/* this is the first time this port is opened */
		/* do any hardware initialization needed here */

	}

	up(&vend_scsi->sem);
	return 0;
}

/* close action function used to terminate the connection to 
 * the user space command handler
 */
static void do_close(struct vend_scsi_serial *vend_scsi)
{
	down(&vend_scsi->sem);
	
	handle_commands = 0; /* make sure no more commands are accepted */

	if (!vend_scsi->open_count) {
		/* port was never opened */
		goto exit;
	}

	--vend_scsi->open_count;
	if (vend_scsi->open_count <= 0) {
		/* The port is being closed by the last user. */
		/* Do any hardware specific stuff here */

	}
exit:
	up(&vend_scsi->sem);
}

/* the close function called by the user space command handler to 
 * disconnect from the driver 
 */
static void vend_scsi_close(struct tty_struct *tty, struct file *file)
{
	struct vend_scsi_serial *vend_scsi = tty->driver_data;

	if (vend_scsi)
		do_close(vend_scsi);
}	

/* the file write function used to accept data from the user space
 * command processor for forward to the SCSI initiator.
 */
static int vend_scsi_write(struct tty_struct *tty, 
		      const unsigned char *buffer, int count)
{
	struct vend_scsi_serial *vend_scsi = tty->driver_data;
	int transfer;
	int retval = -EINVAL;
	
	if (!buffer) {
		return -EINVAL;
	}

	if (!vend_scsi) {
		return -ENODEV;
	}

	/* wait for  buffer to be available */
	if(down_interruptible(&buffer_avail_sema))
			return -ERESTARTSYS;

	down(&vend_scsi->sem);

	if (!vend_scsi->open_count) {
		printk(KERN_ERR "vend_scsi:write:no port open\n");
		/* port was not opened */
		goto exit;
	}

	transfer=min(count, buffer_size);
	memcpy(active_buffer, buffer, transfer);
	buffer_size -= transfer;
	retval = transfer;
	up(&buffer_ready_sema);
	
exit:	
	up(&vend_scsi->sem);

	
	return retval;
}

/* obligatory function used to provide write space status to 
 * the user write function routines. 
 */
static int vend_scsi_write_room(struct tty_struct *tty) 
{
	struct vend_scsi_serial *vend_scsi = tty->driver_data;
	int room = -EINVAL;

	if (!vend_scsi)
		return -ENODEV;

	down(&vend_scsi->sem);
	
	if (!vend_scsi->open_count) {
		/* port was not opened */
		goto exit;
	}

	/* calculate how much room is left in the device */
	room = buffer_size;

exit:
	up(&vend_scsi->sem);
	return room;
}

/* the ioctl function which allows the command processing routine in
 * the user space to establish which commands it will accept and 
 * to signal when it is ready to accept commands.
 */
static int vend_scsi_ioctl(struct tty_struct *tty, struct file *file,
                      unsigned int cmd, unsigned long arg)
{
	struct scsi_control data;
	int rc = 0;

	
	if (arg != 0) {
		if (copy_from_user(&data,(void __user *) arg, sizeof(data)))
			return -EFAULT;
	}

	
	switch (cmd) {
	case VEND_IORESET:
		/* force system to release semaphores 
			- remove write lock from terminal and read wait.
			-  clear command list.
		*/
		registered_commands_count = 0; /* all commands are cleared */
		handle_commands = 0; /* stop handling commands */
		break;
	case VEND_IOSCMD:
		if (arg==0)
		  return -EINVAL;
		/* set command details for command handling. */
		{
			struct cmd_lst * cur= command_list + registered_commands_count;
			cur->command = data.option.cmd_format.scsi_cmd;
			cur->sub_command = data.option.cmd_format.scsi_sub_cmd;
			cur->direction = data.option.cmd_format.data_direction;
			cur->data_type = data.option.cmd_format.length_type;
			cur->cmd_length = data.option.cmd_format.cmd_length;
			if (cur->data_type == LENGTH_TYPE_FIXED) 
				cur->length = (data.option.cmd_format.length_msb << 8) +
				data.option.cmd_format.length_lsb;
			else {
				cur->msb_index = data.option.cmd_format.length_msb;
				cur->lsb_index = data.option.cmd_format.length_lsb;
			}
			registered_commands_count++;
		}
		break;
	case VEND_IOSRPLY:
		if (arg==0)
		  return -EINVAL;
		/* set sense data for failed commands - release waiting 
		 semaphores */
		user_sense_data = data.option.set_sense_data.sense_data;
		user_sense_info = data.option.set_sense_data.sense_data_info;		 
		 break;
	case VEND_IOSTRT:  
		/* allow systme to process incomming commands */
		handle_commands =  1;
		break;
	case VEND_IOSTOP:
		/* stop incomming command handling */
		
		handle_commands = 0;
		break;
	default:
	    rc = -ENOIOCTLCMD;
	}

	return rc;
}

/* basic file operations for a serial device */
static struct tty_operations serial_ops = {
	.open = vend_scsi_open,
	.close = vend_scsi_close,
	.write = vend_scsi_write,
	.write_room = vend_scsi_write_room,
	.ioctl = vend_scsi_ioctl,
};

/* module initialisation routines
 * 
 * This module registers its availability to handle vendor specific 
 * commands to the SCSI handler in the usb gadget framework. 
 * It currently only works with the file_storage module.
 * It is not part of the usb composite gadget framework!
 */
static int __init vend_scsi_init(void)
{
	int retval;
	int i;

	/* allocate the tty driver */
	vend_scsi_tty_driver = alloc_tty_driver(VEND_SCSI_TTY_MINORS);
	if (!vend_scsi_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	vend_scsi_tty_driver->owner = THIS_MODULE;
	vend_scsi_tty_driver->driver_name = "vend_scsi_tty";
	vend_scsi_tty_driver->name = "ttyG";
	vend_scsi_tty_driver->major = VEND_SCSI_TTY_MAJOR,
	vend_scsi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	vend_scsi_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	vend_scsi_tty_driver ->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV,
	vend_scsi_tty_driver->init_termios = tty_std_termios;
	vend_scsi_tty_driver->init_termios.c_iflag = 0;
	vend_scsi_tty_driver->init_termios.c_oflag = 0;
	vend_scsi_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD | HUPCL | CLOCAL;
	vend_scsi_tty_driver->init_termios.c_ispeed = 38400;
	vend_scsi_tty_driver->init_termios.c_ospeed = 38400;
	vend_scsi_tty_driver->init_termios.c_lflag = 0;
	vend_scsi_tty_driver->init_termios.c_cc[VTIME] = 0;
	vend_scsi_tty_driver->init_termios.c_cc[VMIN] = 1;
	
	
	tty_set_operations(vend_scsi_tty_driver, &serial_ops);

	/* hack to make the book purty, yet still use these functions in the
	 * real driver.  They really should be set up in the serial_ops
	 * structure above... */
	// vend_scsi_tty_driver->read_proc = vend_scsi_read_proc;
	// vend_scsi_tty_driver->tiocmget = vend_scsi_tiocmget;
	// vend_scsi_tty_driver->tiocmset = vend_scsi_tiocmset;
	// vend_scsi_tty_driver->ioctl = vend_scsi_ioctl;

	/* register the tty driver */
	retval = tty_register_driver(vend_scsi_tty_driver);
	if (retval) {
		printk(KERN_ERR "vend_scsi:failed to register vend_scsi tty driver\n");
		put_tty_driver(vend_scsi_tty_driver);
		return retval;
	}

	for (i = 0; i < VEND_SCSI_TTY_MINORS; ++i)
		tty_register_device(vend_scsi_tty_driver, i, NULL);

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION);
	return retval;
}

/* the module cleanup and removal function
 */
static void __exit vend_scsi_exit(void)
{
	struct vend_scsi_serial *vend_scsi;
	int i;

	for (i = 0; i < VEND_SCSI_TTY_MINORS; ++i)
		tty_unregister_device(vend_scsi_tty_driver, i);
	tty_unregister_driver(vend_scsi_tty_driver);

	/* shut down all of the timers and free the memory */
	for (i = 0; i < VEND_SCSI_TTY_MINORS; ++i) {
		vend_scsi = vend_scsi_table[i];
		if (vend_scsi) {
			/* close the port */
			while (vend_scsi->open_count)
				do_close(vend_scsi);

			kfree(vend_scsi);
			vend_scsi_table[i] = NULL;
		}
	}
}

/*
 * Finally, the module stuff
 */

/*
 * The cleanup function is used to handle initialization failures as well.
 * Thefore, it must be careful to work correctly even if some of the items
 * have not been initialized
 */
void vend_scsi_cmd_cleanup_module(void)
{
	unregister_vendor_handler();
	vend_scsi_exit();
}


static int __init vend_scsi_cmd_init_module(void)
{
	
	sema_init(&buffer_avail_sema,0);
	sema_init(&buffer_ready_sema,0);

	register_vendor_handler(&vendor_ops_all);

	return vend_scsi_init(); /* succeed */

}

module_init(vend_scsi_cmd_init_module);
module_exit(vend_scsi_cmd_cleanup_module);
