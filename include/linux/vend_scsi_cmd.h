/* vend_scsi_cmd.h - header defining the interface between bulk storage 
 * scsi command handler and the vsc command handler.
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

#ifndef _VEND_SCSI_CMD_H_
#define _VEND_SCSI_CMD_H_

#include <linux/cdev.h>
enum scsi_data_direction {
	SCSI_DATA_DIR_UNKNOWN = 0,
	SCSI_DATA_DIR_FROM_HOST,
	SCSI_DATA_DIR_TO_HOST,
	SCSI_DATA_DIR_NONE
};

struct vend_scsi_cmd_dev {
	struct vend_scsi_cmd_qset *data;  /* Pointer to first quantum set */
	int quantum;              /* the current quantum size */
	int qset;                 /* the current array size */
	unsigned long size;       /* amount of data stored here */
	unsigned int access_key;  /* used by vend_scsi_cmduid and vend_scsi_cmdpriv */
	struct semaphore sem;     /* mutual exclusion semaphore     */
	struct cdev cdev;	  /* Char device structure		*/
};

struct vendor_cmds {
	int (*vendor_cmd_data_size)(char cmd[], int *direction);
	int (*do_vendor_cmd)(int data_dir, char * buffer, int size); //TODO: put proper parameters in this call
	void (*get_sense_data)(u32 *sense_data, u32 *sense_data_info);
}; 
	
extern int register_vendor_handler(struct vendor_cmds *handler);
extern void unregister_vendor_handler(void);
	
#endif /* _VEND_SCSI_CMD_H_ */
