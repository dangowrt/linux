/*******************************************************************************
GPL License Option

If you received this File from somewhere, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/

#ifndef _BTNS_DRIVER_H_
#define _BTNS_DRIVER_H_

typedef enum
{
	BTN_NO_OP,
	BTN_PUSH,
	BTN_RELEASE,
	BTN_CHANGE /* Both Push & Release will be monitored */
} BTN_OP;


typedef struct {
        unsigned int btn_id;
        unsigned int btn_push_cntr;
        unsigned int btn_release_cntr;
        unsigned int gppVal;			// -> [J.Chiang], 2010/06/04 - Added to get GPPIn value
} BTN, *BTN_PTR;


typedef struct {
	unsigned int btns_number;
        BTN* btns;
} BTNS_STS, *BTNS_STS_PTR;

#define ATON_BTNS_NAME       "BTNS"


struct btn_data {
        unsigned int    gpp_id;
        unsigned int    default_gpp_val;
        BTN_OP          btn_op;
        char            *btn_name;
};

#define CONFIG_GPP_MAX_PINS	32
/*
 *  done against open of /dev/gpp, to get a cloned descriptor.
 */
#define CIOCWAIT_P      _IOWR('c', 150, BTNS_STS)
#define CIOCNOWAIT_P    _IOWR('c', 151, BTNS_STS)
#define CIOCGPIO_P		_IOWR('c', 152, BTNS_STS)
#define CIOCPOWEROFF_P	_IOWR('c', 153, BTNS_STS)
//#define CIOCUSBINIT_P	_IOWR('c', 154, BTNS_STS)
#define CIOFCTRYRST_P	_IOWR('c', 155, BTNS_STS)
#define CIOCREADY_P		_IOWR('c', 156, BTNS_STS)
#define CIOCBUSY_P		_IOWR('c', 157, BTNS_STS)
#define CIOCLEDS_p		_IOWR('c', 158, BTNS_STS)
#define CIOCFAN_p		_IOWR('c', 159, BTNS_STS)
#define CIOCBUZZ_p		_IOWR('c', 160, BTNS_STS)
//#define CIOCUSBSWITCH_P	_IOWR('c', 161, BTNS_STS)
#define CIOCERROR_P		_IOWR('c', 162, BTNS_STS)
#define	CIOCDEBUG_P		_IOWR('c', 163, BTNS_STS)
#define CIOCSETTZ_p		_IOWR('c', 164, BTNS_STS)

#define CIOCHD1_BTN_p	_IOWR('c', 165, BTNS_STS)
#define CIOCHD2_BTN_p	_IOWR('c', 166, BTNS_STS)
//#define CIOCDET_HD1_p	_IOWR('c', 167, BTNS_STS)
//#define CIOCDET_HD2_p	_IOWR('c', 168, BTNS_STS)
#define CIOCPWR_HD1_p	_IOWR('c', 169, BTNS_STS)
#define CIOCPWR_HD2_p	_IOWR('c', 170, BTNS_STS)

#define CIOCERR_HD1_p	_IOWR('c', 171, BTNS_STS)
#define CIOCERR_HD2_p	_IOWR('c', 172, BTNS_STS)

#define CIOCBRTHLED_p	_IOWR('c', 173, BTNS_STS)

#endif /* _BTNS_DRIVER_H_ */

