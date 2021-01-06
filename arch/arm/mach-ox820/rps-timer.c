/*
 * arch/arm/mach-ox820/rps-time.c
 *
 * Copyright (C) 2009 Oxford Semiconductor Ltd
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clockchips.h>
#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/rps-timer.h>
#include <mach/rps-irq.h>

// -> [Walker Chen], 2011/02/08 - PWN LED CTRL and system mode
#ifdef CONFIG_BOARD_D20
#define	LOW_BRIGHTNESS		(0xFF >> 2)
#define	MIDDLE_BRIGHTNESS	(0xFF >> 1)
#define	HIGH_BRIGHTNESS		0xFF

unsigned long system_B_brightness = HIGH_BRIGHTNESS ;
unsigned long system_R_brightness = HIGH_BRIGHTNESS ;
unsigned long HD1_B_brightness = HIGH_BRIGHTNESS ;
unsigned long HD1_R_brightness = HIGH_BRIGHTNESS ;
unsigned long HD2_B_brightness = HIGH_BRIGHTNESS ;
unsigned long HD2_R_brightness = HIGH_BRIGHTNESS ;
unsigned long USB_brightness = HIGH_BRIGHTNESS ;

EXPORT_SYMBOL(system_B_brightness);
EXPORT_SYMBOL(system_R_brightness);
EXPORT_SYMBOL(HD1_B_brightness);
EXPORT_SYMBOL(HD1_R_brightness);
EXPORT_SYMBOL(HD2_B_brightness);
EXPORT_SYMBOL(HD2_R_brightness);
EXPORT_SYMBOL(USB_brightness);

unsigned int breathing_led_on = 0;
EXPORT_SYMBOL(breathing_led_on);


#endif //CONFIG_BOARD_D20
// <- End.

// -> [Walker Chen], 2010/07/21 - Added variables for initialized usage.
int sys_ready=0;
EXPORT_SYMBOL(sys_ready);
int sys_error=0;
EXPORT_SYMBOL(sys_error);
// <- End.

// -> [Walker Chen], 2012/04/23 - SATA nad USB bus Disk Activity Indicator
extern int sata1_active;
extern int sata2_active;
int sata1_err=0;
int sata2_err=0;
EXPORT_SYMBOL(sata1_err);
EXPORT_SYMBOL(sata2_err);

int usb_hdd_active=0;
EXPORT_SYMBOL(usb_hdd_active);
// <- End.

// -> [Walker Chen], 2011/01/17 - Ethernet Link State Indicator
int eth_link=0;
EXPORT_SYMBOL(eth_link);
// <- End.

// -> [Walker Chen], 2011/02/19 -  how many USB devices in used
int usb_device_in_use=0;
EXPORT_SYMBOL(usb_device_in_use);

int ethusb_device_in_use=0;
EXPORT_SYMBOL(ethusb_device_in_use);
//<-End.


static struct clock_event_device ckevt_rps_timer1;

static int oxnas_rps_set_next_event(unsigned long delta, struct clock_event_device* unused)
{
	if (delta == 0)
		return -ETIME;

	/* Stop timers before programming */
    *((volatile unsigned long*)TIMER1_CONTROL) = 0;

    /* Setup timer 1 load value */
    *((volatile unsigned long*)TIMER1_LOAD) = delta;

    /* Setup timer 1 prescaler, periodic operation and start it */
    *((volatile unsigned long*)TIMER1_CONTROL) =
        (TIMER_1_PRESCALE_ENUM << TIMER_PRESCALE_BIT) |
        (TIMER_1_MODE          << TIMER_MODE_BIT)     |
        (TIMER_ENABLE_ENABLE   << TIMER_ENABLE_BIT);

	return 0;
}

static void oxnas_rps_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
    switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		/* Stop timers before programming */
		*((volatile unsigned long*)TIMER1_CONTROL) = 0;

		/* Set period to match HZ */
		*((volatile unsigned long*)TIMER1_LOAD) = TIMER_1_LOAD_VALUE;

		/* Setup prescaler, periodic operation and start it */
        *((volatile unsigned long*)TIMER1_CONTROL) =
            (TIMER_1_PRESCALE_ENUM << TIMER_PRESCALE_BIT) |
            (TIMER_1_MODE          << TIMER_MODE_BIT)     |
            (TIMER_ENABLE_ENABLE   << TIMER_ENABLE_BIT);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
        /* Stop timer */
        *((volatile unsigned long*)TIMER1_CONTROL) &=
            ~(TIMER_ENABLE_ENABLE   << TIMER_ENABLE_BIT);
        break;
	}
}


// -> [Walker Chen], 2011/11/04 - Behavior of breathing lights
inline void breathing_led_function( int clk ){

	u8 const ZXB_code[256]={
	0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,0x98,0x9c,0x9f,0xa2,
	0xa5,0xa8,0xab,0xae,0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,
	0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,0xda,0xdc,0xde,0xe0,
	0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf4,
	0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,0xfd,0xfe,0xfe,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,
	0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,0xf6,0xf5,0xf3,0xf2,
	0xf0,0xef,0xed,0xec,0xea,0xe8,0xe6,0xe4,0xe3,0xe1,0xde,0xdc,
	0xda,0xd8,0xd6,0xd3,0xd1,0xce,0xcc,0xc9,0xc7,0xc4,0xc1,0xbf,
	0xbc,0xb9,0xb6,0xb4,0xb1,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,
	0x99,0x96,0x92,0x8f,0x8c,0x89,0x86,0x83,0x80,0x7d,0x79,0x76,
	0x73,0x70,0x6d,0x6a,0x67,0x64,0x61,0x5e,0x5b,0x58,0x55,0x52,
	0x4f,0x4c,0x49,0x46,0x43,0x41,0x3e,0x3b,0x39,0x36,0x33,0x31,
	0x2e,0x2c,0x2a,0x27,0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,
	0x15,0x14,0x12,0x10,0xf,0xd,0xc,0xb,0x9,0x8,0x7,0x6,0x5,0x4,
	0x3,0x3,0x2,0x1,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x1,0x1,0x2,0x3,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xa,0xc,0xd,
	0xe,0x10,0x12,0x13,0x15,0x17,0x18,0x1a,0x1c,0x1e,0x20,0x23,
	0x25,0x27,0x29,0x2c,0x2e,0x30,0x33,0x35,0x38,0x3b,0x3d,0x40,
	0x43,0x46,0x48,0x4b,0x4e,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,
	0x66,0x69,0x6c,0x6f,0x73,0x76,0x79,0x7c
	};
	static u8 index=0;
	if ( clk ){
		*(volatile u32*) GPIO_B_PWM16_VALUE = ZXB_code[ index++ ];
		*(volatile u32*) GPIO_B_PWM17_VALUE = 0x0;
	}
}
// <- End.

// -> [Walker Chen], 2012/04/23 - LEDs Functin
inline void led_function( int clk , int clk2 ){

	#ifdef CONFIG_BOARD_D20
	switch( sys_ready ){
		case 0:	/* system is buzy */
			if( sys_error == 0 ){
				/* blinking Blue */
				if (clk){
					*(volatile u32*)GPIO_B_PWM16_VALUE = (*(volatile u32*)GPIO_B_PWM16_VALUE)?0:system_B_brightness;
					*(volatile u32*)GPIO_B_PWM17_VALUE = 0x0;
				}
			}else{
				/* blinking RED */
				if (clk){
					*(volatile u32*)GPIO_B_PWM16_VALUE = 0x0;
					*(volatile u32*)GPIO_B_PWM17_VALUE = (*(volatile u32*)GPIO_B_PWM17_VALUE)?0:system_R_brightness;
				}
			}
			/* HD1 Indicator */
			if(  *(volatile u32*)GPIO_B_DATA & ( 1UL << 0 )  ){
				if ( sata1_err == 0){
					/* normal */
					if ( sata1_active ){
						*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM4_VALUE = 0x0;
					}else{
						*(volatile u32*)GPIO_A_PWM27_VALUE = HD1_B_brightness;
						*(volatile u32*)GPIO_B_PWM4_VALUE = 0x0;
					}
				}else if(sata1_err == 1){
					/* solid RED */
					if (clk){
						*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM4_VALUE = HD1_R_brightness;
					}
				}else{
					/* blinking RED */
					if (clk){
						*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM4_VALUE = (*(volatile u32*)GPIO_B_PWM4_VALUE)?0:HD1_R_brightness;
					}
				}
			}else{
				/* no disk1 */
				*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
				*(volatile u32*)GPIO_B_PWM4_VALUE = 0x0;
			}
			/* HD2 Indicator */
			if( *(volatile u32*)GPIO_B_DATA & ( 1UL << 1 ) ){
				if ( sata2_err == 0){
					/* normal */
					if ( sata2_active ){
						*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM7_VALUE = 0x0;
					}else{
						*(volatile u32*)GPIO_B_PWM6_VALUE = HD2_B_brightness;
						*(volatile u32*)GPIO_B_PWM7_VALUE = 0x0;
					}
				}else if( sata2_err == 1){
					/* Solid RED */
					if (clk){
						*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM7_VALUE = HD2_R_brightness;
					}
				}else{
					/* blinking RED */
					if (clk){
						*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM7_VALUE = (*(volatile u32*)GPIO_B_PWM7_VALUE)?0:HD2_R_brightness;
					}
				}
			}else{
				/* no disk2 */
				*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
				*(volatile u32*)GPIO_B_PWM7_VALUE = 0x0;
			}
			/* USB Indicator */
			if ( usb_device_in_use > 2 || ethusb_device_in_use >1 ){
				if ( usb_hdd_active ){
					*(volatile u32*)GPIO_B_PWM8_VALUE = 0x0;
				}else{
					*(volatile u32*)GPIO_B_PWM8_VALUE = USB_brightness;
				}
			}else{
				*(volatile u32*)GPIO_B_PWM8_VALUE = 0x0;
			}
			break;
		case 1:	/* system is ready */
			if( sys_error == 0 ){
				if( breathing_led_on ){
					breathing_led_function( clk2 );
				}else{	
					/* solid Blue */
					if (clk){
						*(volatile u32*)GPIO_B_PWM16_VALUE = system_B_brightness;
						*(volatile u32*)GPIO_B_PWM17_VALUE = 0x0;
					}
				}
			}else{
				/* blink RED */
				if (clk){
					*(volatile u32*)GPIO_B_PWM16_VALUE = 0x0;
					*(volatile u32*)GPIO_B_PWM17_VALUE = (*(volatile u32*)GPIO_B_PWM17_VALUE)?0:system_R_brightness;
				}
			}
			/* HD1 Indicator */
			if( *(volatile u32*)GPIO_B_DATA & ( 1UL << 0 )  ){
				if ( sata1_err == 0){
					/* normal */
					if ( sata1_active ){
						breathing_led_on = 0;
						*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM4_VALUE = 0x0;
					}else{
						*(volatile u32*)GPIO_A_PWM27_VALUE = HD1_B_brightness;
						*(volatile u32*)GPIO_B_PWM4_VALUE = 0x0;
					}
				}else if(sata1_err == 1){
					/* solid RED */
					if (clk){
						*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM4_VALUE = HD1_R_brightness;
					}
				}else{
					/* blinking RED */
					if (clk){
						*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM4_VALUE = (*(volatile u32*)GPIO_B_PWM4_VALUE)?0:HD1_R_brightness;
					}
				}
			}else{
				/* no disk1 */
				*(volatile u32*)GPIO_A_PWM27_VALUE = 0x0;
				*(volatile u32*)GPIO_B_PWM4_VALUE = 0x0;
			}
			/* HD2 Indicator */
			if( *(volatile u32*)GPIO_B_DATA & ( 1UL << 1 ) ){
				if ( sata2_err == 0){
					/* normal */
					if ( sata2_active ){
						breathing_led_on = 0;
						*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM7_VALUE = 0x0;
					}else{
						*(volatile u32*)GPIO_B_PWM6_VALUE = HD2_B_brightness;
						*(volatile u32*)GPIO_B_PWM7_VALUE = 0x0;
					}
				}else if( sata2_err == 1){
					/* Solid RED */
					if (clk){
						*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM7_VALUE = HD2_R_brightness;
					}
				}else{
					/* blinking RED */
					if (clk){
						*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
						*(volatile u32*)GPIO_B_PWM7_VALUE = (*(volatile u32*)GPIO_B_PWM7_VALUE)?0:HD2_R_brightness;
					}	
				}
			}else{
				/* no disk2 */
				*(volatile u32*)GPIO_B_PWM6_VALUE = 0x0;
				*(volatile u32*)GPIO_B_PWM7_VALUE = 0x0;
			}
			/* USB Indicator */
			if ( usb_device_in_use > 2 || ethusb_device_in_use >1 ){
				if ( usb_hdd_active ){
					*(volatile u32*)GPIO_B_PWM8_VALUE = 0x0;
				}else{
					*(volatile u32*)GPIO_B_PWM8_VALUE = USB_brightness;
				}
			}else{
				*(volatile u32*)GPIO_B_PWM8_VALUE = 0x0;
			}
			break;
		default: /* gpio debug mode */
			*(volatile u32*)GPIO_B_PWM16_VALUE = system_B_brightness;
			*(volatile u32*)GPIO_B_PWM17_VALUE = system_R_brightness;
			*(volatile u32*)GPIO_A_PWM27_VALUE = HD1_B_brightness;
			*(volatile u32*)GPIO_B_PWM4_VALUE = HD1_R_brightness;
			*(volatile u32*)GPIO_B_PWM6_VALUE = HD2_B_brightness;
			*(volatile u32*)GPIO_B_PWM7_VALUE = HD2_R_brightness;
			*(volatile u32*)GPIO_B_PWM8_VALUE = USB_brightness;
	}
	#endif //CONFIG_BOARD_D20

}
// <- End.

static irqreturn_t OXNAS_RPS_timer_interrupt(int irq, void *dev_id)
{
    /* Clear the timer interrupt - any write will do */
    *((volatile unsigned long*)TIMER1_CLEAR) = 0;

    /* Quick, to the high level handler... */
    if(ckevt_rps_timer1.event_handler) {
        ckevt_rps_timer1.event_handler(&ckevt_rps_timer1);
    }

	// -> [Walker Chen], 2011/02/08 - timer_interrupt_function
 	static int count = HZ / 6;
 	if ( --count == 0 ){
 	count = HZ / 6;
	}
	
 	static int count2 = HZ / 50;
 	if ( --count2 == 0 ){
 	count2 = HZ / 50;
	}	
	
	led_function( count == 1 , count2 == 1 );

	// <- End.

    return IRQ_HANDLED;
}

static struct irqaction oxnas_timer_irq = {
	.name    = "RPSA timer1",
	.flags	 = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = OXNAS_RPS_timer_interrupt
};

static cycle_t ox820_get_cycles(struct clocksource *cs)
{
	cycle_t time = *((volatile unsigned long*)TIMER2_VALUE);
	return ~time;
}


static struct clocksource clocksource_ox820 = {
	.name	= "rps-timer2",
	.rating	= 200,
	.read	= ox820_get_cycles,
	.mask	= CLOCKSOURCE_MASK(24),
	.shift	= 10,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init ox820_clocksource_init(void)
{
	/* setup timer 2 as free-running clocksource */
	*((volatile unsigned long*)TIMER2_CONTROL) = 0;
	*((volatile unsigned long*)TIMER2_LOAD) = TIMER_2_LOAD_VALUE;

	*((volatile unsigned long*)TIMER2_CONTROL) =
		(TIMER_2_PRESCALE_ENUM << TIMER_PRESCALE_BIT) |
		(TIMER_2_MODE          << TIMER_MODE_BIT) |
		(TIMER_ENABLE_ENABLE   << TIMER_ENABLE_BIT );

	clocksource_ox820.mult = clocksource_hz2mult(TIMER_2_PRESCALED_CLK, clocksource_ox820.shift);

	printk(KERN_INFO "ox820_clocksource_init() Timer 2 running at %d Hz\n", TIMER_2_PRESCALED_CLK);
	clocksource_register(&clocksource_ox820);
}

void oxnas_init_time(void) {
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = __io_address(OX820_TWD_BASE);
#endif

	ckevt_rps_timer1.name = "RPSA-timer1";
    ckevt_rps_timer1.features = CLOCK_EVT_FEAT_PERIODIC;
    ckevt_rps_timer1.rating = 306;
    ckevt_rps_timer1.shift = 24;
    ckevt_rps_timer1.mult =
        div_sc(CLOCK_TICK_RATE , NSEC_PER_SEC, ckevt_rps_timer1.shift);
    ckevt_rps_timer1.max_delta_ns = clockevent_delta2ns(0x7fff, &ckevt_rps_timer1);
    ckevt_rps_timer1.min_delta_ns = clockevent_delta2ns(0xf, &ckevt_rps_timer1);
    ckevt_rps_timer1.set_next_event	= oxnas_rps_set_next_event;
    ckevt_rps_timer1.set_mode = oxnas_rps_set_mode;
	ckevt_rps_timer1.cpumask = cpu_all_mask;

    // Connect the timer interrupt handler
    oxnas_timer_irq.handler = OXNAS_RPS_timer_interrupt;
    setup_irq(RPS_TIMER_1_INTERRUPT, &oxnas_timer_irq);

    ox820_clocksource_init();
  	clockevents_register_device(&ckevt_rps_timer1);
}

#ifndef CONFIG_GENERIC_TIME
/*
 * Returns number of microseconds since last clock tick interrupt.
 * Note that interrupts will be disabled when this is called
 * Should take account of any pending timer tick interrupt
 */
static unsigned long oxnas_gettimeoffset(void)
{
	// How long since last timer interrupt?
    unsigned long ticks_since_last_intr =
		(unsigned long)TIMER_1_LOAD_VALUE - *((volatile unsigned long*)TIMER1_VALUE);

    // Is there a timer interrupt pending
    int timer_int_pending = *((volatile unsigned long*)RPSA_IRQ_RAW_STATUS) &
		(1UL << DIRECT_RPS_TIMER_1_INTERRUPT);

    if (timer_int_pending) {
		// Sample time since last timer interrupt again. Theoretical race between
		// interrupt occuring and ARM reading value before reload has taken
		// effect, but in practice it's not going to happen because it takes
		// multiple clock cycles for the ARM to read the timer value register
		unsigned long ticks2 = (unsigned long)TIMER_1_LOAD_VALUE - *((volatile unsigned long*)TIMER1_VALUE);

		// If the timer interrupt which hasn't yet been serviced, and thus has
		// not yet contributed to the tick count, occured before our initial
		// read of the current timer value then we need to account for a whole
		// timer interrupt period
		if (ticks_since_last_intr <= ticks2) {
			// Add on a whole timer interrupt period, as the tick count will have
			// wrapped around since the previously seen timer interrupt (?)
			ticks_since_last_intr += TIMER_1_LOAD_VALUE;
		}
    }

    return TICKS_TO_US(ticks_since_last_intr);
}
#endif // !CONFIG_GENERIC_TIME

struct sys_timer oxnas_timer = {
    .init   = oxnas_init_time,
#ifndef CONFIG_GENERIC_TIME
    .offset = oxnas_gettimeoffset,
#endif // !CONFIG_GENERIC_TIME
};
