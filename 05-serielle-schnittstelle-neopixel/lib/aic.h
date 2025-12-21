/* Driver for the AT91SAM7's Advanced Interrupt Controller (AIC).
 *
 * The AIC is responsible for queuing interrupts from other
 * peripherals on the board. It then hands them one by one to the ARM
 * CPU core for handling, according to each peripheral's configured
 * priority.
 */

#ifndef __AIC_H__
#define __AIC_H__

#include <stdint.h>
#include "isr.h"

/* Priority levels for interrupt lines. */

#define AIC_INT_LEVEL_LOWEST       0
#define AIC_INT_LEVEL_LOW          1
#define AIC_INT_LEVEL_ABOVE_LOW    2
#define AIC_INT_LEVEL_BELOW_NORMAL 3
#define AIC_INT_LEVEL_NORMAL       4
#define AIC_INT_LEVEL_ABOVE_NORMAL 5
#define AIC_INT_LEVEL_HIGH         6
#define AIC_INT_LEVEL_HIGHEST      7

/* Interrupt Source Type (definiert in AT91SAM7S64.h) */
//#define     AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL       ((unsigned int) 0x0 <<  5) // (AIC) Internal Sources Code Label High-level Sensitive
//#define     AT91C_AIC_SRCTYPE_EXT_LOW_LEVEL        ((unsigned int) 0x0 <<  5) // (AIC) External Sources Code Label Low-level Sensitive
//#define     AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE    ((unsigned int) 0x1 <<  5) // (AIC) Internal Sources Code Label Positive Edge triggered
//#define     AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE    ((unsigned int) 0x1 <<  5) // (AIC) External Sources Code Label Negative Edge triggered
//#define     AT91C_AIC_SRCTYPE_HIGH_LEVEL           ((unsigned int) 0x2 <<  5) // (AIC) Internal Or External Sources Code Label High-level Sensitive
//#define     AT91C_AIC_SRCTYPE_POSITIVE_EDGE        ((unsigned int) 0x3 <<  5) // (AIC) Internal Or External Sources Code Label Positive Edge triggered

typedef void (*aic_cb_fcn)(void);
 
typedef struct aic_sys_vl {
	struct aic_sys_vl *next;
	aic_cb_fcn         fcn;
} aic_sys_vl_t;

void aic_init(void);
void aic_set_fiq   (aic_cb_fcn isr);
void aic_set_vector(uint32_t vector, uint32_t mode, aic_cb_fcn isr);
void aic_mask_on   (uint32_t vector);
void aic_mask_off  (uint32_t vector);
void aic_clear     (uint32_t mask);

void aic_sys_init           (uint32_t srctype,uint32_t level);
void aic_sys_register_pit   (aic_sys_vl_t *append,aic_cb_fcn fcn);
void aic_sys_register_commxx(aic_cb_fcn fcn);

#endif
