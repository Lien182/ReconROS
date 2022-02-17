#ifndef A9TIMER_H
#define A9TIMER_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t	TMR_CNT_REG_L;
    volatile uint32_t 	TMR_CNT_REG_H;
    volatile uint32_t	TMR_CNTRL_REG;
    volatile uint32_t 	TMR_INT_STATUS_REG;
    volatile uint32_t	TMR_COMPARE_REG_L;
    volatile uint32_t	TMR_COMPARE_REG_H;
    volatile uint32_t	TMR_AUTO_INC_REG;

}t_a9timer;


/* == Timer functions ================================================== */

/*
 * Initializes the timer. Must be called before any other function can
 * be used.
 *
 *   no parameters
 */
int a9timer_init();

/*
 * Gets the current timestamp.
 *
 *   no parameters
 */
uint64_t a9timer_get();

/*
 * Cleans up the driver.
 *
 *   no parameters
 */
void a9timer_cleanup();

/*
 * Converts the timer value to milliseconds.
 *
 *   t - timer value or difference
 */
float a9timer_toms(uint64_t t);

/*
 * Converts a time in ms to a timer value.
 *
 *   t - time in ms
 */
uint64_t a9timer_msto(float t);


#endif /* TIMER_H */