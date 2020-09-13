#ifndef __SYSTIME_H
#define __SYSTIME_H


unsigned int millis(void);

unsigned int micros(void);



/* microsecond delay precision is usually not guarenteed on a general operating system 
 * This is more for delaying at least xxx microseconds
 * (general OS is not realtime)
 */
void delay_us(unsigned int microseconds);

void delay(unsigned int milliseconds);


#endif 