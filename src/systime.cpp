#include <boost/chrono.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/thread.hpp> 
#include "systime.hpp"

unsigned int millis(void) {
    auto t = boost::chrono::high_resolution_clock::now();
    return (unsigned int)(double(t.time_since_epoch().count()) / 1000000.00f);
}

unsigned int micros(void) {
    auto t = boost::chrono::high_resolution_clock::now();
    return (unsigned int)(double(t.time_since_epoch().count()) / 1000.00f);
}

/* microsecond delay precision is usually not guarenteed on a general operating system 
 * This is more for delaying at least xxx microseconds
 * (general OS is not realtime)
 */
void delay_us(unsigned int microseconds) {
    boost::this_thread::sleep_for(boost::chrono::microseconds(microseconds));
}

void delay(unsigned int milliseconds) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(milliseconds));
}
