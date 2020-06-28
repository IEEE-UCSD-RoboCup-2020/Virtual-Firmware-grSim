#include <boost/chrono.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/thread.hpp> 

unsigned int millis(void) {
    auto t = boost::chrono::high_resolution_clock::now();
    return (unsigned int)(double(t.time_since_epoch().count()) / 1000000.00f);
}

unsigned int micros(void) {
    auto t = boost::chrono::high_resolution_clock::now();
    return (unsigned int)(double(t.time_since_epoch().count()) / 1000.00f);
}

void delay_us(unsigned int microseconds) {
    boost::this_thread::sleep_for(boost::chrono::microseconds(microseconds));
}

void delay(unsigned int milliseconds) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(milliseconds));
}
