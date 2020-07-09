#include "pid.hpp"

/* C++ templated class can't not separate .hpp & .cpp file, 
   all function definition is cramped inside .hpp file due 
   to this limitation */

// checkout pid.hpp for all function definition

// example tester

// double millis_wrapper() { return (double)millis();}

/*
    PID_Controller<double> pid_test(0.00, 1.00, 0.00);
    // pid_test.init(&millis_wrapper);
    pid_test.init(100); // 100Hz --> 10ms period
    // pid_test.set_output_range(-100, 100);
    double curr_error;
    while(1) {
        std::cin >> curr_error;
        std::cout << " :: "<< pid_test.calculate(curr_error) << std::endl;
    }
*/

