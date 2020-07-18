#include "common.hpp"
#include "systime.hpp"
#include "actuators.hpp"
#include "sensors.hpp"
#include "pid.hpp"

using namespace boost;
using namespace boost::asio;
using namespace arma;
typedef boost::asio::ip::udp udp;
typedef boost::asio::ip::tcp tcp;


/* Some constants:
 * motor max speed around 416 rpm at 1N*m torque load
 * 416 rpm ~= 43.56 rad/s 
 */

std::ostream& operator<<(std::ostream& os, const arma::vec& v);

int main(int argc, char **argv) {

    B_Log::static_init();
    B_Log::sink->set_filter(severity >= Info);
    

    udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.2"), 10020);
    udp::endpoint grsim_console_ep(ip::address::from_string(LOCAL_HOST), 20011);
    
    Sensor_System sensors(BLUE, 0, grsim_ssl_vision_ep);
    Actuator_System actuators(BLUE, 0, grsim_console_ep);


    delay(500); 

    // mute all logs
    B_Log::sink->set_filter(severity > Fatal);


    double m1, m2, m3, m4;
    vec d = {0, 0}, v = {0, 0}, prev_d = {0, 0}, prev_v = {0, 0};
    double theta, omega, prev_theta = 0.00, prev_omega = 0.00;
    while(1) {
        std::cin >> m1 >> m2 >> m3 >> m4;

        B_Log::set_shortest_format();
        // B_Log::sink->set_filter(tag_attr == "vision data" && severity == Trace);
        
        sensors.init();
        int t0 = millis();
        while(millis() - t0 < 1000) {
            actuators.set_wheels_speeds(m1, m2, m3, m4);
            d = sensors.get_translational_displacement();
            v = sensors.get_translational_velocity();
            theta = sensors.get_rotational_displacement();
            omega = sensors.get_rotational_velocity();
            // if(!arma::approx_equal(v, prev_v, "absdiff", 0,00001) || omega != prev_omega ) { 
                std::cout << d << " " 
                          << v << " "
                          << theta << " " 
                          << omega 
                          << " time: " << millis() - t0 << " ms" << std::endl;  
            // }
            /*
            std::cout << d(0) << ", " << d(1) << ", "
                      << v(0) << ", " << v(1) << ", "
                      << theta << ", "
                      << omega << ", " 
                      << sensors.get_curr_timestamp() << std::endl;
            */
            delay(10);
            prev_d = d; prev_v = v; prev_theta = theta; prev_omega = omega;
        }
        
        B_Log::sink->set_filter(severity >= Info);

        t0 = millis();
        while(millis() - t0 < 100) actuators.stop();
    }

/*
    boost::mutex mu;
    boost::thread *threads[6];
    for(int i = 0; i < 6; i++) {
        threads[i] = new boost::thread( [i, &mu] () -> void  {
            io_service ios;
            
            mu.lock();
            std::cout << i << std::endl;
            mu.unlock();
            udp::endpoint ep_grsim_console(ip::address::from_string(LOCAL_HOST), 20011);
            GrSim_Console console(ios, ep_grsim_console);

            udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.3"), 10020);
            GrSim_Vision sim_vision(ios, grsim_ssl_vision_ep);

            while(1) {
                float speed = (i + 1) * 10;
                console.send_command(false, i, speed, speed, speed, speed, 0, 0, false);
                sim_vision.receive_packet();
                delay(10);
            }
        });
    }

    for(auto& thread : threads) {
        thread->join();
        delete thread;
    }
*/

  
    return 0;
}



std::ostream& operator<<(std::ostream& os, const arma::vec& v)
{
    char fmt_str[15]; // not so important size, greater than printed str size is fine, use magic number here 
    int num_rows = arma::size(v).n_rows;
    os << "<";
    for(int i = 0; i < num_rows; i++) {
        sprintf(fmt_str, "%8.3lf", v(i));
        os << std::string(fmt_str);
        if(i != num_rows - 1) os << ", ";
    }
    os << ">";
    return os;
}


/*
 * Armadillo C++ library Citation:
 * 
 * Conrad Sanderson and Ryan Curtin.
 * Armadillo: a template-based C++ library for linear algebra.
 * Journal of Open Source Software, Vol. 1, pp. 26, 2016.
 *
 * Conrad Sanderson and Ryan Curtin.
 * A User-Friendly Hybrid Sparse Matrix Class in C++.
 * Lecture Notes in Computer Science (LNCS), Vol. 10931, pp. 422-430, 2018. 
 */
