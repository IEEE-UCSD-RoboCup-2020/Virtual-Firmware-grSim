#ifndef __ACTUATORS_H
#define __ACTUATORS_H

#include "common.hpp"
#include "sensors.hpp"


class GrSim_Console{
private:
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
    static const unsigned int BUF_SIZE = 256;

    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;
    boost::mutex mu;

public:
    GrSim_Console(io_service& io_srvs, udp::endpoint& endpoint);

    ~GrSim_Console();

    void send_command(bool is_team_yellow, int id, 
                      float upper_left_wheel_speed, 
                      float lower_left_wheel_speed,
                      float lower_right_wheel_speed, 
                      float upper_right_wheel_speed,
                      // float x, float y, float omega, 
                      float kick_speed_x, float kick_speed_y, 
                      bool spinner);
};


class Actuator_System {
private:
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<GrSim_Console> GrSim_Console_ptr;
    typedef boost::shared_ptr<boost::thread> thread_ptr;
    typedef boost::shared_ptr<boost::asio::deadline_timer> timer_ptr;

    team_color_t color;
    int id;
    GrSim_Console_ptr console;
    thread_ptr v_thread;
    boost::mutex mu;
    boost::condition_variable_any cond_init_finished;
    unsigned int ctrl_period_ms = 10; // milliseconds  
    timer_ptr timer;

    void send_cmd_thread(udp::endpoint& c_ep);
    void timer_expire_callback();
    
    // To-do:
    float max_possible_speed(arma::vec direction);
    

public:
    float wheel_upper_left_vel = 0.00, 
          wheel_lower_left_vel = 0.00, 
          wheel_lower_right_vel = 0.00, 
          wheel_upper_right_vel = 0.00;
    float kick_speed_x = 0.00, kick_speed_y = 0.00;
    bool dribbler_on = false;

    Actuator_System(team_color_t color, int robot_id, udp::endpoint& grsim_console_ep);

    void set_ctrl_freq(float freq_Hz);
    void set_ctrl_period(float period_ms);

    /*
     * Set motor "target speed" (not immediate speed, acceleration is needed), 
     * which is essentially motor output pwr (or acceleration that's not constant, 
     * the smaller the gap between the target and current speed, the smaller the acceleration)
     * unit: rad/s
     */
    void set_wheels_speeds(float upper_left, float lower_left, 
                                  float lower_right, float upper_right) {
        mu.lock();
        wheel_upper_left_vel = upper_left;
        wheel_lower_left_vel = lower_left;
        wheel_lower_right_vel = lower_right;
        wheel_upper_right_vel = upper_right;
        mu.unlock();
    }

    void turn_on_dribbler() { 
        mu.lock();
        dribbler_on = true;
        mu.unlock(); 
    }
    void turn_off_dribbler() { 
        mu.lock();
        dribbler_on = false;
        mu.unlock(); 
    }
    void kick(float speed_x, float speed_y) { 
        mu.lock();
        kick_speed_x = speed_x; 
        kick_speed_y = speed_y; 
        mu.unlock();
    }
    
    void stop();


    /* 
    * (check the comment for "set_wheels_speeds" function in hpp first)
    * Using a 3D vector (x,y,w) to represent a movement towards
    * a target linear velocity with direction and magnitude of vector (x,y),
    * and at same time a angular velocity with direction(+/-) and magnitude of scalar w
    * The unit for the magnitude quantification is represented in percentage of max speed
    * The net magnitude of this 3D vector must not be greater than 100.00f which corresponds to 100% of motor pwr
    * (think about why having a nonzero angular speed decreases the max possible linear speed)
    * Example1: (0.00, 100.00, 0.00) sets the robot to move forward at full speed
    * Example2: (100.00, 0.00, 0.00) sets the robot to move right at fulll speed
    * Example3: (0.00, 0.00, 100.00) sets the robot to rotate left at full speed
    * and this method also allow a robot to move linearly & angularly at the same time to 
    * produce a curved motion.
    * Underline assumption: linear & angular motion are linearly independent  
    */
    void move(arma::vec pwr_vec_3D);




};


#endif