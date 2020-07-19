#include "actuators.hpp"
#include "systime.hpp"


using namespace arma;
using namespace boost;
using namespace boost::asio;
using namespace boost::posix_time;




Actuator_System::Actuator_System(team_color_t color, int robot_id, udp::endpoint& grsim_console_ep) {
    this->color = color;
    this->id = robot_id;
    //save the thread_ptr copy to extend the life scope of the smart pointer thread_ptr 
    v_thread = thread_ptr(
        new boost::thread(boost::bind(&Actuator_System::send_cmd_thread, this, grsim_console_ep))
    );
    mu.lock();
    cond_init_finished.wait(mu);
    mu.unlock();
    logger << "\033[0;32m actuator system initialized \033[0m";
    logger.add_tag("motion cmd");
}

void Actuator_System::send_cmd_thread(udp::endpoint& c_ep) {
    io_service ios;
    this->timer = timer_ptr(new deadline_timer(ios));
    this->console = GrSim_Console_ptr(new GrSim_Console(ios, c_ep));
    
    logger << "start sending motion cmd to grSim";
    
    cond_init_finished.notify_all();

    
    // timer-driven background cmd-sending async task
    this->timer->expires_from_now(milliseconds(ctrl_period_ms));
    this->timer->async_wait(boost::bind(&Actuator_System::timer_expire_callback, this));
    
    

    ios.run();
}



/*
    * Set motor "target speed" (not immediate speed, acceleration is needed), 
    * which is essentially motor output pwr (or acceleration that's not constant, 
    * the smaller the gap between the target and current speed, the smaller the acceleration)
    * unit: rad/s
    */
void Actuator_System::set_wheels_speeds(float upper_left, float lower_left, 
                                float lower_right, float upper_right) {
    writer_lock(rwmu);
    wheel_upper_left_vel = upper_left;
    wheel_lower_left_vel = lower_left;
    wheel_lower_right_vel = lower_right;
    wheel_upper_right_vel = upper_right;
}

void Actuator_System::turn_on_dribbler() { 
    writer_lock(rwmu);
    dribbler_on = true;
}
void Actuator_System::turn_off_dribbler() { 
    writer_lock(rwmu);
    dribbler_on = false;
}
void Actuator_System::kick(float speed_x, float speed_y) { 
    writer_lock(rwmu);
    kick_speed_x = speed_x; 
    kick_speed_y = speed_y; 
}



void Actuator_System::timer_expire_callback() {
    reader_lock(rwmu);
    this->console->send_command(this->color == YELLOW ? true : false, this->id, 
            wheel_upper_left_vel, wheel_lower_left_vel, wheel_lower_right_vel, wheel_upper_right_vel,
            kick_speed_x, kick_speed_y, dribbler_on);   

    logger.log(Trace, "motion packet <" 
                + repr(wheel_upper_left_vel) + ", " 
                + repr(wheel_lower_left_vel) + ", "
                + repr(wheel_lower_right_vel) + ", "
                + repr(wheel_upper_right_vel) + ", "
                + "> sent!");

    this->timer->expires_from_now(milliseconds(ctrl_period_ms));
    this->timer->async_wait(boost::bind(&Actuator_System::timer_expire_callback, this));
}

void Actuator_System::set_ctrl_freq(float freq_Hz) {
    ctrl_period_ms = int((1.000 / freq_Hz) * 1000.000);
}

void Actuator_System::set_ctrl_period(float period_ms) {
    ctrl_period_ms = period_ms;
}


void Actuator_System::set_included_angle(float angle_degree) {
    inc_angle = angle_degree;
}



void Actuator_System::stop() {
    writer_lock(rwmu);
    wheel_upper_left_vel = 0.00;
    wheel_lower_left_vel = 0.00; 
    wheel_lower_right_vel = 0.00; 
    wheel_upper_right_vel = 0.00;
    kick_speed_x = 0.00;
    kick_speed_y = 0.00;
    dribbler_on = false;
}




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
void Actuator_System::move(arma::vec pwr_vec_3D) {
    if(arma::norm(pwr_vec_3D) > 100.00) {
        logger(Error) << "Warning: move vector exceeds magnitude 100.00";
    }
    
}
