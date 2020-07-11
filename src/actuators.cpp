#include "actuators.hpp"
#include "systime.hpp"


using namespace arma;
using namespace boost;
using namespace boost::asio;
using namespace boost::posix_time;


GrSim_Console::GrSim_Console(io_service& io_srvs, udp::endpoint& endpoint) {
    this->ios = &io_srvs;
    this->ep = &endpoint;
    this->socket = socket_ptr(new udp::socket(io_srvs));
    this->socket->open(udp::v4());
}

GrSim_Console::~GrSim_Console() {}

void GrSim_Console::send_command(bool is_team_yellow, int id, 
                    float upper_left_wheel_speed, float lower_left_wheel_speed,
                    float lower_right_wheel_speed, float upper_right_wheel_speed, 
                   // float x, float y, float omega, 
                    float kick_speed_x, float kick_speed_y, bool spinner) 
{
    mu.lock();

    grSim_Packet packet;
    
    packet.mutable_commands()->set_isteamyellow(is_team_yellow);
    packet.mutable_commands()->set_timestamp(millis());


    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(id);
    command->set_wheelsspeed(true);

    command->set_wheel1(upper_left_wheel_speed); // upper_left
    command->set_wheel2(lower_left_wheel_speed); // lower_left
    command->set_wheel3(lower_right_wheel_speed); // lower_right
    command->set_wheel4(upper_right_wheel_speed); // upper_right
    command->set_veltangent(0.00);
    command->set_velnormal(0.00);
    command->set_velangular(0.00);

    command->set_kickspeedx(kick_speed_x);
    command->set_kickspeedz(kick_speed_y);
    command->set_spinner(spinner);


    char to_send[BUF_SIZE];
    packet.SerializeToArray(to_send, packet.ByteSizeLong());

    try {
        socket->send_to(asio::buffer(to_send), *ep);
    }
    catch (std::exception& e) {
        // To-do : Exception Handling & sync
        std::cout << "[Exception] " << e.what() << std::endl;
    }

    mu.unlock();
}

// ==================================================================================================== //

// Gorgeous divide line between two different classes :)

// ==================================================================================================== //


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

}

void Actuator_System::send_cmd_thread(udp::endpoint& c_ep) {
    io_service ios;
    this->timer = timer_ptr(new deadline_timer(ios));
    this->console = GrSim_Console_ptr(new GrSim_Console(ios, c_ep));
    cond_init_finished.notify_all();

    // timer-driven background cmd-sending async task
    this->timer->expires_from_now(milliseconds(ctrl_period_ms));
    this->timer->async_wait(boost::bind(&Actuator_System::timer_expire_callback, this));
    
    ios.run();
}

void Actuator_System::timer_expire_callback() {
  
    this->console->send_command(this->color == YELLOW ? true : false, this->id, 
            wheel_upper_left_vel, wheel_lower_left_vel, wheel_lower_right_vel, wheel_upper_right_vel,
            kick_speed_x, kick_speed_y, dribbler_on);

    this->timer->expires_from_now(milliseconds(ctrl_period_ms));
    this->timer->async_wait(boost::bind(&Actuator_System::timer_expire_callback, this));
}

void Actuator_System::set_ctrl_freq(float freq_Hz) {
    ctrl_period_ms = int((1.000 / freq_Hz) * 1000.000);
}

void Actuator_System::set_ctrl_period(float period_ms) {
    ctrl_period_ms = period_ms;
}




void Actuator_System::stop() {
    mu.lock();
    wheel_upper_left_vel = 0.00;
    wheel_lower_left_vel = 0.00; 
    wheel_lower_right_vel = 0.00; 
    wheel_upper_right_vel = 0.00;
    kick_speed_x = 0.00;
    kick_speed_y = 0.00;
    dribbler_on = false;
    mu.unlock();
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
        std::cout << "Warning: move vector exceeds magnitude 100.00" << std::endl;
    }
    
}
