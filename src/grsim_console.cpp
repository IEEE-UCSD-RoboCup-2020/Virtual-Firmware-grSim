#include "grsim_console.hpp"
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
    writer_lock(rwmu);

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
}
