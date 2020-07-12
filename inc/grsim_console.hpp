#ifndef __GRSIM_CONSOLE_H
#define __GRSIM_CONSOLE_H

#include "common.hpp"


class GrSim_Console{
private:
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
    static const unsigned int BUF_SIZE = 256;

    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;

    reader_writer_mutex rwmu;

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


#endif