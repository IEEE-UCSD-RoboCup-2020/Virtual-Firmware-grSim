#include "grsim_vision.hpp"
#include "systime.hpp"


using namespace std;
using namespace arma;
using namespace boost;
using namespace boost::asio;
using namespace boost::posix_time;
using byte = unsigned char;



vec GrSim_Vision::blue_loc_vecs[NUM_ROBOTS];
vec GrSim_Vision::yellow_loc_vecs[NUM_ROBOTS];
double timestamp;


static double norm_ang(double ang) {
    ang = (ang > 180) ? ang - 360 : ang;
    ang = (ang < -180) ? ang + 360 : ang;
    return ang;
}


void GrSim_Vision::publish_robots_vinfo(
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
    team_color_t team_color) 
{
    writer_lock(rwmu);
    /*
     * x, y reversed due to global vision system views from the right hand side-wall view
     * which needs to be transformed to the viewing vector starting from goal of our side to
     * goal of opponent side 
     */
    for(auto& bot : robots) {
        
        if(team_color == BLUE) {
            if(bot.robot_id() >= NUM_ROBOTS || NUM_ROBOTS < 0){
                continue;
            }
            blue_loc_vecs[bot.robot_id()] = {-bot.y(), bot.x(), bot.orientation()};
            // print_robot_vinfo(bot); // for debugging
        }
        if(team_color == YELLOW) {
            if(bot.robot_id() >= NUM_ROBOTS || NUM_ROBOTS < 0){
                continue;
            }
            yellow_loc_vecs[bot.robot_id()] = {bot.y(), -bot.x(), bot.orientation()};
            // print_robot_vinfo(bot); // for debugging
        }
    }
}

void GrSim_Vision::publish_t_capture(double t_capture) {
    writer_lock(rwmu);
    timestamp = t_capture;
}
    

GrSim_Vision::GrSim_Vision(io_service& io_srvs, udp::endpoint& grsim_endpoint) {
    this->ios = &io_srvs;
    this->ep = &grsim_endpoint;
    this->receive_buffer = buffer_array_ptr(new boost::array<char, BUF_SIZE>());
    this->socket = socket_ptr(new udp::socket(io_srvs));

    socket->open(grsim_endpoint.protocol());
    socket->set_option(udp::socket::reuse_address(true));
    socket->bind(grsim_endpoint);
    socket->set_option(ip::multicast::join_group(grsim_endpoint.address()));
    writer_lock(rwmu);
    for(auto& vec : blue_loc_vecs) {
        vec = arma::vec("0 0 0");
    }
    for(auto& vec : yellow_loc_vecs) {
        vec = arma::vec("0 0 0");
    }
}
GrSim_Vision::~GrSim_Vision() {}

void GrSim_Vision::receive_packet() {
    size_t num_bytes_received;
    std::string packet_string;
    SSL_WrapperPacket packet;
    google::protobuf::RepeatedPtrField<SSL_DetectionRobot> *blue_robots, *yellow_robots;
    try {
        num_bytes_received = socket->receive_from(asio::buffer(*receive_buffer), *ep);
        packet_string = std::string(receive_buffer->begin(), 
                                    receive_buffer->begin() + num_bytes_received);

        packet.ParseFromString(packet_string);
        
        double t_curr = packet.detection().t_capture();


        publish_robots_vinfo(packet.detection().robots_blue(), BLUE);
        publish_robots_vinfo(packet.detection().robots_yellow(), YELLOW);

        if(t_curr > t_prev) {

            publish_t_capture(packet.detection().t_capture());

            // execute the callback functions passed by other classes
            for(auto& callback_function : this->on_packet_received_callbacks) {
                callback_function();
            }
        }
        t_prev = t_curr;

    }
    catch (std::exception& e) {
        // To-do : Exception Handling
        std::cout << "[Exception] " << e.what() << std::endl;
    }
}

//--------------------------------------------------------------------------------------------------//

void GrSim_Vision::async_receive_packet() {
    socket->async_receive_from(asio::buffer(*receive_buffer), *ep,
        boost::bind(&GrSim_Vision::async_receive_handler, this,
        asio::placeholders::bytes_transferred, asio::placeholders::error)
    );
}

void GrSim_Vision::async_receive_handler(std::size_t num_bytes_received,
                                     const boost::system::error_code& error) 
{    
    if(error) {
        std::cerr << "[Error Code] " << error.message() << std::endl;
    }

    std::string packet_string;
    SSL_WrapperPacket packet;
    google::protobuf::RepeatedPtrField<SSL_DetectionRobot> *blue_robots, *yellow_robots;

    packet_string = std::string(receive_buffer->begin(), 
                                    receive_buffer->begin() + num_bytes_received);

    packet.ParseFromString(packet_string);
    

    double t_curr = packet.detection().t_capture();


    
    publish_robots_vinfo(packet.detection().robots_blue(), BLUE);
    publish_robots_vinfo(packet.detection().robots_yellow(), YELLOW);

    if(t_curr > t_prev) {
        publish_t_capture(t_curr);

        // To-do add publish ball vinfo

        // std::cout << millis() << std::endl;

        // execute the callback functions passed by other classes
        for(auto& callback_function : this->on_packet_received_callbacks) {
            callback_function();
        }
    }

    t_prev = t_curr;
    // start the next receive cycle
    this->async_receive_packet();
}





vec GrSim_Vision::get_robot_location(team_color_t color, int robot_id) {
    reader_lock(rwmu);
    vec location;
    if(color == BLUE) {
        location = {GrSim_Vision::blue_loc_vecs[robot_id](0), 
                        GrSim_Vision::blue_loc_vecs[robot_id](1)};
    }
    else {
        location = {GrSim_Vision::yellow_loc_vecs[robot_id](0), 
                        GrSim_Vision::yellow_loc_vecs[robot_id](1)};
    }
    return location;
}

float GrSim_Vision::get_robot_orientation(team_color_t color, int robot_id) {
    reader_lock(rwmu);
    return color == BLUE ? to_degree( GrSim_Vision::blue_loc_vecs[robot_id](2) ) 
                         : norm_ang(to_degree( GrSim_Vision::yellow_loc_vecs[robot_id](2)) + 180);
}

double GrSim_Vision::get_timestamp_ms() {
    reader_lock(rwmu);
    return timestamp * 1000.00;
}





/*
// for debugging only
void GrSim_Vision::print_robot_vinfo(const SSL_DetectionRobot& robot) {
    std::cout << "ID[" << robot.robot_id() << "] "
                << "[<x,y>:(" << robot.x() << ", " << robot.y() << ")]"
                << "orien[" << robot.orientation() << "] "
                << "confidence[" << robot.confidence() << "]"
                << std::endl;
} 
*/


