#include "sensors.hpp"
#include "systime.hpp"


using namespace std;
using namespace arma;
using namespace boost;
using namespace boost::asio;
using namespace boost::posix_time;
using byte = unsigned char;



vec GrSim_Vision::blue_loc_vecs[NUM_ROBOTS];
vec GrSim_Vision::yellow_loc_vecs[NUM_ROBOTS];

void GrSim_Vision::publish_robots_vinfo(
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
    team_color_t team_color) 
{
    /*
     * x, y reversed due to global vision system views from the right hand side-wall view
     * which needs to be transformed to the viewing vector starting from goal of our side to
     * goal of opponent side 
     */
    for(auto& bot : robots) {
        mu.lock();
        if(team_color == BLUE) {
            blue_loc_vecs[bot.robot_id()] = {-bot.y(), bot.x(), bot.orientation()};
            // print_robot_vinfo(bot); // for debugging
        }
        if(team_color == YELLOW) {
            yellow_loc_vecs[bot.robot_id()] = {-bot.y(), bot.x(), bot.orientation()};
            // print_robot_vinfo(bot); // for debugging
        }
        mu.unlock();
    }
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
    mu.lock();
    for(auto& vec : blue_loc_vecs) {
        vec = arma::vec("0 0 0");
    }
    for(auto& vec : yellow_loc_vecs) {
        vec = arma::vec("0 0 0");
    }
    mu.unlock();
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
        
        publish_robots_vinfo(packet.detection().robots_blue(), BLUE);
        publish_robots_vinfo(packet.detection().robots_yellow(), YELLOW);

    }
    catch (std::exception& e) {
        // To-do : Exception Handling
        std::cout << "[Exception] " << e.what() << std::endl;
    }
}


void GrSim_Vision::async_receive_packet() {
    
    socket->async_receive_from(asio::buffer(*receive_buffer), *ep,
        boost::bind(&GrSim_Vision::on_receive_packet, this,
        asio::placeholders::bytes_transferred, asio::placeholders::error)
    );
}

void GrSim_Vision::on_receive_packet(std::size_t num_bytes_received,
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
    
    publish_robots_vinfo(packet.detection().robots_blue(), BLUE);
    publish_robots_vinfo(packet.detection().robots_yellow(), YELLOW);

    // To-do add publish ball vinfo

    // start the next receive cycle
    this->async_receive_packet();
}





vec GrSim_Vision::get_robot_location(team_color_t color, int robot_id) {
    mu.lock();
    vec location;
    if(color == BLUE) {
        location = {GrSim_Vision::blue_loc_vecs[robot_id](0), 
                        GrSim_Vision::blue_loc_vecs[robot_id](1)};
    }
    else {
        location = {GrSim_Vision::yellow_loc_vecs[robot_id](0), 
                        GrSim_Vision::yellow_loc_vecs[robot_id](1)};
    }
    mu.unlock();
    return location;
}

float GrSim_Vision::get_robot_orientation(team_color_t color, int robot_id) {
    mu.lock();
    float ret = color == BLUE ? to_degree( GrSim_Vision::blue_loc_vecs[robot_id](2) ) 
                         : to_degree( GrSim_Vision::yellow_loc_vecs[robot_id](2) );
    mu.unlock();
    return ret;
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


// ==================================================================================================== //

// Gorgeous divide line between two different classes :)

// ==================================================================================================== //



Sensor_System::Sensor_System(team_color_t color, int robot_id, udp::endpoint& grsim_vision_ep) {
    this->color = color;
    this->id = robot_id;
    
    //save the thread_ptr copy to extend the life scope of the smart pointer thread_ptr 
    v_thread = thread_ptr(
        new boost::thread(boost::bind(&Sensor_System::vision_thread, this, grsim_vision_ep))
    );
    mu.lock();
    cond_init_finished.wait(mu);
    mu.unlock();
}

void Sensor_System::vision_thread(udp::endpoint& v_ep) {
    io_service ios;
    this->timer = timer_ptr(new deadline_timer(ios));
    this->vision = GrSim_Vision_ptr(new GrSim_Vision(ios, v_ep));
    cond_init_finished.notify_all();
    /* sync way
    while(1) {
        // collecting vision data packets from grSim in a background-running thread
        this->vision->receive_packet(); 
        // ....
    } */

    // async way
    this->vision->async_receive_packet();
    this->timer->expires_from_now(milliseconds(sample_period_ms));
    this->timer->async_wait(boost::bind(&Sensor_System::timer_expire_callback, this));
    
    ios.run();
}





void Sensor_System::init() {
    set_init_displacement();
    vec prev_vec_d = {0, 0};
    prev_theta = 0.000;
    is_first_time = true;
}


/*** All methods below returns coordinate relative to robot's own body frame ***/

void Sensor_System::set_init_displacement() {
    // set the current location as the (0,0) location vector for this robot
    init_loc = this->vision->get_robot_location(this->color, this->id);
}

// Getter for \vec{d} and \theta (physics)
/* get net translational displacement (which is the 2D Location vector relative to robot's body reference frame)
    used to simulate the motor encoder vector addition cumulation 
    unit: mm */
arma::vec Sensor_System::get_translational_displacement() {
    mat rot = rotation_matrix_2D(get_rotational_displacement());
    vec body_frame_x = rot * unit_vec_x;
    vec body_frame_y = rot * unit_vec_y;
    mat change_basis = change_basis_matrix_2D(body_frame_x, body_frame_y);
    vec loc_in_world_frame = (this->vision->get_robot_location(this->color, this->id) - init_loc);
    return change_basis * loc_in_world_frame;
}

/* get the rotational displacement in degrees (which is the orientation)
    used to simulate the EKF[encoder difference cumulation + IMU orientation estimation(another ekf within)]
   unit: degree*/
float Sensor_System::get_rotational_displacement() { 
    // +degree left rotation (0~180)
    // -degree right rotation (0~-180)
    return this->vision->get_robot_orientation(this->color, this->id); 
}


// Getter for \vec{v} and \omega (physics)
/* get the translational velocity vector, simulating encoder sensor
   unit: mm/s */
arma::vec Sensor_System::get_translational_velocity() {
    /* measurement taken in a concurrently running thread */
    mu.lock();
    auto ret = this->vec_v * 1000.00; // convert from m/s to mm/s
    mu.unlock();
    return ret;
}

/* get the rotational speed, simulating EKF[Gyro within IMU + Encoder estimation]
   unit: degree/s */
float Sensor_System::get_rotational_velocity() {
    mu.lock();
    auto ret = this->omega * 1000.00;
    mu.unlock();
    return ret;
}


// config the sample rate of the velocity trackers
inline void Sensor_System::set_velocity_sample_rate(unsigned int rate_Hz) {
    sample_period_ms = (1.00 / (double)rate_Hz) * 1000.000;
}


// callback that calculates velocities
void Sensor_System::timer_expire_callback() {

    // Optional To-do: add a low pass filter (queue to get data list, pop oldest and push latest to keep the buffer size small)

    /* calc velocities */

    arma::vec curr_vec_d = this->get_translational_displacement();
    float curr_theta = this->get_rotational_displacement();
    if(curr_theta < 0.000) curr_theta = 360 + curr_theta;

    this->mu.lock();

    if(is_first_time) {
        prev_vec_d = curr_vec_d;
        prev_theta = curr_theta;
        prev_millis = millis();
        prev_millis2 = millis();
        is_first_time = false;

        this->mu.unlock(); // don't forget to unlock before function returns!
        // start the next timer cycle
        this->timer->expires_from_now(milliseconds(sample_period_ms));
        this->timer->async_wait(boost::bind(&Sensor_System::timer_expire_callback, this));
        return;
    }

    
    arma::vec disp_diff = curr_vec_d - prev_vec_d;
    // std::cout << disp_diff << std::endl; // debug
    if( disp_diff(0) >= -zero_thresh && disp_diff(0) <= zero_thresh &&
       disp_diff(1) >= -zero_thresh && disp_diff(1) <= zero_thresh && cnt1 < cnt_thresh){ 
        // if delta is too small to be meaningful
        cnt1++;
    }
    else {
        
        this->vec_v = (disp_diff) / (millis() - prev_millis);
        prev_vec_d = curr_vec_d;
        prev_millis = millis();
        cnt1 = 0;
    }
    
    double angle_diff = curr_theta - prev_theta;
    if(angle_diff >= -zero_thresh && angle_diff <= zero_thresh && cnt2 < cnt_thresh) {
       cnt2++;
    }
    else {
        this->omega = (angle_diff) / (millis() - prev_millis2);    
        prev_theta = curr_theta;
        prev_millis2 = millis();
        cnt2 = 0;
    }

    this->mu.unlock();

    // start the next timer cycle
    this->timer->expires_from_now(milliseconds(sample_period_ms));
    this->timer->async_wait(boost::bind(&Sensor_System::timer_expire_callback, this));
}