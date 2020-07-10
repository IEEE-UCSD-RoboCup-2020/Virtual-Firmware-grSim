#ifndef __SENSORS_H
#define __SENSORS_H

#include "common.hpp"


class GrSim_Vision { 
    // coordinate data are relative to reverted global vision coordinate

    private:   
        typedef boost::asio::ip::udp udp;
        typedef boost::asio::io_service io_service;

        static const unsigned int BUF_SIZE = 115200;
        static arma::vec blue_loc_vecs[NUM_ROBOTS];
        static arma::vec yellow_loc_vecs[NUM_ROBOTS];
        typedef boost::shared_ptr<udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
        typedef boost::shared_ptr<boost::array<char, BUF_SIZE>> buffer_array_ptr;

        io_service *ios;
        udp::endpoint *ep;
        socket_ptr socket;
        buffer_array_ptr receive_buffer; 
        udp::endpoint *local_listen_ep;
        boost::mutex mu;

        std::vector<boost::function<void(void)>> on_packet_received_callbacks;

        void publish_robots_vinfo(const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
                                team_color_t team_color);

        void async_receive_handler(std::size_t num_bytes_received,
                            const boost::system::error_code& error);
        
    public:

        GrSim_Vision(io_service& io_srvs, udp::endpoint& grsim_endpoint);
        ~GrSim_Vision();


        inline void add_on_packet_received_callback(boost::function<void(void)> callback_function) {
            this->on_packet_received_callbacks.push_back(callback_function); // boost::function is also like a smart pointer
        }


        void receive_packet();
        void async_receive_packet();

        arma::vec get_robot_location(team_color_t color, int robot_id);
        float get_robot_orientation(team_color_t color, int robot_id);

        // to-do add get ball and other info

};

class Sensor_System { // corresponding to one particular robot, though multiple robots shared the same vision data source
    private:
        typedef boost::asio::ip::udp udp;
        typedef boost::asio::io_service io_service;
        typedef boost::shared_ptr<GrSim_Vision> GrSim_Vision_ptr;
        typedef boost::shared_ptr<boost::thread> thread_ptr;
        typedef boost::shared_ptr<boost::asio::deadline_timer> timer_ptr;

        team_color_t color;
        int id;
        GrSim_Vision_ptr vision;
        thread_ptr v_thread;
        boost::mutex mu;
        boost::condition_variable_any cond_init_finished;
        unsigned int sample_period_ms = 10; // millisec
        double zero_thresh = 0.001;
        unsigned int cnt_thresh = 5; 
        timer_ptr timer;

        arma::vec init_loc = {0, 0};
        arma::vec vec_v = {0, 0};
        float omega = 0.00;
        arma::vec prev_vec_d = {0, 0};
        float prev_theta = 0.000;
        unsigned int prev_millis = 0;
        unsigned int prev_millis2 = 0;
        unsigned int cnt1 = 0, cnt2 = 0;
        bool is_first_time = true;
    
        void vision_thread(udp::endpoint& v_ep);
        void timer_expire_callback();
        void on_packet_received();

        arma::vec prev_loc = {0.00, 0.00};
        float prev_orien = 0.00;
        void on_location_changed();
        void on_orientation_changed();

    public:
        
        Sensor_System(team_color_t color, int robot_id, udp::endpoint& grsim_vision_ep);

        void init();

        /*** All methods below returns coordinate relative to robot's own body frame ***/

        void set_init_displacement();

        // Getter for \vec{d} and \theta (physics)
        /* get net translational displacement (which is the 2D Location vector)
        used to simulate the motor encoder vector addition cumulation */
        arma::vec get_translational_displacement(); // unit: millimeter

        /* get the rotational displacement  (which is the orientation)
        used to simulate the EKF[encoder difference cumulation + IMU orientation estimation(another ekf within)]*/
        float get_rotational_displacement(); // unit: degree // +degree left rotation (0~180) // -degree right rotation (0~-180)

        // Getter for \vec{v} and \omega (physics)
        /* get the translational velocity vector, simulating encoder sensor*/
        arma::vec get_translational_velocity(); // unit: millimeter/millisecond == m/s

        /* get the rotational speed, simulating EKF[Gyro within IMU + Encoder estimation]*/
        float get_rotational_velocity(); // unit: degree/millisecond == deg/ms

        // config the sample rate of the velocity trackers
        void set_velocity_sample_rate(unsigned int rate_Hz); 
        inline void set_velocity_zero_thresh(unsigned int zero_thresh) {
            this->zero_thresh = zero_thresh;
        }
        inline void set_velocity_cnt_thresh(unsigned int cnt_thresh) {
            this->cnt_thresh = cnt_thresh;
        }
};

// To-do: add ball-latched sensor

#endif