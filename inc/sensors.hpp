#ifndef __SENSORS_H
#define __SENSORS_H

#include "common.hpp"
#include "grsim_vision.hpp"


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
        
        reader_writer_mutex rwmu;
        boost::mutex mu;


        boost::condition_variable_any cond_init_finished;

        arma::vec init_loc = {0, 0};
        arma::vec vec_v = {0, 0};
        float omega = 0.00;
        /*
        unsigned int sample_period_ms = 10; // millisec
        double zero_thresh = 0.001;
        unsigned int cnt_thresh = 5; 
        timer_ptr timer;
        

        arma::vec prev_vec_d = {0, 0};
        float prev_theta = 0.000;
        unsigned int prev_millis = 0;
        unsigned int prev_millis2 = 0;
        unsigned int cnt1 = 0, cnt2 = 0;
        bool is_first_time = true;
        */

        //----------------------------------//
        float translational_resolution = 5.000; // unit: millimeter
        float rotational_resolution = 1.5; // unit: degree 
        unsigned int trans_counter = 0;
        unsigned int orien_counter = 0;
        unsigned int counter_threshold = 3;
        //std::pair<arma::vec, double> disp_stamp; 
        //std::pair<float, double> orien_stamp;
        double disp_t_stamp = 0.00, orien_t_stamp = 0.00;
        arma::vec prev_disp = {0.00, 0.00};
        float prev_orien = 0.00;
        double prev_timestamp1;
        double prev_timestamp2;

        void vision_thread(udp::endpoint& v_ep);
       
        void timer_expire_callback();
       
        void on_packet_received();
        void on_location_changed(arma::vec);
        void on_orientation_changed(float);

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

};

// To-do: add ball-latched sensor

#endif