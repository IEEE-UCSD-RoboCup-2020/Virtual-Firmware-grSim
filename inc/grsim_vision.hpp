#ifndef __GRSIM_VISION_H
#define __GRSIM_VISION_H

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
        
        reader_writer_mutex rwmu;

        std::vector<boost::function<void(void)>> on_packet_received_callbacks;

        void publish_robots_vinfo(const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
                                team_color_t team_color);
        void publish_t_capture(double t_capture);

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
        double get_timestamp_ms();

        // to-do add get ball and other info

};


#endif