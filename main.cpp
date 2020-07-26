#include <string>
#include "common.hpp"
#include "systime.hpp"
#include "actuators.hpp"
#include "sensors.hpp"
#include "pid.hpp"
#include "protobuf-auto-gen/vFirmware_API.pb.h"

using namespace boost;
using namespace boost::asio;
using namespace arma;
using namespace rapidjson;
typedef boost::asio::ip::udp udp;
typedef boost::asio::ip::tcp tcp;
typedef boost::shared_ptr<ip::tcp::socket> socket_ptr;
typedef boost::shared_ptr<ip::udp::socket> udp_socket_ptr;

/* Some constants:
 * motor max speed around 416 rpm at 1N*m torque load
 * 416 rpm ~= 43.56 rad/s 
 */

std::ostream& operator<<(std::ostream& os, const arma::vec& v);

int main(int argc, char *argv[]) {
    B_Log::static_init();
    B_Log::sink->set_filter(severity >= Info);
    
    B_Log logger;

    bool tcp = true;
    unsigned int robot_id = 0;
    bool is_blue = true;
    std::string grSim_vision_ip;
    unsigned int grSim_vision_port = 0;
    char option;

    /* Protobuf Variables */
    VF_Commands commands;
    bool init = false;
    float trans_vec_x = 0;
    float trans_vec_y = 0;
    float rotate = 0;
    float kick_vec_x = 0;
    float kick_vec_y = 0;
    bool drib = false;

    unsigned int port = 0;
    double data_up_freq_hz = 1;

    while ( (option = getopt(argc,argv,":u")) != -1 ) {
        switch(option) {
            case 'u':
                tcp = false;
                break;
            case '?':
                printf("Unknown option: %c\n", optopt);
                break;
        } 
    }

    // 0 1 2 3
    // 1 2 3 4

    if ( optind + 3 == argc ) {
        for ( int i = optind; i < argc; i++ ) {
            if ( i == argc - 3 ) {
                port = std::stoi( std::string(argv[i]), nullptr, 10 );
            }
            if ( i == argc - 2 ) {
                robot_id = std::stoi( std::string(argv[i]), nullptr, 10 );
            }
            if ( i == argc - 1 ) {
                is_blue = std::stoi( std::string(argv[i]), nullptr, 10 );
            }
        }
    }
    else {
        logger(Error) << "Not enough arguments";
        return 1;
    }
    
    Document document;
    logger(Info) << "Reading from JSON file.";

    std::ifstream file("settings.json");
    if ( !file ) {
        logger(Error) << "JSON file does not exist.";
        return 1;
    }

    IStreamWrapper isw(file);
    document.ParseStream(isw);

    if (document.HasParseError()) {
        logger(Error) << "JSON file is not formatted correctly.";
        return 1;
    }

    for (auto& m : document.GetObject()) {
        //printf("Type of member %s is %s\n", m.name.GetString(), kTypeNames[m.value.GetType()]);
        if ( strncmp(m.name.GetString(), "data_up_freq_hz", 20) == 0 ) {
            if ( m.value.IsNumber() ) {
                data_up_freq_hz = m.value.GetDouble();
            }
            else {
                logger.log(Error, "data_up_freq_hz is invalid.");
            }
        }
        else if ( strncmp(m.name.GetString(), "grSim_vision_ip", 20) == 0 ) {
            if ( m.value.IsString() ) {
                grSim_vision_ip = m.value.GetString();
            }
            else {
                logger.log(Error, "grSim_vision_ip is invalid.");
            }
            
        }
        else if ( strncmp(m.name.GetString(), "grSim_vision_port", 20) == 0 ) {
            if ( m.value.IsNumber() ) {
                grSim_vision_port = m.value.GetUint();
            }
            else {
                logger.log(Error, "grSim_vision_port is invalid.");
            }
            
        }
    }  
    file.close(); 

// ================================================================================================================ //

    io_service service;

    udp::endpoint grsim_ssl_vision_ep(ip::address::from_string(grSim_vision_ip), grSim_vision_port);
    udp::endpoint grsim_console_ep(ip::address::from_string(LOCAL_HOST), 20011);
    
    Sensor_System sensors(is_blue?BLUE:YELLOW, robot_id, grsim_ssl_vision_ep);
    Actuator_System actuators(is_blue?BLUE:YELLOW, robot_id, grsim_console_ep);



    if ( tcp ) {
        ip::tcp::endpoint endpoint_to_listen(ip::tcp::v4(), port);
    
        ip::tcp::acceptor acceptor(service, endpoint_to_listen);

        cout << ">> Server started, port number: " << repr(port) << endl;

        socket_ptr socket(new ip::tcp::socket(service));

        try {
            acceptor.accept(*socket); // blocking func

            cout << "Accepted socket request from: " << socket->remote_endpoint().address().to_string() << endl;
        }
        catch(std::exception& e)
        {
            logger.log(Error, "[Exception]" + std::string(e.what()));
        }

        // read command from the client
        boost::thread cmd_thread([&]()     
        {
            B_Log logger;
            asio::streambuf read_buffer;
            std::istream input_stream(&read_buffer);
            std::string received;

            try{
                asio::streambuf init_buffer;
                std::istream init_stream(&init_buffer);
                logger.log(Info, "Waiting for censor init request...\n");
                
                while(!init) {
                    asio::read_until(*socket, init_buffer, "\n");
                    received = std::string(std::istreambuf_iterator<char>(init_stream), {});
                    commands.ParseFromString(received);
                    init = commands.init();
                    sensors.init();
                }
                
                while(true){
                    asio::streambuf read_buffer;
                    std::istream input_stream(&read_buffer);
                    // input_stream.clear();
                
                    asio::read_until(*socket, read_buffer, "\n");

                    received = std::string(std::istreambuf_iterator<char>(input_stream), {});
                    
                    commands.ParseFromString(received);
                    trans_vec_x = commands.translational_output().x();
                    trans_vec_y = commands.translational_output().y();
                    rotate = commands.rotational_output();
                    kick_vec_x = commands.kicker().x();
                    kick_vec_y = commands.kicker().y();
                    drib = commands.dribbler();

                    logger.log(Info,
                        "\ntranslational_output x y: " + repr(trans_vec_x) + " " + repr(trans_vec_y) + "\n" 
                        + "rotational_output: " + repr(rotate) + "\n"
                        + "kicker x y: " + repr(kick_vec_x) + " " + repr(kick_vec_y) + "\n"
                        + "dribbler: " + repr(drib) + "\n");

                    
                    arma::vec trans_vec = {trans_vec_x, trans_vec_y, rotate};
                    actuators.move(trans_vec);
                    actuators.kick(kick_vec_x, kick_vec_y);
                    if(drib) actuators.turn_on_dribbler();
                    else actuators.turn_off_dribbler();
                        
                }
            }
            catch (std::exception& e) {
                logger.log(Error, "[Exception]" + std::string(e.what()));
            }

        });

        // sent data to the client 
        boost::thread data_thread([&socket, &data_up_freq_hz]()     
        {
            B_Log logger;
            std::string write;

            try{
                while(true){
                    write = "Hello World!\n";
                    boost::asio::write(*socket, boost::asio::buffer(write));
                    delay(1.00/data_up_freq_hz * 1000.00);
                }
            }
            catch (std::exception& e) {
                logger.log(Error, "[Exception]" + std::string(e.what()));
            }
        });        

        cmd_thread.join();
        data_thread.join();
    }
    else {
        // ip::udp::endpoint ep_listen(ip::udp::v4(), port);

        // cout << ">> Server started, port number: " << repr(port) << endl;

        // udp_socket_ptr socket(new ip::udp::socket(service, ep_listen));

        // boost::array<char, 1024> receive_buffer; 

        // // read command from the client
        // boost::thread cmd_thread([&socket]()     
        // {
        //     B_Log logger;
        //     asio::streambuf read_buffer;
        //     std::istream input_stream(&read_buffer);
        //     std::string received;

        //     try{
        //         while(true){

        //             asio::read_until(*socket, read_buffer, "\n");

        //             received = std::string(std::istreambuf_iterator<char>(input_stream), {});
        //             logger.log(Info, received);
        //         }
        //     }
        //     catch (std::exception& e) {
        //         logger.log(Error, "[Exception]" + std::string(e.what()));
        //     }

        // });

        // cmd_thread.join();
    }

    return 0;

//     udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.2"), 10020);
//     udp::endpoint grsim_console_ep(ip::address::from_string(LOCAL_HOST), 20011);
    
//     Sensor_System sensors(BLUE, 0, grsim_ssl_vision_ep);
//     Actuator_System actuators(BLUE, 0, grsim_console_ep);

    


//     delay(500); 
//     /*
//     actuators.move(arma::vec("100 0 0"));
//     actuators.move(arma::vec("0 100 0"));
//     actuators.move(arma::vec("0 -100 0"));
//     actuators.move(arma::vec("-100 0 0"));
//     actuators.move(arma::vec("70.70 70.70 0"));
// */
//     // sqrt( x^2 + y^2 + z^2 ) <= 100
//     // x^2 + y^2 <= 10000

//     // mute all logs
//     B_Log::sink->set_filter(severity > Fatal);


//     double m1, m2, m3, m4;
//     double vx, vy, vw;
//     vec d = {0, 0}, v = {0, 0}, prev_d = {0, 0}, prev_v = {0, 0};
//     double theta, omega, prev_theta = 0.00, prev_omega = 0.00;
//     while(1) {
//         // std::cin >> m1 >> m2 >> m3 >> m4;
//         std::cin >> vx >> vy >> vw;
//         vec mv = {vx, vy, vw};
//         B_Log::set_shortest_format();
//         B_Log::sink->set_filter(tag_attr == "motion cmd" && severity == Trace);
        
//         sensors.init();
//         int t0 = millis();
//         while(millis() - t0 < 1000) {
//             // actuators.set_wheels_speeds(m1, m2, m3, m4);
//             actuators.move(mv);
//             d = sensors.get_translational_displacement();
//             v = sensors.get_translational_velocity();
//             theta = sensors.get_rotational_displacement();
//             omega = sensors.get_rotational_velocity();
//             // if(!arma::approx_equal(v, prev_v, "absdiff", 0,00001) || omega != prev_omega ) { 
//                 std::cout << d << " " 
//                           << v << " "
//                           << theta << " " 
//                           << omega 
//                           << " time: " << millis() - t0 << " ms" << std::endl;  
//             // }
//             delay(10);
//             prev_d = d; prev_v = v; prev_theta = theta; prev_omega = omega;
//         }
        
//         B_Log::sink->set_filter(severity >= Info);

//         t0 = millis();
//         while(millis() - t0 < 100) actuators.stop();
//     }

/*
    boost::mutex mu;
    boost::thread *threads[6];
    for(int i = 0; i < 6; i++) {
        threads[i] = new boost::thread( [i, &mu] () -> void  {
            io_service ios;
            
            mu.lock();
            std::cout << i << std::endl;
            mu.unlock();
            udp::endpoint ep_grsim_console(ip::address::from_string(LOCAL_HOST), 20011);
            GrSim_Console console(ios, ep_grsim_console);

            udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.3"), 10020);
            GrSim_Vision sim_vision(ios, grsim_ssl_vision_ep);

            while(1) {
                float speed = (i + 1) * 10;
                console.send_command(false, i, speed, speed, speed, speed, 0, 0, false);
                sim_vision.receive_packet();
                delay(10);
            }
        });
    }

    for(auto& thread : threads) {
        thread->join();
        delete thread;
    }
*/

}



std::ostream& operator<<(std::ostream& os, const arma::vec& v)
{
    char fmt_str[15]; // not so important size, greater than printed str size is fine, use magic number here 
    int num_rows = arma::size(v).n_rows;
    os << "<";
    for(int i = 0; i < num_rows; i++) {
        sprintf(fmt_str, "%8.3lf", v(i));
        os << std::string(fmt_str);
        if(i != num_rows - 1) os << ", ";
    }
    os << ">";
    return os;
}


/*
 * Armadillo C++ library Citation:
 * 
 * Conrad Sanderson and Ryan Curtin.
 * Armadillo: a template-based C++ library for linear algebra.
 * Journal of Open Source Software, Vol. 1, pp. 26, 2016.
 *
 * Conrad Sanderson and Ryan Curtin.
 * A User-Friendly Hybrid Sparse Matrix Class in C++.
 * Lecture Notes in Computer Science (LNCS), Vol. 10931, pp. 422-430, 2018. 
 */
