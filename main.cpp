#include <string>
#include "common.hpp"
#include "systime.hpp"
#include "actuators.hpp"
#include "sensors.hpp"
#include "pid.hpp"
#include "protobuf-auto-gen/vFirmware_API.pb.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/error/en.h"


#define RECEIVE_BUFFER_SIZE 1024


using namespace boost;
using namespace boost::asio;
using namespace arma;
using namespace rapidjson;
typedef boost::asio::ip::udp udp;
typedef boost::asio::ip::tcp tcp;
typedef boost::shared_ptr<ip::tcp::socket> socket_ptr;
typedef boost::shared_ptr<ip::udp::socket> udp_socket_ptr;

struct argVals {
    bool tcp = true;
    unsigned int robot_id = 0;
    bool is_blue = true;
    std::string grSim_vision_ip;
    std::string grSim_console_ip;
    unsigned int grSim_vision_port;
    unsigned int grSim_console_port;

    unsigned int port = 0;
    double data_up_freq_hz = 1;

    bool failed = false;
};


/**
 * Get ip from domain name
 * Source: https://www.binarytides.com/hostname-to-ip-address-c-sockets-linux/
 */

int hostname_to_ip(const char* hostname , char* ip)
{
	struct hostent *he;
	struct in_addr **addr_list;
	int i;
		
	if ( (he = gethostbyname( hostname ) ) == NULL) {
		// get the host info
		herror("gethostbyname");
		return 1;
	}

	addr_list = (struct in_addr **) he->h_addr_list;
	
	for(i = 0; addr_list[i] != NULL; i++) {
		//Return the first one;
		strcpy(ip , inet_ntoa(*addr_list[i]) );
		return 0;
	}
	
	return 1;
}

/* Some constants:
 * motor max speed around 416 rpm at 1N*m torque load
 * 416 rpm ~= 43.56 rad/s 
 */

std::ostream& operator<<(std::ostream& os, const arma::vec& v);
void help_print();
argVals arg_proc( int count, char * args[], std::string filename, B_Log * logger );


/////////////////////////////////////////////////////////////////////////////////
// async callback handler declarations

static void on_socket_accepted(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                Sensor_System& sensors, 
                                Actuator_System& actuators,
                                deadline_timer& timer,
                                unsigned int data_up_period,
                                B_Log& logger,  
                                const system::error_code& error);

static void on_cmd_received(asio::ip::tcp::socket& socket, 
                                asio::streambuf& read_buf,  
                                Actuator_System& actuators,
                                Sensor_System& sensors,
                                B_Log& logger,  
                                const system::error_code& error);

static void on_timer_expired(asio::ip::tcp::socket& socket,
                              std::string& write_buf,
                              Sensor_System& sensors,
                              deadline_timer& timer,
                              unsigned int data_up_period,
                              B_Log& logger,
                              const system::error_code& error);
                              
static void on_data_sent(asio::ip::tcp::socket& socket,
                              std::string& write_buf,
                              Sensor_System& sensors,
                              deadline_timer& timer,
                              unsigned int data_up_period,
                              B_Log& logger,
                              const system::error_code& error);
/////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
    B_Log::static_init();
    B_Log::set_shorter_format();
    // B_Log::sink->set_filter(severity == Debug && tag_attr == "Main");
    B_Log::sink->set_filter(severity > Error); 
    
    B_Log logger;
    logger.add_tag("Main");

    struct argVals json_vars;



    json_vars = arg_proc( argc, argv, "robot_config.json", &logger );

    if ( json_vars.failed ) {
        return 1;
    }

// ================================================================================================================ //

    io_service service;

    // convert from possibly hostname to ip
    char ip[100];
    hostname_to_ip(json_vars.grSim_console_ip.c_str() , ip);
    std::string str(ip);
    json_vars.grSim_console_ip = str;

    // instantiate grSim simulator endpoints
    udp::endpoint grsim_ssl_vision_ep(ip::address::from_string(json_vars.grSim_vision_ip), json_vars.grSim_vision_port);
    udp::endpoint grsim_console_ep(ip::address::from_string(json_vars.grSim_console_ip), json_vars.grSim_console_port);
    
    // instantiate sensor and actuator systems
    // these systems get started running in their constructors
    Sensor_System sensors(json_vars.is_blue?BLUE:YELLOW, json_vars.robot_id, grsim_ssl_vision_ep);
    Actuator_System actuators(json_vars.is_blue?BLUE:YELLOW, json_vars.robot_id, grsim_console_ep);

    boost::mutex mu; 
    asio::streambuf read_buf;
    std::string write_buf;

    ip::tcp::endpoint endpoint_to_listen(ip::tcp::v4(), json_vars.port);
    ip::tcp::acceptor acceptor(service, endpoint_to_listen);
    ip::tcp::socket socket(service);

    deadline_timer timer(service);
    unsigned int data_upstream_period;

    if(json_vars.tcp) {

        logger.log(Info, "Server started on LocalHost, port number: " + repr(json_vars.port));
        data_upstream_period = 1.00/json_vars.data_up_freq_hz * 1000.00;
        //data_upstream_period = 1000;
        
        logger.log(Info, "Data Upstream Period (1/frequency) is " + repr(data_upstream_period) + " milliseconds");
        try {
            acceptor.async_accept(socket, boost::bind(&on_socket_accepted, 
                                                boost::ref(socket),
                                                boost::ref(write_buf), 
                                                boost::ref(read_buf), 
                                                boost::ref(sensors), 
                                                boost::ref(actuators),
                                                boost::ref(timer), 
                                                data_upstream_period,
                                                boost::ref(logger), 
                                                asio::placeholders::error)); 

            
        }
        catch(std::exception& e)
        {
            logger.log(Error, "[Exception]" + std::string(e.what()));
        }


    }
    else {
        logger(Error) << "UDP version is no longer supported, please use TCP";
    }


    service.run();
    return 0;

}





















/////////////////////////////////////////////////////////////////////////////////////////

static void on_cmd_received(asio::ip::tcp::socket& socket, 
                                asio::streambuf& read_buf,  
                                Actuator_System& actuators,
                                Sensor_System& sensors,
                                B_Log& logger,  
                                const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
    }
    VF_Commands commands;
    bool init = false;
    float trans_vec_x = 0;
    float trans_vec_y = 0;
    float rotate = 0;
    float kick_vec_x = 0;
    float kick_vec_y = 0;
    bool drib = false;

    std::istream input_stream(&read_buf);
    std::string received;
    received = std::string(std::istreambuf_iterator<char>(input_stream), {});
    
    commands.ParseFromString(received);
    if(commands.init()) {
        sensors.init();
    }
    trans_vec_x = commands.translational_output().x();
    trans_vec_y = commands.translational_output().y();
    rotate = commands.rotational_output();
    kick_vec_x = commands.kicker().x();
    kick_vec_y = commands.kicker().y();
    drib = commands.dribbler();

    arma::vec mov_vec = {trans_vec_x, trans_vec_y, rotate};
    actuators.move(mov_vec);
    actuators.kick(kick_vec_x, kick_vec_y);
    if(drib) actuators.turn_on_dribbler();
    else actuators.turn_off_dribbler();

    std::ostringstream debug_out_stream;
    debug_out_stream << "CMD Received: " 
                     << "Mov"  << mov_vec << " "
                     << "Kick" << "<" << kick_vec_x << ", " << kick_vec_y << "> "
                     << "Drib" << "<" << drib << ">"
                     << std::endl; 

    logger.log(Info, debug_out_stream.str());
    
    // schedule the next task (similar to recursion)
    boost::asio::async_read_until(socket, read_buf, "\n",
                            boost::bind(&on_cmd_received, 
                                        boost::ref(socket), 
                                        boost::ref(read_buf), 
                                        boost::ref(actuators), 
                                        boost::ref(sensors),
                                        boost::ref(logger), 
                                        asio::placeholders::error)); 
    
}

static void on_timer_expired(asio::ip::tcp::socket& socket,
                              std::string& write_buf,
                              Sensor_System& sensors,
                              deadline_timer& timer,
                              unsigned int data_up_period,
                              B_Log& logger,
                              const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
    }
    VF_Data data;
    arma::vec tdisp, tvel;
    Vec_2D trans_disp, trans_vel;
    float rotat_disp, rotat_vel;
    std::ostringstream debug_out_stream;

    write_buf.clear();

    tdisp = sensors.get_translational_displacement();
    trans_disp.set_x(tdisp(0));
    trans_disp.set_y(tdisp(1));
    data.set_allocated_translational_displacement(&trans_disp);

    tvel = sensors.get_translational_velocity();
    trans_vel.set_x(tvel(0));
    trans_vel.set_y(tvel(1));
    data.set_allocated_translational_velocity(&trans_vel);
    
    rotat_disp = sensors.get_rotational_displacement();
    rotat_vel = sensors.get_rotational_velocity();
    data.set_rotational_displacement(rotat_disp);
    data.set_rotational_velocity(rotat_vel);

    data.SerializeToString(&write_buf);
    write_buf += "\n";

    debug_out_stream << "Data Sent: "
                     << "Trans[" << tdisp << "] ["  
                     << tvel << "] "  
                     << "Rotat[" << rotat_disp << "] ["
                     << rotat_vel << "]" << std::endl;

    logger.log(Debug, debug_out_stream.str());

    data.release_translational_displacement();
    data.release_translational_velocity();

    // schedule the next task (similar to recursion)
    boost::asio::async_write(socket, boost::asio::buffer(write_buf),
                                boost::bind(&on_data_sent, 
                                            boost::ref(socket),
                                            boost::ref(write_buf),
                                            boost::ref(sensors),
                                            boost::ref(timer), 
                                            data_up_period,
                                            boost::ref(logger), 
                                            asio::placeholders::error));


}
static void on_data_sent(asio::ip::tcp::socket& socket,
                              std::string& write_buf,
                              Sensor_System& sensors,
                              deadline_timer& timer,
                              unsigned int data_up_period,
                              B_Log& logger,
                              const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
    }

    // schedule the next task (similar to recursion)
    timer.expires_from_now(posix_time::milliseconds(data_up_period));
    timer.async_wait(boost::bind(&on_timer_expired, 
                                boost::ref(socket),
                                boost::ref(write_buf),
                                boost::ref(sensors),
                                boost::ref(timer), 
                                data_up_period,
                                boost::ref(logger), 
                                asio::placeholders::error)
                                );
}



static void on_socket_accepted(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                Sensor_System& sensors, 
                                Actuator_System& actuators,
                                deadline_timer& timer,
                                unsigned int data_up_period,
                                B_Log& logger,  
                                const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
    }
    // To-Do : add color code
    logger.log(Info, "Accepted socket request from: " 
                        + socket.remote_endpoint().address().to_string());

    
    // timer driven Data upstream async task
    timer.expires_from_now(posix_time::milliseconds(data_up_period));
    timer.async_wait(boost::bind(&on_timer_expired, 
                                boost::ref(socket),
                                boost::ref(write_buf),
                                boost::ref(sensors),
                                boost::ref(timer), 
                                data_up_period,
                                boost::ref(logger), 
                                asio::placeholders::error)
                                );
    
    // socket event driven Command download async task
    boost::asio::async_read_until(socket, read_buf, "\n",
                                boost::bind(&on_cmd_received, 
                                            boost::ref(socket), 
                                            boost::ref(read_buf), 
                                            boost::ref(actuators), 
                                            boost::ref(sensors),
                                            boost::ref(logger), 
                                            asio::placeholders::error)); 
    
}


/////////////////////////////////////////////////////////////////////////////////////////
























void help_print() {
    printf("Command:\n");
    printf("./vfirm.exe (-u) <port> <robot_id> <is_blue>\n\n");

    printf("Execute Parameters:\n");
    printf("By default, launches TCP server.\n");
    printf("\t-u: Changes to host UDP server.\n");
    printf("\t<port>: Port to host server on.\n");
    printf("\t<robot_id>: Defines which robot to control. Must be a non-negative number.\n");
    printf("\t<is_blue>: Defines which team to control. Must be either 1 (true) or 0 (false)\n");
}

argVals arg_proc( int count, char * args[], std::string filename, B_Log * logger ) {
    struct argVals ret;
    char option;

    while ( (option = getopt(count,args,":u")) != -1 ) {
        switch(option) {
            case 'u':
                ret.tcp = false;
                break;
            case '?':
                printf("Unknown option: %c\n", optopt);
                help_print();
                ret.failed = true;
                return ret;
                break;
        } 
    }

    if ( optind + 3 == count ) {
        for ( int i = optind; i < count; i++ ) {
            if ( i == count - 3 ) {
                ret.port = std::stoi( std::string(args[i]), nullptr, 10 );
            }
            if ( i == count - 2 ) {
                ret.robot_id = std::stoi( std::string(args[i]), nullptr, 10 );
            }
            if ( i == count - 1 ) {
                ret.is_blue = std::stoi( std::string(args[i]), nullptr, 10 );
            }
        }
    }
    else {
        (*logger)(Error) << "Not enough arguments\n";
        help_print();
        ret.failed = true;
        return ret;
    }
    
    Document document;
    (*logger)(Info) << "Reading from JSON file";

    std::ifstream file(filename);
    if ( !file ) {
        (*logger)(Error) << "JSON file does not exist.\n";
        ret.failed = true;
        return ret;
    }

    IStreamWrapper isw(file);
    document.ParseStream(isw);

    if (document.HasParseError()) {
        (*logger)(Error) << "JSON file is not formatted correctly.\n";
        ret.failed = true;
        return ret;
    }

    for (auto& m : document.GetObject()) {
        //printf("Type of member %s is %s\n", m.name.GetString(), kTypeNames[m.value.GetType()]);
        /* ================ Connection Information ================= */
        if ( strncmp(m.name.GetString(), "grSim_vision_ip", 20) == 0 ) {
            if ( m.value.IsString() ) {
                ret.grSim_vision_ip = m.value.GetString();
            }
            else {
                logger->log(Error, "grSim_vision_ip is invalid.");
                ret.failed = true;
                return ret;
            }
        }
        else if ( strncmp(m.name.GetString(), "grSim_vision_port", 20) == 0 ) {
            if ( m.value.IsNumber() ) {
                ret.grSim_vision_port = m.value.GetUint();
            }
            else {
                logger->log(Error, "grSim_vision_port is invalid.");
                ret.failed = true;
                return ret;
            }
        }
        else if(strncmp(m.name.GetString(), "grSim_console_ip", 20) == 0) {
            if ( m.value.IsString() ) {
                ret.grSim_console_ip = m.value.GetString();
            }
            else {
                logger->log(Error, "grSim_console_ip is invalid.");
                ret.failed = true;
                return ret;
            }
        }
        else if(strncmp(m.name.GetString(), "grSim_console_port", 20) == 0) {
            if ( m.value.IsNumber() ) {
                ret.grSim_console_port = m.value.GetUint();
            }
            else {
                logger->log(Error, "grSim_console_port is invalid.");
                ret.failed = true;
                return ret;
            }
        }
        /* ====================== Server Settings ======================= */
        else if ( strncmp(m.name.GetString(), "data_up_freq_hz", 20) == 0 ) {
            if ( m.value.IsNumber() ) {
                ret.data_up_freq_hz = m.value.GetDouble();
            }
            else {
                logger->log(Error, "data_up_freq_hz is invalid.");
                ret.failed = true;
                return ret;
            }
        }
        /* ====================== Robot Parameters ===================== */
    }  
    file.close();

    return ret;
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
