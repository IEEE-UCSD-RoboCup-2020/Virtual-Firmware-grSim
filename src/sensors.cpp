#include "sensors.hpp"
#include "systime.hpp"

using namespace std;
using namespace arma;
using namespace boost;
using namespace boost::asio;
using namespace boost::posix_time;
using byte = unsigned char;


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


    logger << "\033[0;32m sensor system initialized \033[0m";
    logger.add_tag("vision data");
}

void Sensor_System::vision_thread(udp::endpoint& v_ep) {
    io_service ios;
    // this->timer = timer_ptr(new deadline_timer(ios));
    this->vision = GrSim_Vision_ptr(new GrSim_Vision(ios, v_ep));
    this->vision->add_on_packet_received_callback(boost::bind(&Sensor_System::on_packet_received, this));

    logger << "start receiving udp multicast packets from grSim";
    cond_init_finished.notify_all();
    /* sync way
    while(1) {
        // collecting vision data packets from grSim in a background-running thread
        this->vision->receive_packet(); 
        // ....
    } */

    
    
    // async way
    this->vision->async_receive_packet();
    //this->timer->expires_from_now(milliseconds(sample_period_ms));
    //this->timer->async_wait(boost::bind(&Sensor_System::timer_expire_callback, this));
    
    ios.run();
}


void Sensor_System::init() {
    init_displacement();

    prev_orien = get_rotational_displacement();
    orien_t_stamp = this->vision->get_timestamp_ms();
    prev_disp = get_translational_displacement();
    disp_t_stamp = this->vision->get_timestamp_ms();
    /*
    vec prev_vec_d = {0, 0};
    prev_theta = 0.000;
    is_first_time = true; */
}


/*** All methods below returns coordinate relative to robot's own body frame ***/

void Sensor_System::init_displacement() {
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
    reader_lock(rwmu);
    return this->vec_v * 1000.00; // convert from m/s to mm/s
}

/* get the rotational speed, simulating EKF[Gyro within IMU + Encoder estimation]
   unit: degree/s */
float Sensor_System::get_rotational_velocity() {
    reader_lock(rwmu);
    return this->omega * 1000.00;
}

double Sensor_System::get_curr_timestamp() {
    return this->vision->get_timestamp_ms();
}



/* note: this callback is running on the thread this->vision object runs, 
   don't forget synchronization */
void Sensor_System::on_packet_received() {
    
    // logger(Trace) << "\033[0;31m <======================packet received<=====================> \033[0m"; 

    vec curr_disp = this->get_translational_displacement();
    float curr_orien = this->get_rotational_displacement();
    double curr_t_stamp = this->vision->get_timestamp_ms();

    logger.log(Debug, "[curr_displacement]: " + repr(curr_disp(0)) + ", " + repr(curr_disp(1)) 
                    + " [curr_orientation]: " + repr(curr_orien)
                    + "[curr_timestamp]: " + repr(curr_t_stamp));
    
    logger.log(Trace, "disp, " + repr(curr_disp(0)) + ", " + repr(curr_disp(1)) 
                    + ", " + repr(curr_orien)
                    + ", " + repr(curr_t_stamp));


    vec disp_diff = curr_disp - prev_disp;

    float orien_diff; 
    // transistion from +180 to -180
    if(curr_orien < -150.00 && prev_orien > 150.00) {
        orien_diff = (180 + curr_orien) + (180 - prev_orien);
    }
    // transistion from -180 to +180
    else if(curr_orien > 150.00 && prev_orien < -150.00) {
        orien_diff = (curr_orien - 180) - (prev_orien + 180);
    }
    else {
        orien_diff = curr_orien - prev_orien;
    }


    // translational timeout
    if((curr_t_stamp - disp_t_stamp) > vel_reset_period && norm(disp_diff) <=  translational_resolution) {
        writer_lock(rwmu);       
        this->vec_v = {0, 0};
        logger.log(Trace, "vel, " + repr(vec_v(0)) + ", " + repr(vec_v(1)) 
                            + ", " + repr(vision->get_timestamp_ms()));
        prev_disp = curr_disp;
        disp_t_stamp = this->vision->get_timestamp_ms();
        return;
    }

    // rotational timeout
    if((curr_t_stamp - orien_t_stamp) > vel_reset_period && fabs(orien_diff) <= rotational_resolution) {
        writer_lock(rwmu);       
        this->omega = 0.0;
        logger.log(Trace, "rvel, " + repr(omega) 
                       + ", " + repr(vision->get_timestamp_ms()));
        prev_orien = curr_orien;
        orien_t_stamp = this->vision->get_timestamp_ms();
        return;
    }

    // on location having meaningful amount of change
    if(norm(disp_diff) >  translational_resolution) {
        this->on_location_changed(disp_diff / (curr_t_stamp - disp_t_stamp) );
        prev_disp = curr_disp;
        disp_t_stamp = this->vision->get_timestamp_ms();
    }

    // on rotation having meaningful amount of change
    if(fabs(orien_diff) > rotational_resolution) {
        this->on_orientation_changed(orien_diff / (curr_t_stamp - orien_t_stamp));
        prev_orien = curr_orien;
        orien_t_stamp = this->vision->get_timestamp_ms();
    }



}


void Sensor_System::on_location_changed(arma::vec disp_vel) {
    /*
    if(trans_counter == 0) {
        disp_stamp.first = curr_disp;
        disp_stamp.second = this->vision->get_timestamp_ms();
        trans_counter++;
    }
    else if(trans_counter < counter_threshold) {
        trans_counter++;
    }
    else {
        trans_counter = 0;
        vec trans_vel = (curr_disp - disp_stamp.first) / float(this->vision->get_timestamp_ms() - disp_stamp.second);
        std::cout << trans_vel << std::endl;
    }*/
    // std::cout << disp_vel << std::endl;
    writer_lock(rwmu);
    this->vec_v = disp_vel;
    // logger.log(Trace, "[\033[0;32m On location changed: displacement velocity]\033[0m, " + repr(vec_v(0)) + "," + repr(vec_v(1))); 
    logger.log(Trace, "vel, " + repr(vec_v(0)) + ", " + repr(vec_v(1)) 
                            + ", " + repr(vision->get_timestamp_ms()));

}


void Sensor_System::on_orientation_changed(float rot_vel) {
    writer_lock(rwmu);
    this->omega = rot_vel;
    logger.log(Trace, "rvel, " + repr(omega) 
                            + ", " + repr(vision->get_timestamp_ms()));
}














/*
// config the sample rate of the velocity trackers
inline void Sensor_System::set_velocity_sample_rate(unsigned int rate_Hz) {
    sample_period_ms = (1.00 / (double)rate_Hz) * 1000.000;
}
*/

/*
// callback that calculates velocities
void Sensor_System::timer_expire_callback() {

    // Optional To-do: add a low pass filter (queue to get data list, pop oldest and push latest to keep the buffer size small)

    // calc velocities //

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
    // std::cout << angle_diff << std::endl; // debug
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
*/