// A node where the PX4 SITL interface runs
// Based on https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_mavlink_interface.cpp

#include <last_letter_2_libs/geo_mag_declination.hpp>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <random>

#include <controllers/px4_sitl_interface.hpp>

Controller::Controller()
{
    //Init Subscribers
    sub_chan = n.subscribe("last_letter_2/channels", 1, &Controller::chan_2_signal, this, ros::TransportHints().tcpNoDelay());
    sub_mod_st = n.subscribe("last_letter_2/model_states", 1, &Controller::store_states, this, ros::TransportHints().tcpNoDelay());

    //Init service
    get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::return_control_inputs, this);

    // TODO: Parameters are most likely unneeded
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}
    //Read the update rate
    if (!ros::param::getCached("updatePhysics/deltaT", dt)) { ROS_FATAL("Invalid parameters for deltaT in param server!"); ros::shutdown();}

    char paramMsg[50];

    sprintf(paramMsg, "channels/roll_in_chan");
    if (!ros::param::getCached(paramMsg, roll_in_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/pitch_in_chan");
    if (!ros::param::getCached(paramMsg, pitch_in_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/yaw_in_chan");
    if (!ros::param::getCached(paramMsg, yaw_in_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/throttle_in_chan");
    if (!ros::param::getCached(paramMsg, throttle_in_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}

    configure_ports();
    init_controller_variables();
}

//Store new joystick values
void Controller::chan_2_signal(last_letter_2_msgs::channels msg)
{
    channels_ = msg;

    //Keep basic signals
    roll_input = channels_.value[roll_in_chan];                 // roll angle signal
    pitch_input = channels_.value[pitch_in_chan];               // pitch angle signal
    yaw_input = channels_.value[yaw_in_chan];                   // yaw angle signal
    thrust_input = (channels_.value[throttle_in_chan] + 1) / 2; // throttle signal
    channel_functions();
}

// store the model_states_ published by gazebo
void Controller::store_states(const last_letter_2_msgs::model_states msg)
{
    model_states_ = msg;
}

// Entry point for the model_node controls query
// calculate and send back to Model class new control model inputs
bool Controller::return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                                     last_letter_2_msgs::get_control_inputs_srv::Response &res)
{
    ROS_DEBUG("return_control_inputs service called");
    if (!ros::ok())
    {
        got_sig_int_ = true;
    }

    uint32_t previous_states_seq = model_states_.header.seq;
    // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
    if (!(previous_states_seq % update_skip_factor_ == 0)) {
        // ROS_INFO("Skipping current controls, simulation factor is higher.");
        return false;
    }

    if (!received_first_actuator_)
    {
        starting_timestamp_ = model_states_.header.stamp;
    }

    ros::Duration current_time_ = model_states_.header.stamp - starting_timestamp_;

    close_conn_ = false;
    poll_for_mavlink_messages(); // Reads msgs from SITL, updates input_reference_.

    // Send previously generated simulation state
    ROS_DEBUG("Sending HIL_ msgs to SITL, with stamp %g", current_time_.toSec());
    send_sensor_message();
    send_gps_message();
    send_ground_truth();
    send_rc_inputs_message();

    if (close_conn_) { // close connection if required
        close();
    }

    //check for model_states_ update. If previous model_states_, spin once to call storeState clb for new onces and then continue
    if (req.header.seq != model_states_.header.seq)
    {
        ROS_WARN("model_states msg is stale, waiting for the next one");
        ros::spinOnce();
    }

    // Build actuator controls
    if (received_first_actuator_) {
        for (i = 0; i < num_motors; i++) // store calculated motor inputs
        {
            res.channels[i] = std::max(std::min((double)input_reference_[i], 1.0), -1.0); // keep motor singals in range [-1, 1]
        }
    }
    else
    {
        for (i = 0; i < num_motors; i++) // store calculated motor inputs
        {
            res.channels[i] = 0.0;
        }
    }


    return true;
}

//initialize variables used in control
void Controller::init_controller_variables()
{
    // No operations needed
}


// TODO: could these be used for pausing the simulation?
// Method to use channel signals for extra functions
void Controller::channel_functions()
{
    // int button_num;
    // button_num = 3;
    // if (channels.value[5 + button_num] == 1)
    // {
    //     std::cout << "function: button No" << button_num << std::endl;
    // }
    // button_num = 4;
    // if (channels.value[5 + button_num] == 1)
    // {
    //     std::cout << "function: button No" << button_num << std::endl;
    // }
}

void Controller::configure_ports()
{
    ROS_INFO("Configuring PX4_SITL network interface...");
    // Specify the outbound MAVLink address
    mavlink_addr_ = htonl(INADDR_ANY);

    memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
    remote_simulator_addr_.sin_family = AF_INET;
    remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

    memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
    local_simulator_addr_.sin_family = AF_INET;
    local_simulator_addr_len_ = sizeof(local_simulator_addr_);

    if (use_tcp_) {
        
        // Configure outbound port
        local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
        local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);
        // Remote address not specified (will be established on connection?)

        // Open the socket
        ROS_INFO("Creating TCP socket...");
        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            ROS_ERROR("Creating TCP socket failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Do not wait for ACK from sent messages, set socket option, IPPROTO_TCP/TCP_NODELAY
        int yes = 1;
        ROS_INFO("Configuring socket as TCP_NODELAY...");
        int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Try to close as fast as possible, set socket option SOL_SOCKET/SO_LINGER
        struct linger nolinger {};
        nolinger.l_onoff = 1;
        nolinger.l_linger = 0;
        ROS_INFO("Configuring socket as NO_LINGER");
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // The socket reuse is necessary for reconnecting to the same address
        // if the socket does not close but gets stuck in TIME_WAIT. This can happen
        // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
        // Set socket option SOL_SOCKET/SO_RESUSEADDR
        int socket_reuse = 1;
        ROS_INFO("Configuring socket for address reuse...");
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Set socket opiton SOL_SOCKET/SO_REUSEPORT
        // Same as above but for a given port
        ROS_INFO("Configuring socket for port reuse...");
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Set socket to non-blocking
        // Not used by FlightGear bridge
        ROS_INFO("Setting socket as non-blocking...");
        result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
        if (result == -1) {
            ROS_ERROR("setting socket to non-blocking failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Bind the socket to the specified address
        ROS_INFO("Binding TCP socket to %x:", local_simulator_addr_.sin_addr.s_addr);
        // std::cout << inet_ntoa(local_simulator_addr_.sin_addr) << ":" << std::endl;
        if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
            ROS_ERROR("bind failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Start listening to the simulator socket
        errno = 0;
        if (listen(simulator_socket_fd_, 0) < 0) {
            ROS_ERROR("listen failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Initialize packet memory
        memset(fds_, 0, sizeof(fds_));
        // Connect fds to the socket
        fds_[LISTEN_FD].fd = simulator_socket_fd_;
        fds_[LISTEN_FD].events = POLLIN; // only listens for new connections on tcp

    }
    else {
        // Configure remote port 
        remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
        remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);
        remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

        // Configure local port
        local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        local_simulator_addr_.sin_port = htons(0);
        local_simulator_addr_len_ = sizeof(local_simulator_addr_);

        // Create the socket
        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            ROS_ERROR("Creating UDP socket failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // set socket to non-blocking
        int result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
        if (result == -1) {
            ROS_ERROR("setting socket to non-blocking failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Bind socket
        if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
            ROS_ERROR("bind failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Initialize packet memory
        memset(fds_, 0, sizeof(fds_));
        // Connect fds to the socket
        fds_[CONNECTION_FD].fd = simulator_socket_fd_;
        fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
    }
  
    mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

    // set the Mavlink protocol version to use on the link
    if (protocol_version_ == 2.0) {
        chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
        ROS_INFO("Using MAVLink protocol v2.0\n");
    }
    else if (protocol_version_ == 1.0) {
        chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        ROS_INFO("Using MAVLink protocol v1.0\n");
    }
    else {
        ROS_ERROR("Unknown protocol version! using v%g by default\n", protocol_version_);
    }
 
}

void Controller::close()
{
    got_sig_int_ = true;
    ::close(fds_[CONNECTION_FD].fd);
    fds_[CONNECTION_FD] = { 0, 0, 0 };
    fds_[CONNECTION_FD].fd = -1;

    received_first_actuator_ = false;
}

// Check for inbound MAVLink control messages
void Controller::poll_for_mavlink_messages()
{
    ROS_DEBUG("Polling for MAVLink msgs");
    // Do nothing if node is about to shut down
    if (got_sig_int_) {
        return;
    }

    bool received_actuator = false;

    // Continuously poll for a new MAVLink message
    do {
        // If at least one message has been received and lockstep is enabled, poll for 1000ms.
        // Otherwise, return immediately.
        int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;
        // Wait for fds to becomre ready to perform I/O
        int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

        if (ret < 0) {
            ROS_ERROR("poll error: %s", strerror(errno));
            return;
        }

        if (ret == 0 && timeout_ms > 0) {
            ROS_ERROR("poll timeout");
            return;
        }

        // For each fds allocated
        for (int i = 0; i < N_FDS; i++) {
            ROS_DEBUG("Polled successfully");
            // If it has no return events, continue
            if (fds_[i].revents == 0) {
                ROS_INFO_THROTTLE(1.0, "Didn't get any events");
                continue;
            }

            // If only read events are present, continue
            if (!(fds_[i].revents & POLLIN)) {
                ROS_INFO_THROTTLE(1.0, "Didn't get any POLLIN events");
                continue;
            }

            if (i == LISTEN_FD) { // if event is raised on the listening socket
                ROS_DEBUG("Got poll hit on LISTEN_FD socket");
                accept_connections();
            }
            else { // if event is raised on connection socket
                ROS_DEBUG("Got poll hit on CONNECTION_FD socket");
                int ret = recvfrom(fds_[i].fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
                if (ret < 0) {
                    // all data is read if EWOULDBLOCK is raised
                    if (errno != EWOULDBLOCK) { // disconnected from client
                        ROS_ERROR("recvfrom error: %s", strerror(errno));
                    }
                    continue;
                }

                // client closed the connection orderly, only makes sense on tcp
                if (use_tcp_ && ret == 0) {
                    ROS_ERROR("Connection closed by client.");
                    close_conn_ = true;
                    continue;
                }

                // data received
                ROS_DEBUG("Data received from SITL");
                int len = ret;
                ROS_DEBUG("%d bytes total", len);
                mavlink_message_t msg;
                mavlink_status_t status;
                for (unsigned i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status)) {
                        handle_message(&msg, received_actuator);
                    }
                }
            }
        }
    } while (!close_conn_ // Until connection is closed.
             && received_first_actuator_ // Only if we have received the first actuator message
             && !received_actuator // Until an actuator message is received
             && enable_lockstep_ // Only if lockstep is enabled
            //  && IsRunning() // Only if the simulation is running, TODO: implement
             && !got_sig_int_ // Until a SIGINT is received
             );
}

void Controller::send_mavlink_message(const mavlink_message_t *message)
{
    assert(message != nullptr);

    if (got_sig_int_ || close_conn_) {
        return;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    if (fds_[CONNECTION_FD].fd > 0) {
        int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;
        int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

        if (ret < 0) {
            ROS_ERROR("poll error: %s", strerror(errno));
            return;
        }

        if (ret == 0 && timeout_ms > 0) {
            ROS_ERROR("poll timeout");
            return;
        }

        if (!(fds_[CONNECTION_FD].revents & POLLOUT)) {
            ROS_ERROR("invalid events at fd: %i", fds_[CONNECTION_FD].revents);
            return;
        }

        size_t len;
        if (use_tcp_) {
            ROS_DEBUG("Sending MAVLink message through TCP, with size of %d.", packetlen);
            len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
        } else {
            len = sendto(fds_[CONNECTION_FD].fd, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
        }
        if (len != packetlen) {
            ROS_ERROR("Failed sending mavlink message: %s", strerror(errno));
            if (errno == ECONNRESET || errno == EPIPE) {
                if (use_tcp_) { // udp socket remains alive
                    ROS_ERROR("Closing connection.");
                    close_conn_ = true;
                }
            }
        }
    }
}

double Controller::generate_noise(double std_dev)
{
    return std_dev*noise_distribution_(rn_generator_);
}

void Controller::send_sensor_message()
{
    // Construct the orientation quaternion
    // Eigen::Quaterniond q_gr;
    // q_gr = Eigen::AngleAxisd(model_states_.base_link_states.phi, Eigen::Vector3d::UnitX())
    //        * Eigen::AngleAxisd(model_states_.base_link_states.theta, Eigen::Vector3d::UnitY())
    //        * Eigen::AngleAxisd(model_states_.base_link_states.psi, Eigen::Vector3d::UnitZ());
 
    // Create and stamp the sensor MAVLink message
    mavlink_hil_sensor_t sensor_msg;
    msg_counter_ += deltat_us_;
    // sensor_msg.time_usec = current_time_.toSec() * 1e6;
    sensor_msg.time_usec = msg_counter_;

    // Insert acceleration information. Gravity component needs to be removed, i.e. real-world accelerometer reading is expected.
    // Needs conversion from FLU to Body-frame
    sensor_msg.xacc = model_states_.base_link_states.acc_x + generate_noise(0.001);
    sensor_msg.yacc = -model_states_.base_link_states.acc_y + generate_noise(0.001);
    sensor_msg.zacc = -model_states_.base_link_states.acc_z + generate_noise(0.001);

    // Insert angular velocity information
    // Needs conversion from FLU to Body-frame
    sensor_msg.xgyro = model_states_.base_link_states.p + generate_noise(0.001);
    sensor_msg.ygyro = -model_states_.base_link_states.q + generate_noise(0.001);
    sensor_msg.zgyro = -model_states_.base_link_states.r + generate_noise(0.001);

    // Insert magnetometer information
    // Magnetic field data from WMM2018 (10^5xnanoTesla (N, E D) n-frame), using the geo_mag_declination library

    // Magnetic declination and inclination (radians)
    float declination_rad = get_mag_declination(home_lat_deg_, home_lon_deg_) * M_PI / 180;
    float inclination_rad = get_mag_inclination(home_lat_deg_, home_lon_deg_) * M_PI / 180;

    // Magnetic strength (10^5xnanoTesla)
    float strength_ga = 0.01f * get_mag_strength(home_lat_deg_, home_lon_deg_);

    // Magnetic filed components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
    float H = strength_ga * cosf(inclination_rad);
    float mag_Z = tanf(inclination_rad) * H;
    float mag_X = H * cosf(declination_rad);
    float mag_Y = H * sinf(declination_rad);

    sensor_msg.xmag = mag_X + generate_noise(0.002);
    sensor_msg.ymag = mag_Y + generate_noise(0.002);
    sensor_msg.zmag = mag_Z + generate_noise(0.002);

    // Insert barometer information
    float pose_n_z = -model_states_.base_link_states.z; // convert Z-component from ENU to NED

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    float alt_msl = -pose_n_z; // TODO: initialize with correct home altitude
    float temperature_local = temperature_msl - lapse_rate * alt_msl;
    float pressure_ratio = powf((temperature_msl/temperature_local), 5.256f);
    const float pressure_msl = 101325.0f; // pressure at MSL
    float absolute_pressure = pressure_msl / pressure_ratio;

    // convert to hPa
    absolute_pressure *= 0.01f;

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf((temperature_msl/temperature_local) , 4.256f);
    float rho = 1.225f / density_ratio;

    // calculate pressure altitude
    float pressure_altitude = alt_msl;

    // calculate temperature in Celsius
    sensor_msg.temperature = temperature_local - 273.0 + generate_noise(0.1);

    sensor_msg.abs_pressure = absolute_pressure + generate_noise(0.001);
    sensor_msg.pressure_alt = pressure_altitude + generate_noise(0.002);

    // Insert differential pressure information
    sensor_msg.diff_pressure = 0.005f * rho * powf(model_states_.base_link_states.u + generate_noise(0.005), 2.0);

    // Tick which fields have been filled
    sensor_msg.fields_updated = SensorSource::ACCEL
                                | SensorSource::GYRO
                                | SensorSource::MAG
                                | SensorSource::BARO
                                | SensorSource::DIFF_PRESS;




    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    send_mavlink_message(&msg);
}

void Controller::send_gps_message() {

    // Construct the orientation quaternion
    Eigen::Quaterniond q_gr;
    q_gr = Eigen::AngleAxisd(model_states_.base_link_states.phi, Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(model_states_.base_link_states.theta, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(model_states_.base_link_states.psi, Eigen::Vector3d::UnitZ());

    // fill HIL GPS Mavlink msg
    mavlink_hil_gps_t hil_gps_msg;
    // hil_gps_msg.time_usec = current_time_.toSec() * 1e6;
    hil_gps_msg.time_usec = msg_counter_;

    hil_gps_msg.fix_type = 3;
    // Insert coordinate information
    // Small-angle approximation of the coordinates around the home location
    double x_pos = model_states_.base_link_states.x + generate_noise(0.1);
    double y_pos = model_states_.base_link_states.y + generate_noise(0.1);
    double z_pos = model_states_.base_link_states.z + generate_noise(0.2);
    hil_gps_msg.lat = (home_lat_deg_ + x_pos * 180/M_PI/6378137.0)*1e7;
    hil_gps_msg.lon = (home_lon_deg_ + -y_pos * 180/M_PI/6378137.0)*1e7;
    hil_gps_msg.alt = z_pos * 1000;
    hil_gps_msg.eph = 100.0;
    hil_gps_msg.epv = 100.0;

    // Insert inertial velocity information
    Eigen::Vector3d v_B = Eigen::Vector3d(
        model_states_.base_link_states.u + generate_noise(0.01),
        model_states_.base_link_states.v + generate_noise(0.01),
        model_states_.base_link_states.w + generate_noise(0.01)
    );
    Eigen::Vector3d v_I = q_gr.conjugate()*v_B;
    hil_gps_msg.vn = v_I.x() * 100;
    hil_gps_msg.ve = v_I.y() * 100;
    hil_gps_msg.vd = -v_I.z() * 100;
    hil_gps_msg.vel = v_I.norm() * 100.0;

    double cog{atan2(v_I.y(), v_I.x())};
    hil_gps_msg.cog = static_cast<uint16_t>(wrap_to_360(cog*180/M_PI) * 100.0);
    hil_gps_msg.satellites_visible = 10;

    // send HIL_GPS Mavlink msg
    mavlink_message_t msg;
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    send_mavlink_message(&msg);
}

void Controller::send_ground_truth()
{
    // Construct the orientation quaternion
    Eigen::Quaterniond q_gr;
    q_gr = Eigen::AngleAxisd(model_states_.base_link_states.phi, Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(model_states_.base_link_states.theta, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(model_states_.base_link_states.psi, Eigen::Vector3d::UnitZ());

    mavlink_hil_state_quaternion_t hil_state_quat;

    // Insert time information.
    // hil_state_quat.time_usec = current_time_.toSec() * 1e6;
    hil_state_quat.time_usec = msg_counter_;

    // Insert attitude information.
    hil_state_quat.attitude_quaternion[0] = q_gr.w();
    hil_state_quat.attitude_quaternion[1] = q_gr.x();
    hil_state_quat.attitude_quaternion[2] = q_gr.y();
    hil_state_quat.attitude_quaternion[3] = q_gr.z();

    // Insert angular velocity information.
    // Needs conversion from FLU to Body-frame
    hil_state_quat.rollspeed = model_states_.base_link_states.p;
    hil_state_quat.pitchspeed = -model_states_.base_link_states.q;
    hil_state_quat.yawspeed = -model_states_.base_link_states.r;

    // Insert coordinate information
    // Small-angle approximation of the coordinates around the home location
    hil_state_quat.lat = home_lat_deg_ + (model_states_.base_link_states.x*180/M_PI*1e7/6378137.0);
    hil_state_quat.lon = home_lon_deg_ + (-model_states_.base_link_states.y*180/M_PI*1e7/6378137.0);
    hil_state_quat.alt = model_states_.base_link_states.z * 1000;

    // Insert inertial velocity information
    Eigen::Vector3d v_B = Eigen::Vector3d(
        model_states_.base_link_states.u,
        model_states_.base_link_states.v,
        model_states_.base_link_states.w
    );
    Eigen::Vector3d v_I = q_gr.conjugate()*v_B;
    hil_state_quat.vx = v_I.x() * 100;
    hil_state_quat.vy = v_I.y() * 100;
    hil_state_quat.vz = v_I.z() * 100;

    // Insert airspeed information
    // assumed indicated airspeed due to flow aligned with pitot (body x)
    hil_state_quat.ind_airspeed = v_B.x() * 100;
    hil_state_quat.true_airspeed = hil_state_quat.ind_airspeed;

    // Insert acceleration information
    hil_state_quat.xacc = model_states_.base_link_states.acc_x * 1000 / 9.81;
    hil_state_quat.yacc = -model_states_.base_link_states.acc_y * 1000 / 9.81;
    hil_state_quat.zacc = -model_states_.base_link_states.acc_z * 1000 / 9.81;

    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    send_mavlink_message(&msg);
}

void Controller::send_rc_inputs_message()
{
    mavlink_hil_rc_inputs_raw_t hil_rc_inputs_raw_msg;
    // hil_rc_inputs_raw_msg.time_usec = current_time_.toSec() * 1e6;
    hil_rc_inputs_raw_msg.time_usec = msg_counter_;
    hil_rc_inputs_raw_msg.chan1_raw = channels_.value[0]*500 + 1500;
    hil_rc_inputs_raw_msg.chan2_raw = channels_.value[1]*500 + 1500;
    hil_rc_inputs_raw_msg.chan3_raw = channels_.value[2]*1000 + 1000;
    hil_rc_inputs_raw_msg.chan4_raw = channels_.value[3]*500 + 1500;
    hil_rc_inputs_raw_msg.rssi = 254;
    mavlink_message_t msg;
    mavlink_msg_hil_rc_inputs_raw_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_rc_inputs_raw_msg);
    send_mavlink_message(&msg);
}

// Establish TCP connection
void Controller::accept_connections()
{
    if (fds_[CONNECTION_FD].fd > 0) {
        return;
    }

    // accepting incoming connections on listen fd
    ROS_INFO("Establishing the remote (SITL) socket");
    int ret = accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

    if (ret < 0) {
        if (errno != EWOULDBLOCK) {
            ROS_ERROR("accept error: %s", strerror(errno));
        }
        return;
    }

    // assign socket to connection descriptor on success
    fds_[CONNECTION_FD].fd = ret; // socket is replaced with latest connection
    fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
}

// Decoder for incoming MAVLink messages
void Controller::handle_message(mavlink_message_t *msg, bool &received_actuator)
{
    ROS_DEBUG("Handling MAVLink message with ID:%d", msg->msgid);
    switch (msg->msgid) {
    // Parse only actuator messages. None other is expected anyways.
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            ROS_DEBUG("Decoding new HIL_ACTUATOR_CONTROLS msg");
            mavlink_hil_actuator_controls_t controls;
            mavlink_msg_hil_actuator_controls_decode(msg, &controls);

            bool armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

            // Store last-received control message time
            last_actuator_time_ = current_time_;

            // Fill in a linearly-increasing input_index array
            for (unsigned int i = 0; i < n_out_max_; i++) {
                input_index_[i] = i;
            }

            // set rotor speeds, controller targets
            input_reference_.resize(n_out_max_);
            for (int i = 0; i < input_reference_.size(); i++) {
                if (armed) {
                    input_reference_[i] = controls.controls[input_index_[i]];
                }
                else {
                    input_reference_[i] = 0;
                }
            }

            received_actuator = true;
            received_first_actuator_ = true;
            break;
    }
}

// Handler of SIGINT for nodes raised by roslaunch
void Controller::custom_sigint_handler(int sig)
{
    got_sig_int_ = true;
    // TODO: Is it ok calling shutting down here? Will the rest of the calls/functions terminate based on got_sig_int before shutdown takes effect?
    ros::shutdown();
}

// Handler of SIGINT stemming from rosnode kill
void Controller::shutdown_callback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        got_sig_int_ = true; // Set flag
        ros::shutdown();
    }
    
    result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
    // Disable default SIGINT handler
    ros::init(argc, argv, "controller_node", ros::init_options::NoSigintHandler);

    //create the controller
    Controller controller;

    // // Bind the custom SIGINT handler
    // signal(SIGINT, controller.custom_sigint_handler);

    // Override XMLRPC shutdown
    //TODO cannot get this to work, because bind() needs a static function, but we need to set the property got_sig_int_
    // ros::XMLRPCManager::instance()->unbind("shutdown");
    // ros::XMLRPCManager::instance()->bind("shutdown", controller.shutdown_callback);

    ROS_INFO("PX4-SITL interface spawned");

    //Build a thread to spin for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();

    // ros::WallRate temp_spinner(1);
    // while (ros::ok())
    // {
    //     temp_spinner.sleep();
    //     controller.poll_for_mavlink_messages();
    //     ROS_INFO("Sending msgs to SITL");
    //     controller.send_sensor_message();
    //     controller.send_gps_message();
    //     controller.send_ground_truth();
    //     controller.send_rc_inputs_message();
    // } 

    controller.close();
    return 0;
}