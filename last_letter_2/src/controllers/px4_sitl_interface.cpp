// A node where the PX4 SITL interface runs
// Based on https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_mavlink_interface.cpp

#include <px4_sitl_interface.hpp>

Controller::Controller()
{
    //Init Subscribers
    // TODO: Establish UDP sockets to SITL
    sub_chan = n.subscribe("last_letter_2/channels", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
    sub_mod_st = n.subscribe("last_letter_2/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

    //Init service
    get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::returnControlInputs, this);

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

    initControllerVariables();
}

// TODO: Delete this 
//Store new joystick values
void Controller::chan_2_signal(last_letter_2_msgs::channels msg)
{
    channels = msg;

    //Keep basic signals
    roll_input = channels.value[roll_in_chan];                 // roll angle signal
    pitch_input = channels.value[pitch_in_chan];               // pitch angle signal
    yaw_input = channels.value[yaw_in_chan];                   // yaw angle signal
    thrust_input = (channels.value[throttle_in_chan] + 1) / 2; // throttle signal
    channelFunctions();
}

// store the model_states published by gazebo
void Controller::store_states(const last_letter_2_msgs::model_states msg)
{
    model_states = msg;
}

// TODO: Insert UDP query to PX4 SITL inputs
// Entry point for the model_node controls query
// calculate and send back to Model class new control model inputs
bool Controller::return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                                     last_letter_2_msgs::get_control_inputs_srv::Response &res)
{
    previous_states_seq = model_states.seq();
    // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
    if (!(previous_imu_seq_ % update_skip_factor_ == 0)) {
        return;
    }

    ros::Time current_time = ros::Time::now();

    close_conn_ = false;
    poll_for_mavlink_messages();


    //check for model_states update. If previous model_states, spin once to call storeState clb for new onces and then continue
    if (req.header.seq != model_states.header.seq)
        ros::spinOnce();


    //Convert PD outputs to motor inputs using quadcopter matrix
    commands(0) = new_thrust_input; //thrust
    commands(1) = new_roll_input;   //roll
    commands(2) = new_pitch_input;  //pitch
    commands(3) = new_yaw_input;    //yaw
    input_signal_vector = multirotor_matrix_inverse * commands;
    for (i = 0; i < num_motors; i++) // store calculated motor inputs
    {
        res.channels[i] = std::max(std::min((double)input_signal_vector[i], 1.0), 0.0); // keep motor singals in range [0, 1]
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
    int button_num;
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
    // Specify the outbound MAVLink address
    mavlink_addr_ = htonl(INADDR_ANY);

    if (use_tcp_) {
        
        // Configure outbound port
        local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
        local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);
        // Remote address not specified (will be established on connection?)

        // Open the socket
        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            ROS_ERROR("Creating TCP socket failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Set socket option, IPPROTO_TCP/TCP_NODELAY
        int yes = 1;
        int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Set socket option SOL_SOCKET/SO_LINGER
        struct linger nolinger {};
        nolinger.l_onoff = 1;
        nolinger.l_linger = 0;
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
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Set socket opiton SOL_SOCKET/SO_REUSEPORT
        // Same as above but for a given port
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
        if (result != 0) {
            ROS_ERROR("setsockopt failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // set socket to non-blocking
        result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
        if (result == -1) {
            ROS_ERROR("setting socket to non-blocking failed: %s, aborting\n", strerror(errno));
            abort();
        }

        // Bind the socket to the specified address
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

        // Configure local port
        local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        local_simulator_addr_.sin_port = htons(0);

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
        ROS_ERROR("Unknown protocol version! using v%i by default\n", protocol_version_);
    }
 
}

// Check for inbound MAVLink control messages
void Controller::poll_for_mavlink_messages()
{
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
            // If it has no return events, continue
            if (fds_[i].revents == 0) {
                continue;
            }

            // If only read events are present, continue
            if (!(fds_[i].revents & POLLIN)) {
                continue;
            }

            if (i == LISTEN_FD) { // if event is raised on the listening socket
                accept_connections();
            }
            else { // if event is raised on connection socket
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
                int len = ret;
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

// Establish TCP connection
void Controller::accept_connections()
{
    if (fds_[CONNECTION_FD].fd > 0) {
        return;
    }

    // accepting incoming connections on listen fd
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
    switch (msg->msgid) {
    // Parse only actuator messages. None other is expected anyways.
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            mavlink_hil_actuator_controls_t controls;
            mavlink_msg_hil_actuator_controls_decode(msg, &controls);

            bool armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

            // Store last-received control message time
            last_actuator_time_ = ros::Time::now();

            // Fill in a linearly-increasing input_index array
            for (unsigned int i = 0; i < n_out_max; i++) {
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
    }
    
    result = ros::xmlrpc::responseInt(1, "", 0);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    // Disable default SIGINT handler
    ros::init(argc, argv, "controller_node", ros::init_options::NoSigintHandler);

    //create the controller
    Controller controller;

    // Bind the custom SIGINT handler
    signal(SIGINT, controller.custom_sigint_handler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", controller.shutdown_callback);

    //Build a thread to spin for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}