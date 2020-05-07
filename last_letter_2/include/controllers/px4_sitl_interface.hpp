// #include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netinet/tcp.h> // Weird that this is required, not included in original PX4 code
#include <chrono>
#include <mutex>
#include <thread> //TODO: May not be useful, was originally imported because Gazebo plugins run as threads.
#include <atomic>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mavlink/v2.0/common/mavlink.h>
// #include <boost/bind.hpp>

#include <last_letter_2_msgs/channels.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>
#include <last_letter_2_libs/math_lib.hpp>

static const uint32_t kDefaultQGCUdpPort = 14550;
static const uint32_t kDefaultSDKUdpPort = 14540;

//! Enumeration to use on the bitmask in HIL_SENSOR
enum class SensorSource {
  ACCEL      = 0b111,
  GYRO	     = 0b111000,
  MAG		 = 0b111000000,
  BARO		 = 0b1101000000000,
  DIFF_PRESS = 0b10000000000,
};

//! OR operation for the enumeration and unsigned types that returns the bitmask
static inline uint32_t operator |(SensorSource lhs, SensorSource rhs)
{
    return static_cast<uint32_t>
    (
      static_cast<std::underlying_type<SensorSource>::type>(lhs) |
      static_cast<std::underlying_type<SensorSource>::type>(rhs)
    );
}

static inline uint32_t operator |(uint32_t lhs, SensorSource rhs)
{
    return static_cast<uint32_t>
    (
      static_cast<std::underlying_type<SensorSource>::type>(lhs) |
      static_cast<std::underlying_type<SensorSource>::type>(rhs)
    );
}

class Controller
{
private:
    ros::NodeHandle n;

    //Subscribers
    ros::Subscriber sub_chan;
    ros::Subscriber sub_mod_st;

    //Service
    ros::ServiceServer get_control_inputs_service;

    last_letter_2_msgs::channels channels_;
    last_letter_2_msgs::model_states model_states_;

    Eigen::VectorXf commands;
    Eigen::VectorXf input_signal_vector;

    // Essencial variables
    int i;
    int num_wings, num_motors;
    int roll_in_chan, pitch_in_chan, yaw_in_chan, throttle_in_chan;
    float roll_input, pitch_input, yaw_input, thrust_input;
    float new_roll_input, new_pitch_input, new_yaw_input, new_thrust_input;

    // variables for PD controller algorithm
    float prev_roll_error, prev_pitch_error, prev_yaw_error, prev_alt_error;
    float altitude, yaw_direction;
    float dt;

    // port configuration parameters
    unsigned char _buf[65535]; // Buffer for MAVLink message parsing
    enum FD_TYPES {
        LISTEN_FD,
        CONNECTION_FD,
        N_FDS
    };
    struct pollfd fds_[N_FDS]; // The poll file descriptor
    bool use_tcp_{true};
    bool close_conn_{false};
    struct sockaddr_in local_simulator_addr_;
    socklen_t local_simulator_addr_len_;
    struct sockaddr_in remote_simulator_addr_;
    socklen_t remote_simulator_addr_len_;

    in_addr_t mavlink_addr_;
    int mavlink_udp_port_{14560}; // MAVLink refers to the PX4 simulator interface here
    int mavlink_tcp_port_{4560}; // MAVLink refers to the PX4 simulator interface here

    int simulator_socket_fd_;
    int simulator_tcp_client_fd_;

    // MAVLink configuration
    float protocol_version_{2.0};
    static const unsigned int n_out_max_{16};

    ros::Time last_time_;
    ros::Time last_imu_time_;
    ros::Time current_time_;
    ros::Time last_actuator_time_{0};
    Eigen::VectorXd input_reference_;
    int input_index_[n_out_max_];
    bool received_first_actuator_{false};
    bool enable_lockstep_{true};

    int64_t previous_imu_seq_{0};
    unsigned update_skip_factor_{1};
    std::atomic<bool> got_sig_int_ {false};
    // Hardcoded Avy coordinates
    const float home_lat_deg_{52.3936579};
    const float home_lon_deg_{4.8284891};
    Eigen::Vector3d wind_vel_ = Eigen::Vector3d::Zero();


public:
    Controller();
    void chan_2_signal(last_letter_2_msgs::channels msg);
    void store_states(const last_letter_2_msgs::model_states msg);
    bool return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                             last_letter_2_msgs::get_control_inputs_srv::Response &res);
    void init_controller_variables();
    void channel_functions();
    void configure_ports();
    void close();
    void poll_for_mavlink_messages();
    void send_mavlink_message(const mavlink_message_t *message);
    void send_sensor_message();
    void send_gps_message();
    void send_ground_truth();
    void send_rc_inputs_message();
    void accept_connections();
    void handle_message(mavlink_message_t *msg, bool &received_actuator);
    void custom_sigint_handler(int sig);
    void shutdown_callback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
};