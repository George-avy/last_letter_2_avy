// A node where the controller for quadcopter model runs

#include <ros/ros.h>
#include <last_letter_2_msgs/channels.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>
#include "last_letter_2_libs/math_lib.hpp"
#include <dynamic_reconfigure/server.h>
#include <last_letter_2/PD_gainsConfig.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <Eigen/Dense>

// PD gains declaration
std_msgs::Float64 roll_kp;
std_msgs::Float64 roll_kd;
std_msgs::Float64 pitch_kp;
std_msgs::Float64 pitch_kd;
std_msgs::Float64 yaw_kp;
std_msgs::Float64 yaw_kd;
std_msgs::Float64 alt_kp;
std_msgs::Float64 alt_kd;

class Controller
{
private:
    ros::NodeHandle n;

    //Subscribers
    ros::Subscriber sub_chan;
    ros::Subscriber sub_mod_st;

    //Service
    ros::ServiceServer get_control_inputs_service;

    last_letter_2_msgs::channels channels;
    last_letter_2_msgs::model_states model_states;

    Eigen::MatrixXf multirotor_matrix;
    Eigen::MatrixXf multirotor_matrix_inverse;
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

public:
    Controller();
    void chan2signal(last_letter_2_msgs::channels msg);
    void storeStates(const last_letter_2_msgs::model_states msg);
    bool returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                             last_letter_2_msgs::get_control_inputs_srv::Response &res);
    void initControllerVariables();
    void channelFunctions();
    void controlLaw();
};

Controller::Controller()
{
    //Init Subscribers
    sub_chan = n.subscribe("last_letter_2/channels", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
    sub_mod_st = n.subscribe("last_letter_2/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

    //Init service
    get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::returnControlInputs, this);

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

//Store new joystick values
void Controller::chan2signal(last_letter_2_msgs::channels msg)
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
void Controller::storeStates(const last_letter_2_msgs::model_states msg)
{
    model_states = msg;
}

// calculate and send back to Model class new control model inputs
bool Controller::returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                                     last_letter_2_msgs::get_control_inputs_srv::Response &res)
{
    //check for model_states update. If previous model_states, spin once to call storeState clb for new onces and then continue
    if (req.header.seq != model_states.header.seq)
        ros::spinOnce();

    controlLaw();

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

// dynamic reconfigure callback, for real-time tuning
void GainsCallback(last_letter_2::PD_gainsConfig &config, uint32_t level)
{
    roll_kp.data = config.roll_kp;
    roll_kd.data = config.roll_kd;
    pitch_kp.data = config.pitch_kp;
    pitch_kd.data = config.pitch_kd;
    yaw_kp.data = config.yaw_kp;
    yaw_kd.data = config.yaw_kd;
    alt_kp.data = config.alt_kp;
    alt_kd.data = config.alt_kd;
}

//initialize variables used in control
void Controller::initControllerVariables()
{
    for (int i = 0; i < 20; i++)
    {
        channels.value[i] = 0;
    }
    roll_input = 0;
    pitch_input = 0;
    yaw_input = 0;
    thrust_input = 0;

    altitude = 0;
    yaw_direction = 0;
    prev_roll_error = 0;
    prev_pitch_error = 0;
    prev_yaw_error = 0;
    prev_alt_error = 0;

    // Set the size of matrices and vectors based on model
    multirotor_matrix.resize(4, 4);
    multirotor_matrix_inverse.resize(4, 4);
    commands.resize(4);
    input_signal_vector.resize(4);

    //Built quadcopter matrix
    multirotor_matrix << 0.25, 0.25, 0.25, 0.25,    //thrust row
                            0, -0.5,    0,  0.5,    //roll row
                          0.5,    0, -0.5,    0,    //pitch row
                        -0.25, 0.25,-0.25, 0.25;    //yaw row

    //calculate inverse of quadcopter matrix. Usefull for future calculations
    multirotor_matrix_inverse = multirotor_matrix.completeOrthogonalDecomposition().pseudoInverse();
}

// Control law calculations
void Controller::controlLaw()
{
    // PD controller for roll, pitch, yaw and altitude
    new_roll_input = roll_input;
    new_pitch_input = pitch_input;
    new_yaw_input = yaw_input;
    new_thrust_input = thrust_input;

    float error, d_error;
    float kp, kd;

    //stabilize roll
    kp = roll_kp.data;
    kd = roll_kd.data;
    error = 0.8 * roll_input - model_states.base_link_states.phi;
    d_error = (error - prev_roll_error) / dt;
    prev_roll_error = error; // Keep current data for next step
    new_roll_input = kp * error + kd * d_error;
    new_roll_input = std::max(std::min((double)new_roll_input, 1.0), -1.0); // keep in range [-1, 1]

    //stabilize pitch
    kp = pitch_kp.data;
    kd = pitch_kd.data;
    error = 0.8 * pitch_input + model_states.base_link_states.theta;
    d_error = (error - prev_pitch_error) / dt;
    prev_pitch_error = error; // Keep current data for next step
    new_pitch_input = kp * error + kd * d_error;
    new_pitch_input = std::max(std::min((double)new_pitch_input, 1.0), -1.0); // keep in range [-1, 1]

    //yaw direction control
    kp = yaw_kp.data;
    kd = yaw_kd.data;
    yaw_direction += yaw_input * 0.001;
    if (yaw_direction > 3.13)
        yaw_direction = -3.13;
    else if (yaw_direction < -3.13)
        yaw_direction = 3.13;
    error = yaw_direction + model_states.base_link_states.psi;
    if (error > 3.14)
        error = 0.1;
    else if (error < -3.14)
        error = -0.1;
    d_error = (error - prev_yaw_error) / dt;
    prev_yaw_error = error; // Keep current data for next step
    new_yaw_input = kp * error + kd * d_error;
    new_yaw_input = std::max(std::min((double)new_yaw_input, 1.0), -1.0); // keep in range [-1, 1]

    //altitude control
    kp = alt_kp.data;
    kd = alt_kd.data;
    altitude = 50 * thrust_input; //control altitude from thrust signal
    error = altitude - model_states.base_link_states.z;
    d_error = (error - prev_alt_error) / dt;
    prev_alt_error = error; // Keep current data for next step
    new_thrust_input = kp * error + kd * d_error;
    new_thrust_input = std::max(std::min((double)new_thrust_input, 1.0), 0.0); // keep in range [0, 1]
}

// Method to use channel signals for extra functions
void Controller::channelFunctions()
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");

    //create the controller
    Controller controller;

    // dynamic_reconfigure in the real-time tuning
    dynamic_reconfigure::Server<last_letter_2::PD_gainsConfig> server;
    dynamic_reconfigure::Server<last_letter_2::PD_gainsConfig>::CallbackType f;

    f = boost::bind(&GainsCallback, _1, _2);
    server.setCallback(f);

    //Build a thread to spin for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}