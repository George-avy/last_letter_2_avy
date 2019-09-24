// A node that runs the controller for plane models

#include <ros/ros.h>
#include <last_letter_2_msgs/joystick_input.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>
#include "last_letter_2_libs/math_lib.hpp"

class Controller
{
private:
    ros::NodeHandle n;

    //Subscribers
    ros::Subscriber sub_chan;
    ros::Subscriber sub_mod_st;

    //Service
    ros::ServiceServer get_control_inputs_service;

    last_letter_2_msgs::joystick_input channels;
    last_letter_2_msgs::model_states model_states;

    //Create essencial variables, based on parameter server values.
    int i;
    int num_wings, num_motors;
    int phi_chan, theta_chan, psi_chan, throttle_chan; 
    float roll_input, pitch_input, yaw_input, thrust_input;
    float new_roll_input, new_pitch_input, new_yaw_input, new_thrust_input;
    int input_x_chan[4], input_y_chan[4], input_z_chan[4];
    float deltax_max[4], deltay_max[4], deltaz_max[4];

public:
    Controller();
    void chan2signal(last_letter_2_msgs::joystick_input msg);
    void storeStates(const last_letter_2_msgs::model_states msg);
    bool returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res);
    void initControllerVariables();
    void channelFunctions();
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

    char paramMsg[50];

    sprintf(paramMsg, "channels/phi_chan");
    if (!ros::param::getCached(paramMsg, phi_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/theta_chan");
    if (!ros::param::getCached(paramMsg, theta_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/psi_chan");
    if (!ros::param::getCached(paramMsg, psi_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/throttle_chan");
    if (!ros::param::getCached(paramMsg, throttle_chan)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}

    //Load basic characteristics for each airfoil
    for (i = 0; i < num_wings; ++i)
    {
        sprintf(paramMsg, "airfoil%i/input_x_chan", i + 1);
        if (!ros::param::getCached(paramMsg, input_x_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/input_y_chan", i + 1);
        if (!ros::param::getCached(paramMsg, input_y_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/input_z_chan", i + 1);
        if (!ros::param::getCached(paramMsg, input_z_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/deltax_max", i + 1);
        if (!ros::param::getCached(paramMsg, deltax_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/deltay_max", i + 1);
        if (!ros::param::getCached(paramMsg, deltay_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/deltaz_max", i + 1);
        if (!ros::param::getCached(paramMsg, deltaz_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    }
    initControllerVariables();
}

//Store new joystick values
void Controller::chan2signal(last_letter_2_msgs::joystick_input msg)
{
    channels = msg;

    //Keep basic signals
    roll_input = channels.value[phi_chan];         // roll angle signal
    pitch_input = channels.value[theta_chan];       // pitch angle signal
    yaw_input = channels.value[psi_chan];           // yaw angle signal
    thrust_input = (channels.value[throttle_chan] + 1) / 2; // throttle signal
    channelFunctions();
}

// store the model_states published by gazebo
void Controller::storeStates(const last_letter_2_msgs::model_states msg)
{
    model_states = msg;
}

bool Controller::returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                                       last_letter_2_msgs::get_control_inputs_srv::Response &res)
{
    //check for model_states update. If previous model_states, call storeState clb for new onces and then continue
    if (req.header.seq != model_states.header.seq)
        ros::spinOnce();

    new_roll_input = roll_input;
    new_pitch_input = pitch_input;
    new_yaw_input = yaw_input;
    new_thrust_input = thrust_input;

    res.channels[0] = new_roll_input;
    res.channels[1] = new_pitch_input;
    res.channels[2] = new_yaw_input;
    res.channels[3] = new_thrust_input;
    return true;
}

//Controller functions

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
}

// do the button functions
void Controller::channelFunctions()
{
    int button_num;
    button_num = 3;
    if (channels.value[5 + button_num] == 1)
    {
        std::cout << "function: button No" << button_num << std::endl;
    }
    button_num = 4;
    if (channels.value[5 + button_num] == 1)
    {
        std::cout << "function: button No" << button_num << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");

    //create the controller
    Controller controller;

    //Build a thread to spin for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}