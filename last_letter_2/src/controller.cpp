// A node that converts channels signals, to input signals for the model,
// based on the kind of model is loaded (plane, multirotor etc)

#include <ros/ros.h>
#include <last_letter_2_msgs/joystick_input.h>
#include <last_letter_2_msgs/model_inputs.h>
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
    last_letter_2_msgs::model_inputs model_inputs;
    last_letter_2_msgs::model_states model_states;
    int mixerid, i;
    int num_wings, num_motors;
    
    //Create essencial variables, based on parameter server values.
    int roll_angle, pitch_angle, yaw_angle, thrust; // for mixing
    float input_signals[5];

    float delta_a, delta_e, delta_r, delta_t;
    float prev_roll_error, prev_pitch_error, prev_yaw_error, prev_alt_error;
    float altitude, yaw_direction;
    float new_delta_a, new_delta_e, new_delta_r, new_delta_t;
    float buttons[20];
    float dt;

public:
    Controller();
    void chan2signal(last_letter_2_msgs::joystick_input msg);
    void storeStates(const last_letter_2_msgs::model_states msg);
    bool returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res);
    void initVariables();
    void buttonFunctions();

    //control functions are declared here
    void PD();
};

Controller::Controller()
{
    //Init Subscribers
    sub_chan = n.subscribe("last_letter_2/rawPWM", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
    sub_mod_st = n.subscribe("last_letter_2/gazebo/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

    //Init service
    get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::returnControlInputs, this);

    // Read the mixer type
    if (!ros::param::getCached("HID/mixerid", mixerid)) { ROS_INFO("No mixing function selected"); mixerid = 0;}
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}
    //Read the update rate
    if (!ros::param::getCached("world/deltaT", dt)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}

    char paramMsg[50];

    sprintf(paramMsg, "roll_angle");
    if (!ros::param::getCached(paramMsg, roll_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "pitch_angle");
    if (!ros::param::getCached(paramMsg, pitch_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "yaw_angle");
    if (!ros::param::getCached(paramMsg, yaw_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "thrust");
    if (!ros::param::getCached(paramMsg, thrust)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    initVariables();
}

//Store new joystick values
void Controller::chan2signal(last_letter_2_msgs::joystick_input msg)
{
    channels = msg;

    //Channel mixing
    delta_a = (float)(channels.value[roll_angle] - 1500) / 500;  // roll angle signal
    delta_e = (float)(channels.value[pitch_angle] - 1500) / 500; // pitch angle signal
    delta_r = (float)(channels.value[yaw_angle] - 1500) / 500;   // yaw angle signal
    delta_t = (float)(channels.value[thrust] - 1000) / 1000;     // thrust signal
    for (i = 4; i < 20; i++)                                     //store the rest button singals
        buttons[i] = (channels.value[i] - 1500) / 500;
    FRDtoFLU(delta_a, delta_e, delta_r); //convert channels from FRD to FLU frame, that states from gazebo are expressed
                                         //now all data are expressed in FLU frame
    model_inputs.header.stamp = ros::Time::now();
    buttonFunctions();
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

    new_delta_a = delta_a;
    new_delta_e = delta_e;
    new_delta_r = delta_r;
    new_delta_t = delta_t;

switch (mixerid)
    {
    case 0: // No mixing applied
        break;
    case 1: // Airplane 
    //choose plane controllers

        break;
    case 2:
    //choose multirotor controllers
        PD();
        break;
    default:
        ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
        ros::shutdown();
        break;
    }

    FLUtoFRD(new_delta_a, new_delta_e, new_delta_r); //convert signals back to FRD frame, the default model frame
    res.input_signals[0] = 0;
    res.input_signals[1] = new_delta_a; // roll signal
    res.input_signals[2] = new_delta_e; // pitch signal
    res.input_signals[3] = new_delta_r; // yaw signal
    res.input_signals[4] = new_delta_t; // thrust signal
    for (i = 4; i < 20; i++)
    {
        res.button_signals[i] = buttons[i]; //button signals
    }
    return true;
}

//Controller functions

//initialize variables used in control
void Controller::initVariables()
{
    delta_a = 0;
    delta_e = 0;
    delta_r = 0;
    delta_t = 0;
    altitude = 0;
    yaw_direction = 0;
    prev_roll_error = 0;
    prev_pitch_error = 0;
    prev_yaw_error = 0;
    prev_alt_error = 0;
}

//PD controller
void Controller::PD()
{
    float error, d_error;
    float kp, kd;

    //stabilize roll
    kp = 3;
    kd = 0.5;
    error = delta_a - model_states.base_link_states.roll;
    d_error = (error - prev_roll_error) / dt;
    prev_roll_error = error; // Keep current data for next step
    new_delta_a = kp * error + kd * d_error;
    if (new_delta_a < -1)
        new_delta_a = -1;
    if (new_delta_a > 1)
        new_delta_a = 1;

    //stabilize pitch
    kp = 3;
    kd = 0.5;
    error = delta_e - model_states.base_link_states.pitch;
    d_error = (error - prev_pitch_error) / dt;
    prev_pitch_error = error; // Keep current data for next step
    new_delta_e = kp * error + kd * d_error;
    if (new_delta_e < -1)
        new_delta_e = -1;
    if (new_delta_e > 1)
        new_delta_e = 1;

    //yaw direction control
    kp = 1;
    kd = 1;
    yaw_direction += delta_r * 0.01;
    if (yaw_direction > 3.13)
        yaw_direction = -3.13;
    else if (yaw_direction < -3.13)
        yaw_direction = 3.13;
    error = yaw_direction - model_states.base_link_states.yaw;
    if (error > 3.14)
        error = 0.1;
    else if (error < -3.14)
        error = -0.1;
    d_error = (error - prev_yaw_error) / dt;
    prev_yaw_error = error; // Keep current data for next step
    new_delta_r = kp * error + kd * d_error;
    if (new_delta_r < -1)
        new_delta_r = -1;
    if (new_delta_r > 1)
        new_delta_r = 1;

    //altitude control
    kp = 1;
    kd = 0.8;
    altitude = 50 * delta_t; //control altitude from thrust signal
    error = altitude - model_states.base_link_states.z;
    d_error = (error - prev_alt_error) / dt;
    prev_alt_error = error; // Keep current data for next step
    new_delta_t = kp * error + kd * d_error;
    if (new_delta_t < 0)
        new_delta_t = 0;
    if (new_delta_t > 1)
        new_delta_t = 1;
}

// do the button functions
void Controller::buttonFunctions()
{
    int button_num;
    button_num=3;
    if (buttons[5 + button_num] == 1)
    {
        std::cout << "function: button No" <<button_num << std::endl;
    }
    button_num=4;
    if (buttons[5 + button_num] == 1)
    {
        std::cout << "function: button No" <<button_num << std::endl;
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