// A node that export model input signals from channels signals,
// based on model type (plane, multirotor etc)

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
    int handling, model_type, i;
    int num_wings, num_motors;
    
    //Create essencial variables, based on parameter server values.
    int roll_angle, pitch_angle, yaw_angle, thrust; // for mixing
    float input_signals[5];

    float roll_input, pitch_input, yaw_input, thrust_input;
    float prev_roll_error, prev_pitch_error, prev_yaw_error, prev_alt_error;
    float altitude, yaw_direction;
    float new_roll_input, new_pitch_input, new_yaw_input, new_thrust_input;
    float dt;

public:
    Controller();
    void chan2signal(last_letter_2_msgs::joystick_input msg);
    void storeStates(const last_letter_2_msgs::model_states msg);
    bool returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res);
    void initControllerVariables();
    void channelFunctions();

    //control functions are declared here
    void PD();
};

Controller::Controller()
{
    //Init Subscribers
    sub_chan = n.subscribe("last_letter_2/channels", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
    sub_mod_st = n.subscribe("last_letter_2/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

    //Init service
    get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::returnControlInputs, this);

    // Read the type of model
    if (!ros::param::getCached("model/type", model_type)) { ROS_INFO("No model type selected"); model_type = 0;}
    // Read the type of handling 
    if (!ros::param::getCached("model/handling", handling)) { ROS_INFO("No mixing function selected"); handling = 0;}
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}
    //Read the update rate
    if (!ros::param::getCached("updatePhysics/deltaT", dt)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}

    char paramMsg[50];

    sprintf(paramMsg, "channels/roll_angle");
    if (!ros::param::getCached(paramMsg, roll_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/pitch_angle");
    if (!ros::param::getCached(paramMsg, pitch_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/yaw_angle");
    if (!ros::param::getCached(paramMsg, yaw_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/thrust");
    if (!ros::param::getCached(paramMsg, thrust)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}

    for (int i = 0; i < 20; i++)
    {
        channels.value[i] = 0;
    }
    initControllerVariables();
}

//Store new joystick values
void Controller::chan2signal(last_letter_2_msgs::joystick_input msg)
{
    channels = msg;

    //Keep basic signals
    roll_input = channels.value[roll_angle];         // roll angle signal
    pitch_input = channels.value[pitch_angle];       // pitch angle signal
    yaw_input = channels.value[yaw_angle];           // yaw angle signal
    thrust_input = (channels.value[thrust] + 1) / 2; // thrust signal
    FRDtoFLU(roll_input, pitch_input, yaw_input);    //convert channels from FRD to FLU frame, that states from gazebo are expressed
                                                     //now all data are expressed in FLU frame
    model_inputs.header.stamp = ros::Time::now();
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

    switch (handling)
    {
    case 0: // Manual
        break;
    case 1: // Controller
        switch (model_type)
        {
        // Call control algorith based on model type
        case 1: // Plane
            break;
        case 4: // quadcopter
            PD();
            break;
        case 6: // hexacopter
            PD();
            break;
        default:
            ROS_FATAL("Invalid parameter for -/model/type- in param server!");
            ros::shutdown();
            break;
        }
        break;
    default:
        ROS_FATAL("Invalid parameter for -/model/handling- in param server!");
        ros::shutdown();
        break;
    }

    FLUtoFRD(new_roll_input, new_pitch_input, new_yaw_input); //convert signals back to FRD frame, the default model frame
    // Store input_signals separate from channels before send them to model
    res.input_signals[0] = 0;
    res.input_signals[1] = new_roll_input;   // roll signal
    res.input_signals[2] = new_pitch_input;  // pitch signal
    res.input_signals[3] = new_yaw_input;    // yaw signal
    res.input_signals[4] = new_thrust_input; // thrust signal
    for (i = 0; i < 20; i++)
    {
        res.channel_signals[i] = channels.value[i]; // all channel signals
    }
    return true;
}

//Controller functions

//initialize variables used in control
void Controller::initControllerVariables()
{
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
}

//PD controller
void Controller::PD()
{
    float error, d_error;
    float kp, kd;

    //stabilize roll
    kp = 3;
    kd = 0.5;
    error = roll_input - model_states.base_link_states.roll;
    d_error = (error - prev_roll_error) / dt;
    prev_roll_error = error; // Keep current data for next step
    new_roll_input = kp * error + kd * d_error;
    if (new_roll_input < -1)
        new_roll_input = -1;
    if (new_roll_input > 1)
        new_roll_input = 1;

    //stabilize pitch
    kp = 3;
    kd = 0.5;
    error = pitch_input - model_states.base_link_states.pitch;
    d_error = (error - prev_pitch_error) / dt;
    prev_pitch_error = error; // Keep current data for next step
    new_pitch_input = kp * error + kd * d_error;
    if (new_pitch_input < -1)
        new_pitch_input = -1;
    if (new_pitch_input > 1)
        new_pitch_input = 1;

    //yaw direction control
    kp = 1;
    kd = 1;
    yaw_direction += yaw_input * 0.001;
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
    new_yaw_input = kp * error + kd * d_error;
    if (new_yaw_input < -1)
        new_yaw_input = -1;
    if (new_yaw_input > 1)
        new_yaw_input = 1;

    //altitude control
    kp = 1;
    kd = 0.8;
    altitude = 50 * thrust_input; //control altitude from thrust signal
    error = altitude - model_states.base_link_states.z;
    d_error = (error - prev_alt_error) / dt;
    prev_alt_error = error; // Keep current data for next step
    new_thrust_input = kp * error + kd * d_error;
    if (new_thrust_input < 0)
        new_thrust_input = 0;
    if (new_thrust_input > 1)
        new_thrust_input = 1;
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