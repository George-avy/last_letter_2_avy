// A node that runs the controller for hexacopter model

#include <ros/ros.h>
#include <last_letter_2_msgs/joystick_input.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>
#include "last_letter_2_libs/math_lib.hpp"
#include <iostream>
#include <Eigen/Dense>

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

    Eigen::MatrixXf multirotor_matrix;
    Eigen::MatrixXf multirotor_matrix_inverse;
    Eigen::VectorXf commands;
    Eigen::VectorXf input_signal_vector;
   
    //Create essencial variables, based on parameter server values.
    int handling, i;
    int num_wings, num_motors;
    int roll_angle, pitch_angle, yaw_angle, thrust; 
    float roll_input, pitch_input, yaw_input, thrust_input;
    float new_roll_input, new_pitch_input, new_yaw_input, new_thrust_input;

    // variables for PD controller algorithm
    float prev_roll_error, prev_pitch_error, prev_yaw_error, prev_alt_error;
    float altitude, yaw_direction;
    float dt;
    float b, l, d;

public:
    Controller();
    void chan2signal(last_letter_2_msgs::joystick_input msg);
    void storeStates(const last_letter_2_msgs::model_states msg);
    bool returnControlInputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res);
    void initControllerVariables();
    void channelFunctions();

    //control function
    void PD();
};

Controller::Controller()
{
    //Init Subscribers
    sub_chan = n.subscribe("last_letter_2/channels", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
    sub_mod_st = n.subscribe("last_letter_2/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

    //Init service
    get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::returnControlInputs, this);

    // Read the type of handling 
    if (!ros::param::getCached("model/handling", handling)) { ROS_INFO("No mixing function selected"); handling = 0;}
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}
    //Read the update rate
    if (!ros::param::getCached("world/deltaT", dt)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}

    char paramMsg[50];

    sprintf(paramMsg, "channels/roll_angle");
    if (!ros::param::getCached(paramMsg, roll_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/pitch_angle");
    if (!ros::param::getCached(paramMsg, pitch_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/yaw_angle");
    if (!ros::param::getCached(paramMsg, yaw_angle)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    sprintf(paramMsg, "channels/thrust");
    if (!ros::param::getCached(paramMsg, thrust)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}

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
    // FRDtoFLU(roll_input, pitch_input, yaw_input);    //convert channels from FRD to FLU frame, that states from gazebo are expressed
                                                     //now all data are expressed in FLU frame
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

    //Call Controller
    PD();

    // FLUtoFRD(new_roll_input, new_pitch_input, new_yaw_input); //convert signals back to FRD frame, the default model frame
    
    //Convert PD outputs to motor inputs
    commands(0) = new_thrust_input; //thrust
    commands(1) = new_roll_input;   //roll
    commands(2) = new_pitch_input;  //pitch
    commands(3) = new_yaw_input;    //yaw
    input_signal_vector = multirotor_matrix_inverse * commands;
    std::cout<<commands<<std::endl;
    for (i = 0; i < num_motors; i++)
    {
        res.channels[i] = input_signal_vector[i]; // store calculated motor inputs 
    res.channels[i]=std::max(std::min((double)res.channels[i], 1.0), -1.0);

    std::cout<<"d"<<i<<":"<<res.channels[i]<<" ";
        
    }
    std::cout<<std::endl;
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
    
    altitude = 0;
    yaw_direction = 0;
    prev_roll_error = 0;
    prev_pitch_error = 0;
    prev_yaw_error = 0;
    prev_alt_error = 0;

    // Set the size of matrices and vectors based on model
    multirotor_matrix.resize(4, 6);
    multirotor_matrix_inverse.resize(6, 4);
    commands.resize(4);
    input_signal_vector.resize(6);

    // Read the thrust constant
    if (!ros::param::getCached("model/b", b)) { ROS_FATAL("Invalid parameters for -model/b- in param server!"); ros::shutdown(); }
    // Read the motor distance to center of gravity
    if (!ros::param::getCached("model/l", l)) { ROS_FATAL("Invalid parameters for -model/l- in param server!"); ros::shutdown(); }
    // Read the drag factor
    if (!ros::param::getCached("model/d", d)) { ROS_FATAL("Invalid parameters for -model/d- in param server!"); ros::shutdown(); }
   
    //Built hexacopter matrix
    multirotor_matrix << b,    b,           b,          b,    b,          b,            //thrust row 
                         0,   -b*l*1.73/2, -b*l*1.73/2, 0,    b*l*1.73/2, b*l*1.73/2,   //roll row
                         b*l,  b*l/2,      -b*l/2,     -b*l, -b*l/2,      b*l/2,        //pitch row
                        -d,    d,          -d,          d,   -d,          d;            //yaw row

    //calculate inverse of hexacopter matrix. Usefull for future calculations
    multirotor_matrix_inverse = multirotor_matrix.completeOrthogonalDecomposition().pseudoInverse();

}

//PD controller
void Controller::PD()
{
    float error, d_error;
    float kp, kd;

    //stabilize roll
    kp = 0.5;
    kd = 0.1;
    error = roll_input - model_states.base_link_states.roll;
    d_error = (error - prev_roll_error) / dt;
    prev_roll_error = error; // Keep current data for next step
    new_roll_input = kp * error + kd * d_error;
    if (new_roll_input < -1)
        new_roll_input = -1;
    if (new_roll_input > 1)
        new_roll_input = 1;

    //stabilize pitch
    kp = 0.5;
    kd = 0.1;
    error = pitch_input - model_states.base_link_states.pitch;
    d_error = (error - prev_pitch_error) / dt;
    prev_pitch_error = error; // Keep current data for next step
    new_pitch_input = kp * error + kd * d_error;
    if (new_pitch_input < -1)
        new_pitch_input = -1;
    if (new_pitch_input > 1)
        new_pitch_input = 1;

    //yaw direction control
    kp = 0.5;
    kd = 0.5;
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