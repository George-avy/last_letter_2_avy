// A node that convert channels signals, to input signals for the model,
// based on what kind of model is loaded (plane, multirotor etc)

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>
#include <last_letter_2_msgs/joystick_input.h>
#include <last_letter_2_msgs/model_inputs.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>

#include "control_types.cpp"


class Controller
{
private:
  ros::Subscriber sub_chan;
  ros::Subscriber sub_mod_st;
  ros::NodeHandle n;

  ros::ServiceServer get_control_inputs_service;

  int mixerid;
  int num_wings;
  int num_motors;
  int i;
  last_letter_2_msgs::joystick_input channels;
  last_letter_2_msgs::model_inputs model_inputs;
  last_letter_2_msgs::model_states model_states;


  //Create essencial variables, based on parameter server values.
  int aileron, elevator, rudder, thrust;
  int roll_angle, pitch_angle, yaw_angle;
  float input_signals[5];

  float delta_a, delta_e, delta_r, delta_t;
  float prev_roll_error, prev_pitch_error, prev_error;
  float dis_alt;

public:
  Controller()
  {
      //Subscribtions
      sub_chan = n.subscribe("last_letter_2/rawPWM", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
      sub_mod_st = n.subscribe("last_letter_2/gazebo/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

      //Service server
      get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::return_control_inputs, this);

      // Read the mixer type
      if (!ros::param::getCached("HID/mixerid", mixerid)) { ROS_INFO("No mixing function selected"); mixerid = 0;}
      //Read the number of airfoils
      if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
      //Read the number of motors
      if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}

      char paramMsg[50];

      
      switch (mixerid)
        {
        case 0: // No mixing applied
            break;
        case 1: // Airplane mixing
            sprintf(paramMsg, "aileron");
            if (!ros::param::getCached(paramMsg, aileron))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "elevator");
            if (!ros::param::getCached(paramMsg, elevator))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "rudder");
            if (!ros::param::getCached(paramMsg, rudder))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "thrust");
            if (!ros::param::getCached(paramMsg, thrust))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            break;
        case 2: // Mulrtirotor mixing
            // Multirotor inputs
            sprintf(paramMsg, "roll_angle");
            if (!ros::param::getCached(paramMsg, roll_angle))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "pitch_angle");
            if (!ros::param::getCached(paramMsg, pitch_angle))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "yaw_angle");
            if (!ros::param::getCached(paramMsg, yaw_angle))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "thrust");
            if (!ros::param::getCached(paramMsg, thrust))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            break;
        default:
            ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
            ros::shutdown();
            break;
        }
      delta_a = 0;
      delta_e = 0;
      delta_r = 0;
      delta_t = 0;
      dis_alt = 0;
      prev_roll_error = 0;
      prev_pitch_error = 0;
      prev_error = 0;
    }

    //Store new joystick changes
    void chan2signal(last_letter_2_msgs::joystick_input msg)
    {
        channels = msg;
        //Choose which channel covnertion, based on model type
        switch (mixerid)
        {
        case 0: // No mixing applied
            break;
        case 1: // Airplane mixing
            // Airplane inputs
            delta_a = (float) (channels.value[aileron] - 1500) / 500;   // aileron signal
            delta_e = (float) (channels.value[elevator] - 1500) / 500;  // elevator signal
            delta_r = (float) (channels.value[rudder] - 1500) / 500;    // rudder signal
            delta_t = (float) (channels.value[thrust] - 1000) / 1000;   // thrust signal

            break;
        case 2: // Mulrtirotor mixing
            // Multirotor inputs
            delta_a = (float) (channels.value[roll_angle] - 1500) / 500;   // roll angle signal
            delta_e = (float) (channels.value[pitch_angle] - 1500) / 500;  // pitch angle signal
            delta_r = (float) (channels.value[yaw_angle] - 1500) / 500;    // yaw angle signal
            delta_t = (float) (channels.value[thrust] - 1000) / 1000;   //  thrust signal
            break;
        default:
            ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
            ros::shutdown();
            break;
        }
        model_inputs.header.stamp = ros::Time::now();
    }

    // store the model_states published by gazebo
    void storeStates(const last_letter_2_msgs::model_states::ConstPtr& msg)
    {
        model_states.base_link_states = msg->base_link_states;
        model_states.airfoil_states = msg->airfoil_states;
        model_states.motor_states = msg->motor_states;
    }

    bool return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res)
    {
        float temp_delta_a, temp_delta_e, temp_delta_t;
        temp_delta_a=delta_a;
        temp_delta_e = delta_e;
        temp_delta_t = delta_t;
        dis_alt = 50 * delta_t;

        PD(model_states, channels, model_inputs, temp_delta_a, temp_delta_e, temp_delta_t, prev_roll_error, prev_pitch_error, prev_error, dis_alt);

        res.input_signals[0] = 0;
        res.input_signals[1] = temp_delta_a; // aileron signal
        res.input_signals[2] = temp_delta_e; // elevator signal
        res.input_signals[3] = delta_r; // rudder signal
        res.input_signals[4] = temp_delta_t; // thrust signal
        
        // Keep current data for next step
        prev_roll_error=delta_a-model_states.base_link_states.roll;
        prev_pitch_error=delta_e-model_states.base_link_states.pitch;
        prev_error = dis_alt - model_states.base_link_states.z;

        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");

    Controller controller;

    //Build a thread to spin for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}