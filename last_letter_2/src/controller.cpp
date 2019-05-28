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


  //Create essencial variables, based on param server values.
  int roll_enable[4], pitch_enable[4], yaw_enable[4], motor_enable[4];
  int chan_in_x[4], chan_in_y[4], chan_in_z[4], chanMotor[4];
  float wing_input_x[4], wing_input_y[4], wing_input_z[4], motor_input[4];
  float deltax_max[4], deltay_max[4], deltaz_max[4];

public:
  Controller()
  {
      //Subscribtions
      sub_chan = n.subscribe("last_letter_2/rawPWM", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
      sub_mod_st = n.subscribe("last_letter_2/gazebo/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

      //Servers
      get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::return_control_inputs, this);

      // Read the mixer type
      if (!ros::param::getCached("HID/mixerid", mixerid)) { ROS_INFO("No mixing function selected"); mixerid = 0;}
      //Read the number of airfoils
      if (!ros::param::getCached("airfoil/nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
      //Read the number of motors
      if (!ros::param::getCached("motor/nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}

      char paramMsg[50];

      //Load basic characteristics for each airfoil
      for (i = 0; i < num_wings; ++i)
      {
          sprintf(paramMsg, "airfoil%i/roll_enable", i + 1);
          if (!ros::param::getCached(paramMsg, roll_enable[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/pitch_enable", i + 1);
          if (!ros::param::getCached(paramMsg, pitch_enable[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/yaw_enable", i + 1);
          if (!ros::param::getCached(paramMsg, yaw_enable[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/chan_in_x", i + 1);
          if (!ros::param::getCached(paramMsg, chan_in_x[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/chan_in_y", i + 1);
          if (!ros::param::getCached(paramMsg, chan_in_y[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/chan_in_z", i + 1);
          if (!ros::param::getCached(paramMsg, chan_in_z[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/deltax_max", i + 1);
          if (!ros::param::getCached(paramMsg, deltax_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/deltay_max", i + 1);
          if (!ros::param::getCached(paramMsg, deltay_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/deltaz_max", i + 1);
          if (!ros::param::getCached(paramMsg, deltaz_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
      }

      //load basic characteristics for each motor
      for (i = 0; i < num_motors; ++i)
      {
          sprintf(paramMsg, "motor%i/enable", i + 1);
          if (!ros::param::getCached(paramMsg, motor_enable[i])) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "motor%i/chanMotor", i + 1);
          if (!ros::param::getCached(paramMsg, chanMotor[i])) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
      }
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
            // Airfoil inputs
            for (i = 0; i < num_wings; i++)
            {
                model_inputs.wing_input_x[i] = roll_enable[i] * (deltax_max[i] * (channels.value[chan_in_x[i]] - 1500) / 500);
                model_inputs.wing_input_y[i] = pitch_enable[i] * (deltay_max[i] * (channels.value[chan_in_y[i]] - 1500) / 500);
                model_inputs.wing_input_z[i] = yaw_enable[i] * (deltaz_max[i] * (channels.value[chan_in_z[i]] - 1500) / 500);
            }
            // Motor inputs
            for (i = 0; i < num_motors; i++)
            {
                model_inputs.motor_input[i] = motor_enable[i] * ((float)(channels.value[chanMotor[i]] - 1000) / 1000);
            }
            break;
        case 2: // Quadrotor mixing
            // Motor inputs
            for (i = 0; i < num_motors; i++)
            {
                model_inputs.motor_input[i] = motor_enable[i] * ((float)(channels.value[chanMotor[i]] - 1000) / 1000);
            }
            break;
        default:
            ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
            ros::shutdown();
            break;
        }
        model_inputs.header.stamp = ros::Time::now();
    }

    void storeStates(const last_letter_2_msgs::model_states::ConstPtr& msg)
    {
        model_states.base_link_states = msg->base_link_states;
        model_states.airfoil_states = msg->airfoil_states;
        model_states.motor_states = msg->motor_states;
    }

    bool return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res)
    {
        // PD(model_states);
        // Airfoil inputs
        for (i = 0; i < num_wings; i++)
        {
            res.airfoil_inputs[i].x = model_inputs.wing_input_x[i];
            res.airfoil_inputs[i].y = model_inputs.wing_input_y[i];
            res.airfoil_inputs[i].z = model_inputs.wing_input_z[i];
        }

        // Motor inputs
        for (i = 0; i < num_motors; i++)
        {
            res.motor_input[i] = model_inputs.motor_input[i];
        }
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