// A node that convert channels signals, to input signals for the model,
// based on what kind of model is loaded (plane, multirotor etc)

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>
#include <last_letter_2_msgs/joystick_input.h>
#include <last_letter_2_msgs/model_inputs.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>

ros::Publisher pub;

int mixerid;
int num_wings;
int num_motors;
int i;
last_letter_2_msgs::joystick_input channels;
last_letter_2_msgs::model_inputs model_inputs;

//Store new joystick changes
void chan2signal(last_letter_2_msgs::joystick_input msg)
{
    channels=msg;
}

bool return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                           last_letter_2_msgs::get_control_inputs_srv::Response &res)
{
    for (i = 0; i < num_wings; i++)
    {
        res.airfoil_inputs[i].x=model_inputs.wing_input_x[i];
        res.airfoil_inputs[i].y=model_inputs.wing_input_y[i];
        res.airfoil_inputs[i].z=model_inputs.wing_input_z[i];
    }
    // Motor inputs
    for (i = 0; i < num_motors; i++)
    {
        res.motor_input[i] = model_inputs.motor_input[i];
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;

    //Subscribtions
    ros::Subscriber sub = n.subscribe("last_letter_2/rawPWM", 1, chan2signal,ros::TransportHints().tcpNoDelay());
    pub = n.advertise<last_letter_2_msgs::model_inputs>("last_letter_2/model_inputs", 1);

    //Servers 
    ros::ServiceServer get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", return_control_inputs);
   
    int i;
    ros::WallRate r(1100);

    // Read the mixer type
    if (!ros::param::getCached("HID/mixerid", mixerid)) { ROS_INFO("No mixing function selected"); mixerid = 0; }
    //Read the number of airfoils
    if (!ros::param::getCached("airfoil/nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
    //Read the number of motors
    if (!ros::param::getCached("motor/nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }
    
    //Create essencial variables, based on param server values.
    int aileron[num_wings], elevator[num_wings], rudder[num_wings], motor_enable[num_motors];
    int chanAileron[num_wings], chanElevator[num_wings], chanRudder[num_wings], chanMotor[num_motors];
    float wing_input_x[num_wings], wing_input_y[num_wings], wing_input_z[num_wings], motor_input[num_motors];
    float deltax_max[num_wings], deltay_max[num_wings], deltaz_max[num_wings];

    char paramMsg[50];
    //Load basic characteristics for each airfoil
    for (i = 0; i < num_wings; ++i)
    {
        sprintf(paramMsg, "airfoil%i/aileron", i+1);
        if (!ros::param::getCached(paramMsg, aileron[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/elevator", i+1);
        if (!ros::param::getCached(paramMsg, elevator[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/rudder", i+1);
        if (!ros::param::getCached(paramMsg, rudder[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/chanAileron", i+1);
        if (!ros::param::getCached(paramMsg, chanAileron[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/chanElevator", i+1);
        if (!ros::param::getCached(paramMsg, chanElevator[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/chanRudder", i+1);
        if (!ros::param::getCached(paramMsg, chanRudder[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/deltax_max", i+1);
        if (!ros::param::getCached(paramMsg, deltax_max[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/deltay_max", i+1);
        if (!ros::param::getCached(paramMsg, deltay_max[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "airfoil%i/deltaz_max", i+1);
        if (!ros::param::getCached(paramMsg, deltaz_max[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        
    }

    //load basic characteristics for each motor
    for (i = 0; i < num_motors; ++i)
    {
        sprintf(paramMsg, "motor%i/enable", i+1);
        if (!ros::param::getCached(paramMsg, motor_enable[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
        sprintf(paramMsg, "motor%i/chanMotor", i+1);
        if (!ros::param::getCached(paramMsg, chanMotor[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
    }

    //Calculation loop
    
    while (ros::ok())
    {
        //Choose which channel covnertion, based on model type
        switch (mixerid)
        {
        case 0: // No mixing applied
            break;
        case 1: // Airplane mixing
            // Airfoil inputs
            for (i = 0; i < num_wings; i++)
            {
                model_inputs.wing_input_x[i] = aileron[i] * (deltax_max[i] * (channels.value[chanAileron[i]] - 1500) / 500);
                model_inputs.wing_input_y[i] = elevator[i] * (deltay_max[i] * (channels.value[chanElevator[i]] - 1500) / 500);
                model_inputs.wing_input_z[i] = rudder[i] * (deltaz_max[i] * (channels.value[chanRudder[i]] - 1500) / 500);
            }
            // Motor inputs
            for (i = 0; i < num_motors; i++)
            {
                model_inputs.motor_input[i] = motor_enable[i] * ((float)(channels.value[chanMotor[i]] - 1000)/1000);
            }
            break;
        case 2: // Quadrotor mixing
            // Motor inputs
            for (i = 0; i < num_motors; i++)
            {
                model_inputs.motor_input[i] = motor_enable[i] * ((float)(channels.value[chanMotor[i]]-1000)/1000) ;
            }
            break;
        default:
            ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
            ros::shutdown();
            break;
        }
        //Publish data
        model_inputs.header.stamp = ros::Time::now();
        // pub.publish(model_inputs);    // no need, will be send through srv
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
