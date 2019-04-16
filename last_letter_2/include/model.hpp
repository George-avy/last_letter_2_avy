#include <last_letter_2_headers.hpp>
#include "environment.hpp" 
#include "dynamics.hpp"
#include "aerodynamics.hpp"
#include "propulsion/propulsion.hpp"
#include "factory.hpp"
  
 // The model object
 // A class that cordinates all processes for a model step
 
 class Model
{
  public:
    ros::NodeHandle nh;
    
    //Declare service clients
    ros::ServiceClient get_control_inputs_client;
    ros::ServiceClient apply_wrench_client;
    ros::ServiceClient pauseGazebo; 

    //Declare subscribers
    ros::Subscriber gazebo_sub;

    ros::Publisher loop_number;
    // Declare service msgs
    last_letter_2_msgs::apply_model_wrenches_srv apply_wrenches_srv;
    last_letter_2_msgs::get_control_inputs_srv control_inputs_msg;

    Environment environment;
    Dynamics dynamics;
    last_letter_2_msgs::air_data airdata;
    last_letter_2_msgs::model_states model_states;
    geometry_msgs::Vector3 airfoil_inputs[3];
    float motor_input[4];
    ros::WallTime t;
    int i;
    int num_wings, num_motors;
    std_msgs::Int32 loop_num;

    //Class methods
    Model();
    void gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr&);
    void modelStep();
    void getControlInputs();
    void getAirdata();
    void calcDynamics();
    void applyWrenches();
};

