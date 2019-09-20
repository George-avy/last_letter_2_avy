#include <last_letter_2_headers.hpp>
#include "environment.hpp" 
#include "dynamics.hpp"
#include "aerodynamics/aerodynamics.hpp"
#include "propulsion/propulsion.hpp"
#include "factory.hpp"
  
using namespace Eigen;
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

    // Declare service msgs
    last_letter_2_msgs::apply_model_wrenches_srv apply_wrenches_srv;
    last_letter_2_msgs::get_control_inputs_srv control_inputs_msg;

    Environment environment;
    Dynamics dynamics;
    last_letter_2_msgs::air_data airdata;
    last_letter_2_msgs::model_states model_states;
    geometry_msgs::Vector3 airfoil_input[3];
    geometry_msgs::Vector3 body_wind;
    geometry_msgs::TransformStamped transformStamped_;
    Eigen::Vector3d t_in, t_out;

    tf2::Quaternion quat_;
    float motor_input[6];
    float b, l, d;
    int i;
    int model_type, handling, num_wings, num_motors;
    int airfoil_in_x_chan[4], airfoil_in_y_chan[4], airfoil_in_z_chan[4];
    int motor_in_chan[6];
    float deltax_max[4], deltay_max[4], deltaz_max[4];
    float channel_input[20];

    MatrixXf multirotor_matrix;
    MatrixXf multirotor_matrix_inverse;
    VectorXf commands;
    VectorXf input_signal_vector;

    //Class methods
    Model();
    void gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr&);
    void initMultirotorMatrix();
    void modelStep();
    void getControlInputs();
    void getAirdata();
    void calcDynamics();
    void applyWrenches();
};

