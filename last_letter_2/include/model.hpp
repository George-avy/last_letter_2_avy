#include <last_letter_2_headers.hpp>
#include "environment.hpp"
#include "dynamics.hpp"
#include "aerodynamics/aerodynamics.hpp"
#include "propulsion/propulsion.hpp"
#include "factory.hpp"

class Model
{
public:
  ros::NodeHandle nh;

  ros::Time current_time_;

  //Declare service clients
  ros::ServiceClient get_control_inputs_client;
  ros::ServiceClient apply_wrench_client;

  //Declare subscribers
  ros::Subscriber gazebo_sub;

  // Declare service msgs
  last_letter_2_msgs::apply_model_wrenches_srv apply_wrenches_srv;
  last_letter_2_msgs::get_control_inputs_srv control_inputs_msg;
  
  last_letter_2_msgs::air_data airdata;
  last_letter_2_msgs::model_states model_states;
  
  Environment environment;
  Dynamics dynamics;

  float motor_input[10];
  geometry_msgs::Vector3 airfoil_input[10];
  geometry_msgs::Vector3 body_wind;
  KDL::Frame transformation_matrix;
  tf2::Stamped<KDL::Vector> v_out;

  int i, num_wings, num_motors;
  int airfoil_in_x_chan[10], airfoil_in_y_chan[10], airfoil_in_z_chan[10];
  int motor_in_chan[10];
  int start_paused;
  float deltax_max[10], deltay_max[10], deltaz_max[10];
  float channel_input[20];

  Eigen::MatrixXf multirotor_matrix;
  Eigen::MatrixXf multirotor_matrix_inverse;
  Eigen::VectorXf commands;
  Eigen::VectorXf input_signal_vector;

  //Class methods
  Model();
  void gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr &);
  void initMultirotorMatrix();
  void modelStep();
  void getControlInputs();
  void getAirdata();
  void calcDynamics();
  void applyWrenches();
};