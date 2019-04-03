class Propulsion
{
public:
  ros::NodeHandle nh;
  int motor_number;
  double airspeed;
  geometry_msgs::Vector3 relativeWind;
  double normalWind;
  double rotationDir; // motor direction of rotation

  float motor_input;
  geometry_msgs::Vector3 relative_wind;

  //Declare msgs
  last_letter_2_msgs::link_states motor_states;
  last_letter_2_msgs::prop_wrenches prop_wrenches;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::Vector3Stamped v_in, v_out;
    
  Model *model;
  Propulsion(Model* parent, int id);
  // ~Propulsion();
  void calculationCycle();
  void getStates();
  void getInputSignals();
  void rotateWind();
  void calcAirspeed();
  void calcWrench();
  virtual void calcThrust() = 0;
  virtual void calcTorque() = 0;
};

#include "noEngine.hpp"
#include "beardEngine.hpp"
#include "electricEngine.hpp"
