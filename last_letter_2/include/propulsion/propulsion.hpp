class Propulsion
{
public:
  int motor_number;
  double airspeed;
  double normalWind;
  double rotationDir;
  float motor_input;
  geometry_msgs::Vector3 relative_wind;

  //Declare msgs
  last_letter_2_msgs::link_states motor_states;
  last_letter_2_msgs::prop_wrenches prop_wrenches;

  KDL::Frame transformation_matrix;
  tf2::Stamped<KDL::Vector> v_out;

  Model *model;
  Propulsion(Model *parent, int id);
  void calculationCycle();
  void getStates();
  void getInputSignals();
  void rotateWind();
  void calcAirspeed();
  void calcWrench();
  virtual void calcThrust() = 0;
  virtual void calcTorque() = 0;
  virtual void calcOmega() = 0;
};

#include "noEngine.hpp"
#include "genericEngine.hpp"
#include "electricEngine.hpp"
