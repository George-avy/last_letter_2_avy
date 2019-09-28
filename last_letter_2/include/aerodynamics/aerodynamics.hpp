class Aerodynamics
{
public:
  ros::NodeHandle nh;
  double M, oswald, a0;
  double airspeed, alpha, beta;
  int airfoil_number;

  geometry_msgs::Vector3 airfoil_inputs;
  geometry_msgs::Vector3 relative_wind;

  Model *model;

  last_letter_2_msgs::link_states airfoil_states;
  last_letter_2_msgs::aero_wrenches aero_wrenches;

  KDL::Frame transformation_matrix;
  tf2::Stamped<KDL::Vector> v_out;

  Aerodynamics(Model *parent, int id);
  void calculationCycle();
  void getStates();
  void getInputSignals();
  void rotateWind();
  void calcTriplet();
  void calcWrench();
  virtual void calcForces() = 0;
  virtual void calcTorques() = 0;
};

#include "noAerodynamics.hpp"
#include "stdLinearAero.hpp"
#include "polyAero.hpp"