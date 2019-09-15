
class Aerodynamics 
{
  public:
    ros::NodeHandle nh;
    double M;
    double oswald;
    double a0; 
    double airspeed, alpha, beta;
    int airfoil_number;


    geometry_msgs::Vector3 airfoil_inputs;
    geometry_msgs::Vector3 relative_wind;
    
    Model *model;

     //Declare msgs 
    last_letter_2_msgs::link_states wing_states;
    last_letter_2_msgs::aero_wrenches aero_wrenches;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Vector3Stamped v_in, v_out;

    Aerodynamics(Model * parent, int id);
    // ~Aerodynamics();
    void calculationCycle();
    void getStates();
    void getInputSignals();
    void rotateWind();
    void calcTriplet();
    void calcWrench();
    virtual void calcForces()=0;
    virtual void calcTorques()=0;
};

#include "noAerodynamics.hpp"
#include "stdLinearAero.hpp"
#include "polyAero.hpp"