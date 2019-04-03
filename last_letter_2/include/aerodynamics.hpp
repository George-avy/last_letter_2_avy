
class Aerodynamics 
{
  public:
    ros::NodeHandle nh;
    double M = 50;
    double oswald;
    double a0; 
    double airspeed, alpha, beta;
    int airfoil_number;


    geometry_msgs::Vector3 airfoil_inputs;
    geometry_msgs::Vector3 relative_wind;
    
    Model *model;

    //Declare service clients
    ros::ServiceClient airfoil_inputs_client;

     //Declare msgs 
    last_letter_2_msgs::get_airfoil_inputs_srv airfoil_inputs_srv;
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

class NoAerodynamics : public Aerodynamics
{
  public:
    NoAerodynamics(Model * parent, int id);
    void calcForces();
    void calcTorques();
};

class StdLinearAero : public Aerodynamics
{
  public:
  double c_drag_q, c_drag_deltae, c_drag_p, c_drag_0;
  double c_lift_0, c_lift_deltae, c_lift_q, c_lift_a;
  double b, c, s;
  double c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
  double c_l_0, c_l_p, c_l_b, c_l_r, c_l_deltaa, c_l_deltar;
  double c_n_0, c_n_p, c_n_b, c_n_r, c_n_deltaa, c_n_deltar;
  double c_m_0, c_m_a, c_m_q, c_m_deltae;
  StdLinearAero(Model * parent, int id);
  void initParam(int id);
  void calcForces();
  void calcTorques();
  //Calculate lift coefficient from alpha
	virtual double liftCoeff(double );
	//Calculate drag coefficient from alpha
	virtual double dragCoeff(double);
};

class HCUAVAero : public StdLinearAero
{
  public:
  HCUAVAero(Model *  parent, int id);
  Polynomial * liftCoeffPoly;
  Polynomial * dragCoeffPoly;

  double liftCoeff(double );
  double dragCoeff(double);
};