

class StdLinearAero : public Aerodynamics
{
  public:
  double c_drag_q, c_drag_deltae, c_drag_p, c_drag_0, c_drag_a;
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