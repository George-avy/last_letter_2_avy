class genericEngine : public Propulsion
{
public:
  float s_prop, c_prop, k_motor, k_omega, k_t_p, rotationDir;
  genericEngine(Model *parent, int id);
  void initParam(int id);
  void calcThrust();
  void calcTorque();
  void calcOmega();
};