class electricEngine : public Propulsion
{
public:
  double omegaMin, omegaMax;
  double propDiam, engInertia, rho;
  double Kv, Rm, I0;
  double maxThrust;

  double Ei, Im, engPower, advRatio, propPower;
  double npCoeff, fadeFactor, staticThrust, deltaT, omegaDot;
  // Battery specification
  double omega;       // motor angular speed in rad/s
  int Cells;          // Number of LiPo cells
  double Rs;          // Battery internal resistance
  double rotationDir; // motor direction of rotation

  Polynomial *engPowerPoly, *npPoly, *propPowerPoly;

  electricEngine(Model *parent, int id);
  ~electricEngine();

  void calcThrust();
  void calcTorque();
  void calcOmega();
};