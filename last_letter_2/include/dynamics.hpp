// Dynamics model class

class Dynamics
{
  public:
  Model * model;
  std::list<Aerodynamics*> listOfAerodynamics;
  std::list<Propulsion*> listOfPropulsion;
  int i;
  
  Dynamics(Model *);
  void calcAero();
  void calcProp();
};