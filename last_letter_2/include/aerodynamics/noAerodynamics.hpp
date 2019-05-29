class NoAerodynamics : public Aerodynamics
{
  public:
    NoAerodynamics(Model * parent, int id);
    void calcForces();
    void calcTorques();
};