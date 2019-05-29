
class HCUAVAero : public StdLinearAero
{
  public:
  HCUAVAero(Model *  parent, int id);
  Polynomial * liftCoeffPoly;
  Polynomial * dragCoeffPoly;

  double liftCoeff(double );
  double dragCoeff(double);
};