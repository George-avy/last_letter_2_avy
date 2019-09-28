class polyAero : public StdLinearAero
{
public:
  polyAero(Model *parent, int id);
  Polynomial *liftCoeffPoly;
  Polynomial *dragCoeffPoly;

  double liftCoeff(double);
  double dragCoeff(double);
};