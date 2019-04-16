#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cstdio>
#include <cmath>

class Polynomial
{
public:
  Polynomial();
  ~Polynomial();
  virtual double evaluate() { return 0; }
  virtual double evaluate(double x) { return 0; }
  virtual double evaluate(double x, double y) { return 0; }
};

class Polynomial1D : public Polynomial
{
public:
  double coeffNo;
  double *coeffs;

  Polynomial1D(int maxOrder, double *coeffArray);
  ~Polynomial1D();
  double evaluate(double x);
};

class Polynomial2D : public Polynomial
{
public:
  double coeffNo1, coeffNo2;
  double *coeffs;

  Polynomial2D(int maxOrder1, int maxOrder2, double *coeffArray);
  ~Polynomial2D();
  double evaluate(double x, double y);
  // Important notes: maxOrder of variable y must be greater or equal to maxOrder of variable x
  // Pass the coefficient array with the following ordering (eg for maxOrder1=1, maxOrder2=3):
  // [00 01 02 03 10 11 12]
};

class Spline3 : public Polynomial
{
public:
  int breaksNo;
  double *breaks, *coeffs;

  Spline3(int breaksNoIn, double *breaksIn, double *coeffsIn);
  ~Spline3();
  double evaluate(double x);
};

void FLUtoFRD(float &x, float &y, float &z);
void FRDtoFLU(float &x, float &y, float &z);