// A Library of Classes, usefull for Simulator

#include <last_letter_2_libs/math_lib.hpp>

Polynomial::Polynomial(){};
Polynomial::~Polynomial(){};

// Polynomial1D
Polynomial1D::Polynomial1D(int maxOrder, double *coeffArray) : Polynomial()
{
    int i;
    coeffNo = maxOrder;
    // Create and initialize polynomial coefficients container
    coeffs = (double *)malloc(sizeof(double) * (coeffNo + 1));
    for (i = 0; i <= coeffNo; i++)
    {
        coeffs[i] = coeffArray[i];
    }
}

Polynomial1D::~Polynomial1D()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial1D::evaluate(double x)
{
    int i;
    double sum = 0;
    for (i = 0; i <= coeffNo; i++)
    {
        sum += coeffs[i] * pow(x, i);
    }
    return sum;
}

// Define Polynomial2D

Polynomial2D::Polynomial2D(int maxOrder1, int maxOrder2, double *coeffArray) : Polynomial()
{
    int i;
    coeffNo1 = maxOrder1;
    coeffNo2 = maxOrder2;
    // Create and initialize polynomial coefficients container
    int arrayLen = (2 * maxOrder2 + 2 * maxOrder1 * maxOrder2 + maxOrder1 - maxOrder1 * maxOrder1 + 2) / 2;
    coeffs = (double *)malloc(sizeof(double) * arrayLen);
    for (i = 0; i < arrayLen; i++)
    {
        coeffs[i] = coeffArray[i];
    }
}

Polynomial2D::~Polynomial2D()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial2D::evaluate(double x, double y)
{
    int i, j, k = 0;
    double sum = 0;
    for (i = 0; i <= coeffNo1; i++)
    {
        for (j = 0; j <= coeffNo2; j++)
        {
            if (i + j <= coeffNo2)
            {
                sum += coeffs[k] * pow(x, i) * pow(y, j);
                k++;
            }
        }
    }
    return sum;
}

// Define Spline3
// Cubic spline, 4 parameters per variable interval

Spline3::Spline3(int breaksNoIn, double *breaksIn, double *coeffsIn) : Polynomial()
{
    int i;
    breaksNo = breaksNoIn;
    // Create and initialize breaks container
    breaks = (double *)malloc(sizeof(double) * (breaksNo + 1));
    for (i = 0; i <= breaksNo; i++)
    {
        breaks[i] = breaksIn[i];
    }
    // Create and initialize polynomial coefficients container
    coeffs = (double *)malloc(sizeof(double) * (breaksNo * 4));
    for (i = 0; i < (breaksNo * 4); i++)
    {
        coeffs[i] = coeffsIn[i];
    }
}

Spline3::~Spline3()
{
    free(breaks);
    free(coeffs);
}

// polynomial evaluation
double Spline3::evaluate(double x)
{
    int i;
    for (i = 0; i < breaksNo; i++)
    {
        if (x <= breaks[i + 1])
            break;
    }
    if (i == breaksNo)
        i--;
    double delta = x - breaks[i];
    double value = coeffs[4 * i] * pow(delta, 3) + coeffs[4 * i + 1] * pow(delta, 2) + coeffs[4 * i + 2] * delta + coeffs[4 * i + 3];
    return value;
}

//convert a vector from FLU to FRD frame
void FLUtoFRD(float &x, float &y, float &z)
{
    y = -y;
    z = -z;
}

//convert a vector from FRD to FLU frame
void FRDtoFLU(float &x, float &y, float &z)
{
    y = -y;
    z = -z;
}

// Wrap an angle to the interval [0-360)
double wrap_to_360(const double angle)
{
    double a = angle;
    while (a < 0) a += 360.0;
    while (a >= 360.0) a -= 360.0;
    return a;
}