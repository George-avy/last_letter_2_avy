// Class polyAero to create polynomials for lift and drag coeffs

polyAero::polyAero(Model *parent, int id) : StdLinearAero(parent, id)
{
    char s[100];
    Factory factory;
    // Create CLift polynomial
    sprintf(s, "airfoil%i/cLiftPoly", id);
    liftCoeffPoly = factory.buildPolynomial(s);
    // Create CDrag polynomial
    sprintf(s, "airfoil%i/cDragPoly", id);
    dragCoeffPoly = factory.buildPolynomial(s);
    printf("airfoil%i type: polyAero\n", id);
}

//C_lift_alpha calculation
double polyAero::liftCoeff(double alpha)
{
    return liftCoeffPoly->evaluate(alpha);
}

//C_drag_alpha calculation
double polyAero::dragCoeff(double alpha)
{
    return dragCoeffPoly->evaluate(alpha);
}
