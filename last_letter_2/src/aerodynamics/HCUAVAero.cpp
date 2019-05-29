// Class HCUAVAero constructor
HCUAVAero::HCUAVAero (Model * parent, int id) : StdLinearAero(parent, id)
{
	char s[100];
	Factory factory;
	// Create CLift polynomial
	sprintf(s,"airfoil%i/cLiftPoly",id);
	liftCoeffPoly =  factory.buildPolynomial(s);
	// Create CDrag polynomial
	sprintf(s,"airfoil%i/cDragPoly",id);
	dragCoeffPoly =  factory.buildPolynomial(s);
    // printf("wing:%i HCUAVAero built\n",id);

}

//////////////////////////
//C_lift_alpha calculation
double HCUAVAero::liftCoeff (double alpha)
{
	return liftCoeffPoly->evaluate(alpha);
}

//////////////////////////
//C_drag_alpha calculation
double HCUAVAero::dragCoeff (double alpha)
{
	return dragCoeffPoly->evaluate(alpha);
}