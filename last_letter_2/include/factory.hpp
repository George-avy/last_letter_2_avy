class Factory
{
    public:
    Aerodynamics * buildAerodynamics(Model *parent, int id);
    Propulsion * buildPropulsion(Model * parent, int id);
    Polynomial * buildPolynomial(char * );
};
