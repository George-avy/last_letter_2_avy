
// Class NoAero contructor
NoAerodynamics::NoAerodynamics(Model *parent, int id) : Aerodynamics(parent, id)
{   
    printf("wing:%i no aerodynamics built\n", id);
}

void NoAerodynamics::calcForces()
{
    aero_wrenches.drag = 0;
    aero_wrenches.fy = 0;
    aero_wrenches.lift = 0;
}

void NoAerodynamics::calcTorques()
{
    aero_wrenches.l = 0;
    aero_wrenches.m = 0;
    aero_wrenches.n = 0;
}