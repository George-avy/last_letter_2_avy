NoEngine::NoEngine(Model *parent, int id) : Propulsion(parent, id)
{
    printf("motor%i type: noEngine\n", id);
}

void NoEngine::calcThrust()
{
    prop_wrenches.thrust = 0;
}

void NoEngine::calcTorque()
{
    prop_wrenches.torque = 0;
}

void NoEngine::calcOmega()
{
    prop_wrenches.omega = 0;
}