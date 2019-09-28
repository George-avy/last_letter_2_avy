// Electric engine class
electricEngine::electricEngine(Model *parent, int id) : Propulsion(parent,id)
{
    XmlRpc::XmlRpcValue list;
    int i, length;
    char s[100];
    char paramMsg[50];

    sprintf(paramMsg, "prop%i/propDiam", id);
    if (!ros::param::getCached(paramMsg, propDiam)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/rotationDir", id);
    if (!ros::param::getCached(paramMsg, rotationDir)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/engInertia", id);
    if (!ros::param::getCached(paramMsg, engInertia)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/Kv", id);
    if (!ros::param::getCached(paramMsg, Kv)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/Rm", id);
    if (!ros::param::getCached(paramMsg, Rm)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "battery/Rs");
    if (!ros::param::getCached(paramMsg, Rs)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "battery/Cells");
    if (!ros::param::getCached(paramMsg, Cells)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/I0", id);
    if (!ros::param::getCached(paramMsg, I0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/maxThrust", id);
    if (!ros::param::getCached(paramMsg, maxThrust)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    // Initialize RadPS limits
    sprintf(paramMsg, "motor%i/RadPSLimits", id);
    if (!ros::param::getCached(paramMsg, list)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    omegaMin = list[0];
    omegaMax = list[1];

    Factory factory;
    // Create propeller efficiency polynomial
    sprintf(s, "prop%i/nCoeffPoly", id);
    npPoly = factory.buildPolynomial(s);
    // Create propeller power polynomial
    sprintf(s, "prop%i/powerPoly", id);
    propPowerPoly = factory.buildPolynomial(s);

    // Initialize engine rotational speed, thrust, torque
    omega = omegaMin;
    prop_wrenches.thrust = 0.0;
    prop_wrenches.torque = 0.0;

    printf("motor%i type: electricEngine\n", id);
}

electricEngine::~electricEngine()
{
    delete npPoly;
    delete propPowerPoly;
}

// calculate thrust
void electricEngine::calcThrust()
{
    rho = model->airdata.density;
    Ei = std::fabs(omega) / 2 / M_PI / Kv;
    Im = (Cells * 4.0 * motor_input - Ei) / (Rs * motor_input + Rm);
    engPower = Ei * (Im - I0);
    advRatio = normalWind / (std::fabs(omega) / 2.0 / M_PI) / propDiam; // Convert advance ratio to dimensionless units, not 1/rad
    propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 3) * pow(propDiam, 5);
    npCoeff = npPoly->evaluate(advRatio);

    prop_wrenches.thrust = propPower * std::fabs(npCoeff / (normalWind + 1.0e-10)); // Added epsilon for numerical stability

    fadeFactor = (exp(-normalWind * 3 / 12));
    staticThrust = 0.9 * fadeFactor * pow(M_PI / 2.0 * propDiam * propDiam * rho * engPower * engPower, 1.0 / 3); //static thrust fades at 5% at 12m/s
    prop_wrenches.thrust = prop_wrenches.thrust + staticThrust;
    // Constrain propeller thrust to [0,maxThrust]
    prop_wrenches.thrust = std::max(std::min(double(prop_wrenches.thrust), maxThrust), 0.0);
    if (motor_input < 0.01) { prop_wrenches.thrust = 0; }
}

// Calculate motor torque
void electricEngine::calcTorque()
{
    prop_wrenches.torque = propPower / omega;
    if (motor_input < 0.01)
    {
        prop_wrenches.torque = 0;
    }
    if (!std::isfinite(prop_wrenches.torque)) { ROS_FATAL("propulsion.cpp: State NaN in prop_wrenches.torque"); ros::shutdown();}
}

// Calculate omega
void electricEngine::calcOmega()
{
    deltaT = (engPower - propPower) / std::fabs(omega);
    omegaDot = 1 / engInertia * deltaT;
    omega += rotationDir * omegaDot * motor_input;
    omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range
    if (!std::isfinite(prop_wrenches.thrust)) { ROS_FATAL("propulsion.cpp: State NaN in prop_wrenches.thrust"); ros::shutdown(); }
    
    prop_wrenches.omega = omega;
}
