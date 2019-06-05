////////////////////////
// Electric engine model
////////////////////////

// Constructor
ElectricEng::ElectricEng(Model *parent, int id) : Propulsion(parent,id)
{
    XmlRpc::XmlRpcValue list;
    int i, length;
    char s[100];
    char paramMsg[50];

    sprintf(paramMsg, "motor%i/propDiam", id);
    if (!ros::param::getCached(paramMsg, propDiam)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/rotationDir", id);
    if (!ros::param::getCached(paramMsg, rotationDir)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/engInertia", id);
    if (!ros::param::getCached(paramMsg, engInertia)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/Kv", id);
    if (!ros::param::getCached(paramMsg, Kv)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/Rm", id);
    if (!ros::param::getCached(paramMsg, Rm)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "battery%i/Rs", id);
    if (!ros::param::getCached(paramMsg, Rs)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "battery%i/Cells", id);
    if (!ros::param::getCached(paramMsg, Cells)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/I0", id);
    if (!ros::param::getCached(paramMsg, I0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airframe/m");
    if (!ros::param::getCached(paramMsg, mass)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    // Initialize RadPS limits
    sprintf(paramMsg, "motor%i/RadPSLimits", id);
    if (!ros::param::getCached(paramMsg, list)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    omegaMin = list[0];
    omegaMax = list[1];

    Factory factory;
    // Create propeller efficiency polynomial
    sprintf(s, "motor%i/nCoeffPoly", id);
    npPoly = factory.buildPolynomial(s);
    // Create propeller power polynomial
    sprintf(s, "motor%i/propPowerPoly", id);
    propPowerPoly = factory.buildPolynomial(s);

    omega = omegaMin; // Initialize engine rotational speed

    prop_wrenches.thrust = 0.0;
    prop_wrenches.torque = 0.0;

    sprintf(paramMsg, "propulsion%i", id);
    ros::NodeHandle n;
}

// Destructor
ElectricEng::~ElectricEng()
{
    delete npPoly;
    delete propPowerPoly;
}

// Update motor rotational speed and calculate thrust
void ElectricEng::calcThrust()
{

    rho = model->airdata.density;
    double inputMotor = motor_input;
    double Ei = std::fabs(omega) / 2 / M_PI / Kv;
    // double Ei = rotationDir * omega/2/M_PI/Kv;
    double Im = (Cells * 4.0 * inputMotor - Ei) / (Rs * inputMotor + Rm);
    // Im = std::max(Im,0.0); // Current cannot return to the ESC
    // Im = std::max(Im,-1.0); // Allow limited current back to the ESC
    double engPower = Ei * (Im - I0);
    double advRatio = normalWind / (std::fabs(omega) / 2.0 / M_PI) / propDiam; // Convert advance ratio to dimensionless units, not 1/rad
    // advRatio = std::max(advRatio, 0.0); // thrust advance ratio above zero, in lack of a better propeller model
    double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 3) * pow(propDiam, 5);

    double npCoeff = npPoly->evaluate(advRatio);
    prop_wrenches.thrust = propPower * std::fabs(npCoeff / (normalWind + 1.0e-10)); // Added epsilon for numerical stability

    double fadeFactor = (exp(-normalWind * 3 / 12));
    double staticThrust = 0.9 * fadeFactor * pow(M_PI / 2.0 * propDiam * propDiam * rho * engPower * engPower, 1.0 / 3); //static thrust fades at 5% at 12m/s
    prop_wrenches.thrust = prop_wrenches.thrust + staticThrust;
    // Constrain propeller thrust to [0,+5] times the aircraft weight
    prop_wrenches.thrust = std::max(std::min(double(prop_wrenches.thrust), 5.0 * mass * 9.81), 0.0 * mass * 9.81);
    prop_wrenches.torque = propPower / omega;
    if (inputMotor < 0.01)
    {
        prop_wrenches.thrust = 0;
        prop_wrenches.torque = 0;
    } // To avoid aircraft rolling and turning on the ground while throttle is off
    // double deltaP = model->kinematics.thrustInput.x * model->states.velocity.linear.x / npCoeff;
    double deltaT = (engPower - propPower) / std::fabs(omega);
    double omegaDot = 1 / engInertia * deltaT;

    omega += rotationDir * omegaDot * motor_input;

    omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range
    if (!std::isfinite(prop_wrenches.thrust)) { ROS_FATAL("propulsion.cpp: State NaN in prop_wrenches.thrust"); ros::shutdown(); }
     // model->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

void ElectricEng::calcTorque()
{
    if (!std::isfinite(prop_wrenches.torque))
    {
        ROS_FATAL("propulsion.cpp: State NaN in prop_wrenches.torque");
        ros::shutdown();
    }
}

void ElectricEng::calcOmega()
{
    prop_wrenches.omega = omega;
}
