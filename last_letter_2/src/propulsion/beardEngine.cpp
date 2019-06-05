BeardEngine::BeardEngine(Model *parent, int id) : Propulsion(parent, id)
{
    initParam(id);
    printf("motor:%i beard engine built\n",id);
}

void BeardEngine::calcThrust()
{
    float rho=model->airdata.density;
    prop_wrenches.thrust = 1.0 / 2.0 * rho * s_prop * c_prop * (pow(motor_input * k_motor, 2) - pow(airspeed, 2));
}

void BeardEngine::calcTorque()
{
    float rho=model->airdata.density;
    prop_wrenches.torque = -k_t_p * pow((k_omega * motor_input), 2);
}

void BeardEngine::calcOmega()
{
    prop_wrenches.omega = k_omega * motor_input;
}

void BeardEngine::initParam(int id)
{
    char paramMsg[50];
    sprintf(paramMsg, "motor%i/s_prop", id);
    if (!ros::param::getCached(paramMsg, s_prop)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/c_prop", id);
    if (!ros::param::getCached(paramMsg, c_prop)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/k_motor", id);
    if (!ros::param::getCached(paramMsg, k_motor)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/k_omega", id);
    if (!ros::param::getCached(paramMsg, k_omega)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "motor%i/k_t_p", id);
    if (!ros::param::getCached(paramMsg, k_t_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
}
