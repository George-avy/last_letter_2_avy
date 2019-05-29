// Class StdLinearAero constructor
StdLinearAero::StdLinearAero(Model *parent, int id) : Aerodynamics(parent, id)
{
    initParam(id);
    printf("wing:%i stdLinearAero built\n",id);
}

void StdLinearAero::calcForces()
{
    float rho=model->airdata.density;
    double qbar = 0.5 * rho * pow(airspeed, 2) * s;

    double c_lift_alpha = liftCoeff(alpha);
    double c_drag_alpha = dragCoeff(alpha);

    double ca = cos(alpha);
    double sa = sin(alpha);

    double p = wing_states.p;
    double q = wing_states.q;
    double r = wing_states.r;
    double input_x = airfoil_inputs.x;
    double input_y = airfoil_inputs.y;
    double input_z = airfoil_inputs.z;

    // force calculation - - - - - - - - - - -expressed to airfoil link
    if (airspeed == 0)
    {
        aero_wrenches.drag = 0;
        aero_wrenches.fy = 0;
        aero_wrenches.lift = 0;
    }
    else
    {
        aero_wrenches.drag = qbar * ((-c_drag_alpha * ca + c_lift_alpha * sa) + (-c_drag_q * ca + c_lift_q * sa) * 0.5 / airspeed * c * q + (-c_drag_deltae * ca + c_lift_deltae * sa) * input_y);
        aero_wrenches.fy = qbar * (c_y_0 + c_y_b * beta + c_y_p * b / 2 / airspeed * p + c_y_r * b / 2 / airspeed * r + c_y_deltaa * input_x + c_y_deltar * input_z);
        aero_wrenches.lift = qbar * ((-c_drag_alpha * sa - c_lift_alpha * ca) + (-c_drag_q * sa - c_lift_q * ca) * 0.5 / airspeed * c * q + (-c_drag_deltae * sa - c_lift_deltae * ca) * input_y);
    }
}

void StdLinearAero::calcTorques()
{
    float rho=model->airdata.density;
    double qbar = 0.5 * rho * pow(airspeed, 2) * s;

    double ca = cos(alpha);
    double sa = sin(alpha);

    double p = wing_states.p;
    double q = wing_states.q;
    double r = wing_states.r;
    double input_x = airfoil_inputs.x;
    double input_y = airfoil_inputs.y;
    double input_z = airfoil_inputs.z;

    if (airspeed == 0)
    {
        aero_wrenches.l = 0;
        aero_wrenches.m = 0;
        aero_wrenches.n = 0;
    }
    else
    {
        aero_wrenches.l = qbar * (b * (c_l_0 + c_l_b * beta + c_l_p * b / 2 / airspeed * p + c_l_r * b / 2 / airspeed * r + c_l_deltaa * input_x + c_l_deltar * input_z));
        aero_wrenches.m = qbar * (c * (c_m_0 + c_m_a * alpha + c_m_q * c / 2 / airspeed * q + c_m_deltae * input_y));
        aero_wrenches.n = qbar * (b * (c_n_0 + c_n_b * beta + c_n_p * b / 2 / airspeed * p + c_n_r * b / 2 / airspeed * r + c_n_deltaa * input_x + c_n_deltar * input_z));
    }
}

double StdLinearAero::liftCoeff(double alpha)
{
    double sigmoid = (1 + exp(-M * (alpha - a0)) + exp(M * (alpha + a0))) / (1 + exp(-M * (alpha - a0))) / (1 + exp(M * (alpha + a0)));
    if (isnan(sigmoid))
        sigmoid = 0;
    double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a * alpha);                         //Lift at small AoA
    double flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); //Lift beyond stall
    return linear + flatPlate;
}

double StdLinearAero::dragCoeff(double alpha)
{
    double AR = pow(b, 2) / s;
    double c_drag_alpha = c_drag_p + pow(c_lift_0 + c_lift_a * alpha, 2) / (M_PI * oswald * AR);
    return c_drag_alpha;
}

// Load from parameter the essential values for calculations
void StdLinearAero::initParam(int id)
{
    char paramMsg[50];
    sprintf(paramMsg, "airfoil%i/c_drag_p", id);
    if (!ros::param::getCached(paramMsg, c_drag_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_drag_deltae", id);
    if (!ros::param::getCached(paramMsg, c_drag_deltae)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_drag_q", id);
    if (!ros::param::getCached(paramMsg, c_drag_q)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_drag_0", id);
    if (!ros::param::getCached(paramMsg, c_drag_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_lift_0", id);
    if (!ros::param::getCached(paramMsg, c_lift_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_lift_deltae", id);
    if (!ros::param::getCached(paramMsg, c_lift_deltae)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_lift_q", id);
    if (!ros::param::getCached(paramMsg, c_lift_q)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_lift_a", id);
    if (!ros::param::getCached(paramMsg, c_lift_a)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/alpha_stall", id);
    if (!ros::param::getCached(paramMsg, a0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/s", id);
    if (!ros::param::getCached(paramMsg, s)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/b", id);
    if (!ros::param::getCached(paramMsg, b)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c", id);
    if (!ros::param::getCached(paramMsg, c)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_0", id);
    if (!ros::param::getCached(paramMsg, c_y_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_b", id);
    if (!ros::param::getCached(paramMsg, c_y_b)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_p", id);
    if (!ros::param::getCached(paramMsg, c_y_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_r", id);
    if (!ros::param::getCached(paramMsg, c_y_r)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_deltaa", id);
    if (!ros::param::getCached(paramMsg, c_y_deltaa)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_deltar", id);
    if (!ros::param::getCached(paramMsg, c_y_deltar)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_0", id);
    if (!ros::param::getCached(paramMsg, c_l_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_p", id);
    if (!ros::param::getCached(paramMsg, c_l_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_b", id);
    if (!ros::param::getCached(paramMsg, c_l_b)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_r", id);
    if (!ros::param::getCached(paramMsg, c_l_r)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_deltaa", id);
    if (!ros::param::getCached(paramMsg, c_l_deltaa)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_deltar", id);
    if (!ros::param::getCached(paramMsg, c_l_deltar)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_0", id);
    if (!ros::param::getCached(paramMsg, c_m_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_a", id);
    if (!ros::param::getCached(paramMsg, c_m_a)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_q", id);
    if (!ros::param::getCached(paramMsg, c_m_q)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_deltae", id);
    if (!ros::param::getCached(paramMsg, c_m_deltae)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_0", id);
    if (!ros::param::getCached(paramMsg, c_n_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_b", id);
    if (!ros::param::getCached(paramMsg, c_n_b)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_p", id);
    if (!ros::param::getCached(paramMsg, c_n_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_r", id);
    if (!ros::param::getCached(paramMsg, c_n_r)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_deltaa", id);
    if (!ros::param::getCached(paramMsg, c_n_deltaa)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_deltar", id);
    if (!ros::param::getCached(paramMsg, c_n_deltar)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/oswald", id);
    if (!ros::param::getCached(paramMsg, oswald)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
}