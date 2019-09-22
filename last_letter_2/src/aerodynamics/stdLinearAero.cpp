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

    float p = airfoil_states.p;
    float q = airfoil_states.q;
    float r = airfoil_states.r;
    // Transform relative angular speed from airfoil_FLU to airfoil_FRD for calculations
    FLUtoFRD(p, q, r);
    double input_x = airfoil_inputs.x;
    double input_y = airfoil_inputs.y;
    double input_z = airfoil_inputs.z;
    std::cout<<"input_y:"<<input_y<<std::endl;
    // force calculation - - - - - - - - - - -expressed to airfoil link
    if (airspeed == 0)
    {
        aero_wrenches.drag = 0;
        aero_wrenches.fy = 0;
        aero_wrenches.lift = 0;
    }
    else
    {
        aero_wrenches.drag = qbar * ((-c_drag_alpha * ca + c_lift_alpha * sa) + (-c_drag_q * ca + c_lift_q * sa) * 0.5 / airspeed * c * q + (-c_drag_input_y * ca + c_lift_input_y * sa) * input_y);
        aero_wrenches.fy = qbar * (c_y_0 + c_y_b * beta + c_y_p * b / 2 / airspeed * p + c_y_r * b / 2 / airspeed * r + c_y_input_x * input_x + c_y_input_z * input_z);
        aero_wrenches.lift = qbar * ((-c_drag_alpha * sa - c_lift_alpha * ca) + (-c_drag_q * sa - c_lift_q * ca) * 0.5 / airspeed * c * q + (-c_drag_input_y * sa - c_lift_input_y * ca) * input_y);
    }
    std::cout<<"body_FRD_lift:"<<aero_wrenches.lift<<std::endl;

}

void StdLinearAero::calcTorques()
{
    float rho=model->airdata.density;
    double qbar = 0.5 * rho * pow(airspeed, 2) * s;

    double ca = cos(alpha);
    double sa = sin(alpha);

    float p = airfoil_states.p;
    float q = airfoil_states.q;
    float r = airfoil_states.r;
    // Transform relative angular speed from airfoil_FLU to airfoil_FRD for calculations
    FLUtoFRD(p, q, r);
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
        aero_wrenches.l = qbar * (b * (c_l_0 + c_l_b * beta + c_l_p * b / 2 / airspeed * p + c_l_r * b / 2 / airspeed * r + c_l_input_x * input_x + c_l_input_z * input_z));
        aero_wrenches.m = qbar * (c * (c_m_0 + c_m_a * alpha + c_m_q * c / 2 / airspeed * q + c_m_input_y * input_y));
        aero_wrenches.n = qbar * (b * (c_n_0 + c_n_b * beta + c_n_p * b / 2 / airspeed * p + c_n_r * b / 2 / airspeed * r + c_n_input_x * input_x + c_n_input_z * input_z));
    }
    std::cout<<"body_FRD_m:"<<aero_wrenches.m<<std::endl;

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
    sprintf(paramMsg, "airfoil%i/c_drag_input_y", id);
    if (!ros::param::getCached(paramMsg, c_drag_input_y)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_drag_q", id);
    if (!ros::param::getCached(paramMsg, c_drag_q)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_lift_0", id);
    if (!ros::param::getCached(paramMsg, c_lift_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_lift_input_y", id);
    if (!ros::param::getCached(paramMsg, c_lift_input_y)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
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
    sprintf(paramMsg, "airfoil%i/c_y_input_x", id);
    if (!ros::param::getCached(paramMsg, c_y_input_x)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_y_input_z", id);
    if (!ros::param::getCached(paramMsg, c_y_input_z)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_0", id);
    if (!ros::param::getCached(paramMsg, c_l_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_p", id);
    if (!ros::param::getCached(paramMsg, c_l_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_b", id);
    if (!ros::param::getCached(paramMsg, c_l_b)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_r", id);
    if (!ros::param::getCached(paramMsg, c_l_r)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_input_x", id);
    if (!ros::param::getCached(paramMsg, c_l_input_x)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_l_input_z", id);
    if (!ros::param::getCached(paramMsg, c_l_input_z)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_0", id);
    if (!ros::param::getCached(paramMsg, c_m_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_a", id);
    if (!ros::param::getCached(paramMsg, c_m_a)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_q", id);
    if (!ros::param::getCached(paramMsg, c_m_q)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_m_input_y", id);
    if (!ros::param::getCached(paramMsg, c_m_input_y)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_0", id);
    if (!ros::param::getCached(paramMsg, c_n_0)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_b", id);
    if (!ros::param::getCached(paramMsg, c_n_b)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_p", id);
    if (!ros::param::getCached(paramMsg, c_n_p)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_r", id);
    if (!ros::param::getCached(paramMsg, c_n_r)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_input_x", id);
    if (!ros::param::getCached(paramMsg, c_n_input_x)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/c_n_input_z", id);
    if (!ros::param::getCached(paramMsg, c_n_input_z)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/oswald", id);
    if (!ros::param::getCached(paramMsg, oswald)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    sprintf(paramMsg, "airfoil%i/mcoeff", id);
    if (!ros::param::getCached(paramMsg, M)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
}