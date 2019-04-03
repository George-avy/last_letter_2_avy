

//Constructor
Aerodynamics::Aerodynamics(Model *parent, int id) :tfListener(tfBuffer)
{
    model = parent;
    //airfoil ID number
    airfoil_number = id;
}

//calculation steps
void Aerodynamics::calculationCycle()
{
    // std::cout<< "03.1.1="<< ros::WallTime::now()<<std::endl;
    getStates();
    // std::cout<< "03.1.2="<< ros::WallTime::now()<<std::endl;
    getInputSignals();
    // std::cout<< "03.1.3="<< ros::WallTime::now()<<std::endl;
    rotateWind();
    // std::cout<< "03.1.4="<< ros::WallTime::now()<<std::endl;
    calcTriplet();
    // std::cout<< "03.1.5="<< ros::WallTime::now()<<std::endl;
    calcWrench();
}

//get Link States from model plugin
void Aerodynamics::getStates()
{
    wing_states=model->model_states.airfoil_states[airfoil_number-1];
}

// load airfoil input signals from model
void Aerodynamics::getInputSignals()
{
    airfoil_inputs.x = model->airfoil_inputs[airfoil_number - 1].x;
    airfoil_inputs.y = model->airfoil_inputs[airfoil_number - 1].y;
    airfoil_inputs.z = model->airfoil_inputs[airfoil_number - 1].z;
}

// rotate wind vector from body to airfoil Link
void Aerodynamics::rotateWind()
{
    char name_temp[20];
    std::string airfoil_link_name;

    sprintf(name_temp, "airfoil%i", airfoil_number);
    airfoil_link_name.assign(name_temp);

    v_in.header.frame_id = model->airdata.header.frame_id;
    v_in.vector.x = model->airdata.wind_x;
    v_in.vector.y = model->airdata.wind_y;
    v_in.vector.z = model->airdata.wind_z;

    try
    {
        v_out = tfBuffer.transform(v_in, airfoil_link_name);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform body_FLU to airfoil: %s", ex.what());
    }

    relative_wind.x=v_out.vector.x;
    relative_wind.y=v_out.vector.y;
    relative_wind.z=v_out.vector.z;
}

//calculate essensial values
void Aerodynamics::calcTriplet()
{
    // airspeed, alpha, beta relative to airfoil
    float u_r = wing_states.u - relative_wind.x;
    float v_r = wing_states.v - relative_wind.y;
    float w_r = wing_states.w - relative_wind.z;
    airspeed = sqrt(pow(u_r, 2) + pow(v_r, 2) + pow(w_r, 2));
    alpha = atan2(w_r, u_r);
    if (u_r == 0)
    {
        if (v_r == 0)
        {
            beta = 0;
        }
        else
        {
            beta = asin(v_r / abs(v_r));
        }
    }
    else
    {
        beta = atan2(v_r, u_r);
    }
}

void Aerodynamics::calcWrench()
{
    calcForces();
    calcTorques();
}


// Class NoAero contructor
NoAerodynamics::NoAerodynamics(Model *parent, int id) : Aerodynamics(parent, id)
{   
    printf("wing:%i no aerodynamics built\n", id);
}

void NoAerodynamics::calcForces()
{
    aero_wrenches.drag = 0;
    aero_wrenches.lift = 0;
    aero_wrenches.fy = 0;
}

void NoAerodynamics::calcTorques()
{
    aero_wrenches.l = 0;
    aero_wrenches.m = 0;
    aero_wrenches.n = 0;
}

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
        aero_wrenches.lift = 0;
        aero_wrenches.fy = 0;
    }
    else
    {
        aero_wrenches.drag = qbar * ((-c_drag_alpha * ca + c_lift_alpha * sa) + (-c_drag_q * ca + c_lift_q * sa) * 0.5 / airspeed * c * q + (-c_drag_deltae * ca + c_lift_deltae * sa) * input_y);
        aero_wrenches.lift = qbar * ((-c_drag_alpha * sa - c_lift_alpha * ca) + (-c_drag_q * sa - c_lift_q * ca) * 0.5 / airspeed * c * q + (-c_drag_deltae * sa - c_lift_deltae * ca) * input_y);
        aero_wrenches.fy = qbar * (c_y_0 + c_y_b * beta + c_y_p * b / 2 / airspeed * p + c_y_r * b / 2 / airspeed * r + c_y_deltaa * input_x + c_y_deltar * input_z);
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

// Load from parameter the essensial values for calculations
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