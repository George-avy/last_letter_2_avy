#include "noAerodynamics.cpp"
#include "stdLinearAero.cpp"
#include "polyAero.cpp"

//Constructor
Aerodynamics::Aerodynamics(Model *parent, int id)
{
    model = parent;
    airfoil_number = id;   //airfoil ID number
}

//calculation steps
void Aerodynamics::calculationCycle()
{
    getStates();
    getInputSignals();
    rotateWind();
    calcTriplet();
    calcWrench();
}

//get Link States from model plugin
void Aerodynamics::getStates()
{
    airfoil_states=model->model_states.airfoil_states[airfoil_number-1];
}

// load airfoil input signals from model
void Aerodynamics::getInputSignals()
{
    airfoil_inputs.x = model->airfoil_input[airfoil_number - 1].x;
    airfoil_inputs.y = model->airfoil_input[airfoil_number - 1].y;
    airfoil_inputs.z = model->airfoil_input[airfoil_number - 1].z;
}

// rotate wind vector from body to airfoil Link
void Aerodynamics::rotateWind()
{
    char name_temp[20];
    std::string airfoil_link_name;

    sprintf(name_temp, "airfoil%i", airfoil_number);
    airfoil_link_name.assign(name_temp);

    // Transform wing vector from inertial_NWU to airfoil_FLU frame
    transformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX( airfoil_states.yaw,
                                                                airfoil_states.pitch,
                                                                airfoil_states.roll),
                                                                KDL::Vector(0, 0, 0));
    v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(model->airdata.wind_x,model->airdata.wind_y,model->airdata.wind_z), ros::Time::now(), "airfoil_FLU");

    relative_wind.x=v_out[0];
    relative_wind.y=v_out[1];
    relative_wind.z=v_out[2];

    // std::cout<<"wind airfoil"<<airfoil_number<<std::endl;
    // std::cout<<relative_wind<<std::endl<<std::endl;
}

//calculate essential values (airspeed, alpha, beta)
void Aerodynamics::calcTriplet()
{
    // airspeed, alpha, beta relative to airfoil
    float u_r = airfoil_states.u - relative_wind.x;    //relative speeds
    float v_r = airfoil_states.v - relative_wind.y;
    float w_r = airfoil_states.w - relative_wind.z;
    // Transform relative speed from airfoil_FLU to airfoil_FRD for calculations
    FLUtoFRD(u_r, v_r, w_r);
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
