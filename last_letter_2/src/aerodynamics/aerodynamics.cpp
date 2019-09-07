#include "noAerodynamics.cpp"
#include "stdLinearAero.cpp"
#include "polyAero.cpp"

//Constructor
Aerodynamics::Aerodynamics(Model *parent, int id) :tfListener(tfBuffer)
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

    //wind expressed on airfoil frame
    relative_wind.x=v_out.vector.x;
    relative_wind.y=v_out.vector.y;
    relative_wind.z=v_out.vector.z;
}

//calculate essential values (airspeed, alpha, beta)
void Aerodynamics::calcTriplet()
{
    // airspeed, alpha, beta relative to airfoil
    float u_r = wing_states.u - relative_wind.x;    //relative speeds
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
