#include "noEngine.cpp"
#include "beardEngine.cpp"
#include "electricEngine.cpp"

Propulsion::Propulsion(Model *parent, int id) :tfListener(tfBuffer)
{
    model = parent;
    motor_number = id; //Motor ID number
}

//calculation steps
void Propulsion::calculationCycle()
{
    getStates();
    getInputSignals();
    rotateWind();
    calcAirspeed();
    calcWrench();
}

//get Link States from model plugin
void Propulsion::getStates()
{
    motor_states=model->model_states.motor_states[motor_number-1];
}

// load motor input from model
void Propulsion::getInputSignals()
{
    motor_input=model->motor_input[motor_number-1];
}

// rotate wind vector from body to motor Links
void Propulsion::rotateWind()
{
    char name_temp[20];
    std::string motor_link_name;

    sprintf(name_temp, "motor%i", motor_number);
    motor_link_name.assign(name_temp);

    v_in.header.frame_id = model->airdata.header.frame_id;
    v_in.vector.x = model->airdata.wind_x;
    v_in.vector.y = model->airdata.wind_y;
    v_in.vector.z = model->airdata.wind_z;

    try
    {
        v_out = tfBuffer.transform(v_in, motor_link_name);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform body_FLU to motor: %s", ex.what());
    }

    //wind expressed on motor frame
    relative_wind.x=v_out.vector.x;
    relative_wind.y=v_out.vector.y;
    relative_wind.z=v_out.vector.z;
}

//calculate relative airspeed
void Propulsion::calcAirspeed()
{
    // airspeed relative to motor
    float u_r = motor_states.u - relative_wind.x;
    float v_r = motor_states.v - relative_wind.y;
    float w_r = motor_states.w - relative_wind.z;
    airspeed = sqrt(pow(u_r, 2) + pow(v_r, 2) + pow(w_r, 2));
}

void Propulsion::calcWrench()
{
    calcThrust();
    calcTorque();
}