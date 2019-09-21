#include "noEngine.cpp"
#include "genericEngine.cpp"
#include "electricEngine.cpp"

Propulsion::Propulsion(Model *parent, int id)
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

    // Transform wing vector from body_FLU to motor_FLU frame
    transformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX( motor_states.yaw - model->model_states.base_link_states.yaw,
                                                                motor_states.pitch - model->model_states.base_link_states.pitch,
                                                                motor_states.roll - model->model_states.base_link_states.roll),
                                                                KDL::Vector(0, 0, 0));
    v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(model->body_wind.x,model->body_wind.y,model->body_wind.z), ros::Time::now(), "motor_FLU");

    relative_wind.x=v_out[0];
    relative_wind.y=v_out[1];
    relative_wind.z=v_out[2];

    std::cout<<"wind motor "<<motor_number<<" :"<<std::endl;
    std::cout<<relative_wind<<std::endl<<std::endl;
}

//calculate relative airspeed
void Propulsion::calcAirspeed()
{
    // airspeed relative to motor
    float u_r = motor_states.u - relative_wind.x;
    float v_r = motor_states.v - relative_wind.y;
    float w_r = motor_states.w - relative_wind.z;
    // Transform relative speed from airfoil_FLU to airfoil_FRD for calculations
    FLUtoFRD(u_r, v_r, w_r);
    normalWind = u_r;
    airspeed = sqrt(pow(u_r, 2) + pow(v_r, 2) + pow(w_r, 2));
}

void Propulsion::calcWrench()
{
    calcThrust();
    calcTorque();
    calcOmega();
}