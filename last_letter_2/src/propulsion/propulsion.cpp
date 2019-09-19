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

    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = "body_FLU";
    transformStamped_.child_frame_id = motor_link_name;
    transformStamped_.transform.translation.x = 0; //Need only rotation transform
    transformStamped_.transform.translation.y = 0;
    transformStamped_.transform.translation.z = 0;
    quat_.setRPY(model->model_states.motor_states[motor_number - 1].roll - model->model_states.base_link_states.roll,
                 model->model_states.motor_states[motor_number - 1].pitch - model->model_states.base_link_states.pitch,
                 model->model_states.motor_states[motor_number - 1].yaw - model->model_states.base_link_states.yaw);
    transformStamped_.transform.rotation.x = quat_.x();
    transformStamped_.transform.rotation.y = quat_.y();
    transformStamped_.transform.rotation.z = quat_.z();
    transformStamped_.transform.rotation.w = quat_.w();

    t_in(0) = model->airdata.wind_x;
    t_in(1) = model->airdata.wind_y;
    t_in(2) = model->airdata.wind_z;

    try
    {
        tf2::doTransform(t_in, t_out, transformStamped_);
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform body_FLU to motor%i: %s", motor_number, ex.what());
    }

    //wind expressed on motor frame
    relative_wind.x = t_out(0);
    relative_wind.y = t_out(1);
    relative_wind.z = t_out(2);
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
    calcOmega();
}