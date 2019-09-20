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
    wing_states=model->model_states.airfoil_states[airfoil_number-1];
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

    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = "body_FLU";
    transformStamped_.child_frame_id = airfoil_link_name;
    transformStamped_.transform.translation.x = 0;      //Need only rotation transformation
    transformStamped_.transform.translation.y = 0;
    transformStamped_.transform.translation.z = 0;
    quat_.setRPY(model->model_states.airfoil_states[airfoil_number - 1].roll - model->model_states.base_link_states.roll,
                 model->model_states.airfoil_states[airfoil_number - 1].pitch - model->model_states.base_link_states.pitch,
                 model->model_states.airfoil_states[airfoil_number - 1].yaw - model->model_states.base_link_states.yaw);
    transformStamped_.transform.rotation.x = quat_.x();
    transformStamped_.transform.rotation.y = quat_.y();
    transformStamped_.transform.rotation.z = quat_.z();
    transformStamped_.transform.rotation.w = quat_.w();

    t_in(0) = model->body_wind.x;
    t_in(1) = model->body_wind.y;
    t_in(2) = model->body_wind.z;
    
    try
    {
        tf2::doTransform(t_in, t_out, transformStamped_);
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT body_FLU to airfoil%i: %s", airfoil_number, ex.what());
    }

    relative_wind.x=t_out(0);
    relative_wind.y=t_out(1);
    relative_wind.z=t_out(2);


    // std::cout<<"wind airfoil1"<<std::endl;
    // std::cout<<t_out<<std::endl<<std::endl;
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
