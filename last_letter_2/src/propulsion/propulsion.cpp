#include "noEngine.cpp"
#include "beardEngine.cpp"
#include "electricEngine.cpp"

Propulsion::Propulsion(Model *parent, int id) :tfListener(tfBuffer)
{
    model = parent;
    //Motor ID number
    motor_number = id;

    //Init services
    ros::service::waitForService("last_letter_2/link_inputs");
    motor_input_client = nh.serviceClient<last_letter_2_msgs::get_link_inputs_srv>("last_letter_2/link_inputs", true);
}

//calculation steps
void Propulsion::calculationCycle()
{
    std::cout<< "03.2.1="<< ros::WallTime::now()<<std::endl;
    getStates();
    std::cout<< "03.2.2="<< ros::WallTime::now()<<std::endl;
    getInputSignals();
    std::cout<< "03.2.3="<< ros::WallTime::now()<<std::endl;
    rotateWind();
    std::cout<< "03.2.4="<< ros::WallTime::now()<<std::endl;
    calcAirspeed();
    std::cout<< "03.2.5="<< ros::WallTime::now()<<std::endl;
    calcWrench();
}

//get Link States from model plugin
void Propulsion::getStates()
{
    motor_states=model->model_states.motor_states[motor_number-1];
}

// get insput Signals from controller node
void Propulsion::getInputSignals()
{
     //call motor_inputs srv
    link_inputs_srv.request.motor_number=motor_number;
    link_inputs_srv.request.airfoil_number=0;

    if (motor_input_client.isValid())
    {
        if (motor_input_client.call(link_inputs_srv))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service get_link_inputs_srv\n");
            //break;
        }
    }
    else
    {
        ROS_ERROR("Service getInputSignals down, waiting reconnection...");
        motor_input_client.waitForExistence();
        motor_input_client = nh.serviceClient<last_letter_2_msgs::get_link_inputs_srv>("last_letter_2/link_inputs", true);
    }
    motor_input=link_inputs_srv.response.inputs.x;
}

// rotate wind vector from body to airfoil Link
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

    relative_wind.x=v_out.vector.x;
    relative_wind.y=v_out.vector.y;
    relative_wind.z=v_out.vector.z;
}

//calculate relative airspeed
void Propulsion::calcAirspeed()
{
    // airspeed, alpha, beta relative to airfoil
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