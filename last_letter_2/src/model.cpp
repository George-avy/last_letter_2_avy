
Model::Model() : aerodynamics(this), propulsion(this)
{
    ros::service::waitForService("last_letter_2/model_states");
    states_client = nh.serviceClient<last_letter_2_msgs::get_model_states_srv>("last_letter_2/model_states", true);
    ros::service::waitForService("last_letter_2/control_signals");
    control_signals_client = nh.serviceClient<last_letter_2_msgs::get_control_signals_srv>("last_letter_2/control_signals", true);
    ros::service::waitForService("last_letter_2/apply_wrench_srv");
    apply_wrench_client = nh.serviceClient<last_letter_2_msgs::apply_wrench_srv>("last_letter_2/apply_wrench_srv", true);
    ros::service::waitForService("last_letter_2/step");
    sim_step_client = nh.serviceClient<last_letter_2_msgs::apply_wrench_srv>("last_letter_2/step", true);
    ros::service::waitForService("last_letter_2/airdata");
    airdata_client=nh.serviceClient<last_letter_2_msgs::airdata_srv>("last_letter_2/airdata",true);
    // signals_publisher = nh.advertise<last_letter_2::Control_signals>("last_letter_2/control_signals_feedback", 1000);

    ros::service::waitForService("/gazebo/pause_physics");	 	//pause gazebo to succeed synchronization with ros
    pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty emptySrv;
    pauseGazebo.call(emptySrv);
}

void Model::modelStep()
{
    getStates();
    getControlSignals();   // need to create control_singlas_server
    getAirdata();
    calcWrenches();
    applyWrenches();
    simulationStep();
}

void Model::getStates()
{
    //call step states_srv
    std_msgs::Empty msg;
    states_srv.request.empty=msg;
    if (states_client.isValid())
    {
        if (states_client.call(states_srv))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service get_model_states_srv\n");
            //			break;
        }
    }
    else
    {
        ROS_ERROR("Service down, waiting reconnection...");
        states_client.waitForExistence();
        //		    connectToClient(); //Why this??
    }
    //get Positon, Rotation, Linear Vel, Angular Vel
    model_states.x = states_srv.response.model_states.x;
    model_states.y = states_srv.response.model_states.y;
    model_states.z = states_srv.response.model_states.z;
    model_states.roll = states_srv.response.model_states.roll;
    model_states.pitch = states_srv.response.model_states.pitch;
    model_states.yaw = states_srv.response.model_states.yaw;
    model_states.u = states_srv.response.model_states.u;
    model_states.v = states_srv.response.model_states.v;
    model_states.w = states_srv.response.model_states.w;
    model_states.p = states_srv.response.model_states.p;
    model_states.q = states_srv.response.model_states.q;
    model_states.r = states_srv.response.model_states.r;
    model_states.header.stamp=ros::Time::now();  // need fix
}

void Model::getControlSignals()
{
    //call get_control_signals_srv
    std_msgs::Empty msg;
    signals_srv.request.empty=msg;
    if (control_signals_client.isValid())
    {
        if (control_signals_client.call(signals_srv))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service model_signals_srv\n");
            //			break;
        }
    }
    else
    {
        ROS_ERROR("Service down, waiting reconnection...");
        control_signals_client.waitForExistence();
        //		    connectToClient(); //Why this??
    }
    //get delta_e, delta_r, delta_a, delta_t
    control_signals.delta_a=signals_srv.response.signals.delta_a;
    control_signals.delta_e=signals_srv.response.signals.delta_e;
    control_signals.delta_r=signals_srv.response.signals.delta_r;
    control_signals.delta_t=signals_srv.response.signals.delta_t;
    control_signals.header.stamp=ros::Time::now();  // need fix
    // this->signals_publisher.publish(control_signals);

}

void Model::getAirdata()
{
    air_data.request.states=model_states;
    if (airdata_client.isValid())
    {
        if (airdata_client.call(air_data))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service air_data\n");
            //			break;
        }
    }
    else
    {
        ROS_ERROR("Service down, waiting reconnection...");
        airdata_client.waitForExistence();
        //		    connectToClient(); //Why this??
    }
}

void Model::calcWrenches()
{
    aerodynamics.calcWrench();
    propulsion.calcWrench();


}

void Model::applyWrenches()
{
    model_wrenches.thrust = propulsion.prop_wrenches.thrust; // y,z values with opposite sign
    model_wrenches.forces[0] = aerodynamics.aero_wrenches.drag;
    model_wrenches.forces[1] = -aerodynamics.aero_wrenches.fy;
    model_wrenches.forces[2] = -aerodynamics.aero_wrenches.lift;
    model_wrenches.torques[0] = aerodynamics.aero_wrenches.l + propulsion.prop_wrenches.torque;
    model_wrenches.torques[1] = -aerodynamics.aero_wrenches.m;
    model_wrenches.torques[2] = -aerodynamics.aero_wrenches.n;

    // prepare service to call - - - - - - - - - -
    apply_wrench_srv.request.model_wrenches = model_wrenches;

    // call apply_wrench_srv
    if (apply_wrench_client.isValid())
    {
        if (apply_wrench_client.call(apply_wrench_srv))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service apply_wrench_srv\n");
        }
    }
    else
    {
        ROS_ERROR("Service down, waiting reconnection...");
        apply_wrench_client.waitForExistence();
        //		    connectToClient(); //Why this??
    }
}

void Model::simulationStep()
{
    // call apply_wrench_srv
    if (sim_step_client.isValid())
    {
        if (sim_step_client.call(apply_wrench_srv))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service apply_wrench_srv\n");
        }
    }
    else
    {
        ROS_ERROR("Service down, waiting reconnection...");
        sim_step_client.waitForExistence();
        //		    connectToClient(); //Why this??
    }
}