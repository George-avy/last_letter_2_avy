

Model::Model() : environment(this), dynamics(this)
{
    //Read the number of airfoils
    if (!ros::param::getCached("airfoil/nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
    //Read the number of motors
    if (!ros::param::getCached("motor/nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }

    //Subscriber that gets model states from gazebo after physics step
    gazebo_sub = nh.subscribe("last_letter_2/gazebo/model_states", 1, &Model::gazeboStatesClb, this,ros::TransportHints().tcpNoDelay());

    //Service to send the calculated wrenches to gazebo
    ros::service::waitForService("last_letter_2/apply_model_wrenches_srv");
    apply_wrench_client = nh.serviceClient<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv", true);
    ros::service::waitForService("last_letter_2/get_control_inputs_srv");
    get_control_inputs_client = nh.serviceClient<last_letter_2_msgs::get_control_inputs_srv>("last_letter_2/get_control_inputs_srv", true);
    
    // Start gazebo on pause state
    // Freeze gazebo with service after the initialization of all gazebo classes, services and topics.
    // Otherwise there is a problem.
    ros::service::waitForService("/gazebo/pause_physics"); //pause gazebo to succeed synchronization with ros
    pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty emptySrv;
    pauseGazebo.call(emptySrv);
     t=ros::WallTime::now();
}

void Model::gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr& msg)
{
    //Get/store model states from gazebo and do the calculation step
    model_states.base_link_states=msg->base_link_states;
    model_states.airfoil_states=msg->airfoil_states;
    model_states.motor_states=msg->motor_states;

    loop_num.data = msg->loop_number.data;
    // printf("ros   : %i at ", loop_num.data);
    // std::cout<< ros::WallTime::now()<< "start"<< std::endl;
    // //the states exported from gazebo are expressed in NWU frame, but the calculations are based on NED frame
    // //so convert gazebo data from NWU to NED to continue with calculations
    // NWUtoNED(base_link_states.x, base_link_states.y, base_link_states.z);
    // NWUtoNED(base_link_states.roll, base_link_states.pitch, base_link_states.yaw);
    // NWUtoNED(base_link_states.u, base_link_states.v, base_link_states.w);
    // NWUtoNED(base_link_states.p, base_link_states.q, base_link_states.r);
    // // printf("x=%f    y=%f    z=%f\n",base_link_states.x, base_link_states.y,base_link_states.z);

    modelStep();
}

void Model::modelStep()
{
    // std::cout<<(ros::WallTime::now()-t)<<std::endl;
    // t=ros::WallTime::now();
    // std::cout<< "01=    "<< ros::WallTime::now()<<std::endl;
    getControlInputs();
    // std::cout<< "02=    "<< ros::WallTime::now()<<std::endl;
    getAirdata();
    // std::cout<< "03=    "<< ros::WallTime::now()<<std::endl;
    calcDynamics();
    // std::cout<< "04=    "<< ros::WallTime::now()<<std::endl;
    applyWrenches();
    // std::cout<< "05=    "<< ros::WallTime::now()<<std::endl<<std::endl;
}

// get control inputs for all airfoils and motor from controller node
void Model::getControlInputs()
{
    // call get_contol_inputs_srv
    if (get_control_inputs_client.isValid())
    {
        if (get_control_inputs_client.call(control_inputs_msg))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service get_contol_inputs_srv\n");
        }
    }
    else
    {
        ROS_ERROR("Service apply_wrench down, waiting reconnection...");
        get_control_inputs_client.waitForExistence();
        //    connectToClient(); //Why this??
    }

    //store airfoil inputs
    for (i = 0; i < num_wings; i++)
    {
        airfoil_inputs[i].x=control_inputs_msg.response.airfoil_inputs[i].x;
        airfoil_inputs[i].y=control_inputs_msg.response.airfoil_inputs[i].y;
        airfoil_inputs[i].z=control_inputs_msg.response.airfoil_inputs[i].z;
    }

    //store motor inputs
    for (i = 0; i < num_motors; i++)
    {
        motor_input[i]=control_inputs_msg.response.motor_input[i];
    }

}

void Model::getAirdata()
{
    environment.calculateAirdata();

    airdata.header.frame_id = "body_FLU";
    airdata.header.stamp = ros::Time::now();
    airdata.wind_x = environment.airdata.wind_x;
    airdata.wind_y = environment.airdata.wind_y;
    airdata.wind_z = environment.airdata.wind_z;
    airdata.density = environment.airdata.density;
    airdata.temperature = environment.airdata.temperature;
}

void Model::calcDynamics()
{
    // std::cout<< "03.1=  "<< ros::WallTime::now()<<std::endl;
    dynamics.calcAero();
    // std::cout<< "03.2=  "<< ros::WallTime::now()<<std::endl;
    dynamics.calcProp();
    // std::cout<< "03.3=  "<< ros::WallTime::now()<<std::endl;
}

void Model::applyWrenches()
{
    int i = 0;
    std::list<Aerodynamics *>::iterator itAero = dynamics.listOfAerodynamics.begin();
    for (itAero = dynamics.listOfAerodynamics.begin(); itAero != dynamics.listOfAerodynamics.end(); ++itAero)
    {
        apply_wrenches_srv.request.airfoil_forces[i].x = (*itAero)->aero_wrenches.drag;
        apply_wrenches_srv.request.airfoil_forces[i].y = -(*itAero)->aero_wrenches.fy;
        apply_wrenches_srv.request.airfoil_forces[i].z = -(*itAero)->aero_wrenches.lift;
        apply_wrenches_srv.request.airfoil_torques[i].x = (*itAero)->aero_wrenches.l;
        apply_wrenches_srv.request.airfoil_torques[i].y = -(*itAero)->aero_wrenches.m;
        apply_wrenches_srv.request.airfoil_torques[i].z = -(*itAero)->aero_wrenches.n;

        // NEDtoNWU(apply_wrenches_srv.request.airfoil_forces[i].x,apply_wrenches_srv.request.airfoil_forces[i].y,apply_wrenches_srv.request.airfoil_forces[i].z);
        // NEDtoNWU(apply_wrenches_srv.request.airfoil_torques[i].x,apply_wrenches_srv.request.airfoil_torques[i].y,apply_wrenches_srv.request.airfoil_torques[i].z);
        i++;
    }
    // std::cout<< "04.1=  "<< ros::WallTime::now()<<std::endl;

    i=0;
    std::list<Propulsion *>::iterator itProp = dynamics.listOfPropulsion.begin();
    for (itProp = dynamics.listOfPropulsion.begin(); itProp != dynamics.listOfPropulsion.end(); ++itProp)
    {
        apply_wrenches_srv.request.motor_thrust[i] = (*itProp)->prop_wrenches.thrust;
        apply_wrenches_srv.request.motor_torque[i] = (*itProp)->prop_wrenches.torque;
        i++;
    }
    // std::cout<< "04.2=  "<< ros::WallTime::now()<<std::endl;

    // call apply_model_wrenches_srv
    if (apply_wrench_client.isValid())
    {
        if (apply_wrench_client.call(apply_wrenches_srv))
        {
            // ROS_INFO("succeed service call\n");
        }
        else
        {
            ROS_ERROR("Failed to call service apply_model_wrenches_srv\n");
        }
    }
    else
    {
        ROS_ERROR("Service apply_wrench down, waiting reconnection...");
        apply_wrench_client.waitForExistence();
        //    connectToClient(); //Why this??
    }
    // std::cout<< "04.3=  "<< ros::WallTime::now()<<std::endl;

}