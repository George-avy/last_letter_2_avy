Model::Model() : environment(this), dynamics(this)
{

    // Read the type of model
    if (!ros::param::getCached("model/type", model_type)) { ROS_INFO("No model type selected"); model_type = 0;}
    // Read the type of handling 
    if (!ros::param::getCached("model/handling", handling)) { ROS_INFO("No mixing function selected"); handling = 0;}
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }


    char paramMsg[50];


    //Load basic characteristics for each airfoil
    for (i = 0; i < num_wings; ++i)
        {
        sprintf(paramMsg, "airfoil%i/input_x_chan", i + 1);
        if (!ros::param::getCached(paramMsg, airfoil_in_x_chan[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/input_y_chan", i + 1);
        if (!ros::param::getCached(paramMsg, airfoil_in_y_chan[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/input_z_chan", i + 1);
        if (!ros::param::getCached(paramMsg, airfoil_in_z_chan[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/deltax_max", i + 1);
        if (!ros::param::getCached(paramMsg, deltax_max[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/deltay_max", i + 1);
        if (!ros::param::getCached(paramMsg, deltay_max[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        sprintf(paramMsg, "airfoil%i/deltaz_max", i + 1);
        if (!ros::param::getCached(paramMsg, deltaz_max[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    }

    //Load basic characteristics for each motor
    for (i = 0; i < num_motors; ++i)
    {
        sprintf(paramMsg, "motor%i/input_chan", i + 1);
        if (!ros::param::getCached(paramMsg, motor_in_chan[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    }

    //Subscriber that gets model states from gazebo after physics step
    gazebo_sub = nh.subscribe("last_letter_2/model_states", 1, &Model::gazeboStatesClb, this,ros::TransportHints().tcpNoDelay());

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

    for (i = 0; i < num_wings; i++)
    {
        airfoil_input[i].x = 0;
        airfoil_input[i].y = 0;
        airfoil_input[i].z = 0;
    }
    for (i = 0; i < num_motors; i++)
    {
        motor_input[i] = 0;
    }
}

void Model::gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr& msg)
{
    //Get/store model states from gazebo and do the calculation step
    model_states.header=msg->header;
    model_states.base_link_states=msg->base_link_states;
    model_states.airfoil_states=msg->airfoil_states;
    model_states.motor_states=msg->motor_states;
    modelStep();
}

void Model::modelStep()
{
    getControlInputs();
    getAirdata();
    calcDynamics();
    applyWrenches();
}

// get control inputs for the model from controller_node
void Model::getControlInputs()
{
    control_inputs_msg.request.header=model_states.header;
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
        ROS_ERROR("Service get_control_inputs down, waiting reconnection...");
        ros::service::waitForService("last_letter_2/get_control_inputs_srv");
        get_control_inputs_client = nh.serviceClient<last_letter_2_msgs::get_control_inputs_srv>("last_letter_2/get_control_inputs_srv", true);
    }

    //store airfoil inputs
    for (i = 0; i < num_wings; i++)
    {
        if (airfoil_in_x_chan[i] != -1)
        {
            airfoil_input[i].x = deltax_max[i] * control_inputs_msg.response.channels[airfoil_in_x_chan[i]];

        }
        if (airfoil_in_y_chan[i] != -1)
        {
            airfoil_input[i].y = deltay_max[i] * control_inputs_msg.response.channels[airfoil_in_y_chan[i]];

        }
        if (airfoil_in_z_chan[i] != -1)
        {
            airfoil_input[i].z = deltaz_max[i] * control_inputs_msg.response.channels[airfoil_in_z_chan[i]];

        }
    }

    //store motor inputs
    for (i = 0; i < num_motors; i++)
    {
        motor_input[i] = control_inputs_msg.response.channels[motor_in_chan[i]];
    }
}

void Model::getAirdata()
{
    environment.calculateAirdata();

    airdata.header.frame_id = "inertial_NWU";
    airdata.header.stamp = ros::Time::now();
    airdata.wind_x = environment.airdata.wind_x;
    airdata.wind_y = environment.airdata.wind_y;
    airdata.wind_z = environment.airdata.wind_z;
    airdata.density = environment.airdata.density;
    airdata.pressure = environment.airdata.pressure;
    airdata.temperature = environment.airdata.temperature;

    // transformStamped_.header.stamp = ros::Time::now();
    // transformStamped_.header.frame_id = "inertial_NWU";
    // transformStamped_.child_frame_id = "body_FLU";
    // transformStamped_.transform.translation.x = 0; // We need only rotation transform
    // transformStamped_.transform.translation.y = 0;
    // transformStamped_.transform.translation.z = 0;
    // quat_.setRPY(0 - model_states.base_link_states.roll,
    //              0 - model_states.base_link_states.pitch,
    //              0 - model_states.base_link_states.yaw);
    // transformStamped_.transform.rotation.x = quat_.x();
    // transformStamped_.transform.rotation.y = quat_.y();
    // transformStamped_.transform.rotation.z = quat_.z();
    // transformStamped_.transform.rotation.w = quat_.w();

    // Transform wing vector from inertial_NWU frame to body_FLU
    transformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX( model_states.base_link_states.yaw,
                                                                model_states.base_link_states.pitch,
                                                                model_states.base_link_states.roll),
                                                                KDL::Vector(0, 0, 0));
    v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(airdata.wind_x, airdata.wind_y, airdata.wind_z), ros::Time::now(), "bodu_FLU");

    // t_in(0) = airdata.wind_x;
    // t_in(1) = airdata.wind_y;
    // t_in(2) = airdata.wind_z;

    // // std::cout<<"wind inertial_NWU"<<std::endl;
    // // std::cout<<t_in<<std::endl<<std::endl;

    // try
    // {
    //     tf2::doTransform(t_in, t_out, transformStamped_);
    // }
    // catch (const tf2::TransformException &ex)
    // {
    //     ROS_WARN("Could NOT transform inertial_NWU to body_FLU: %s" ,ex.what());
    // }

    body_wind.x = v_out[0];
    body_wind.y = v_out[1];
    body_wind.z = v_out[2];
     std::cout<<"wind body_FLU"<<std::endl;
    std::cout<<body_wind<<std::endl<<std::endl;
}

void Model::calcDynamics()
{
    dynamics.calcAero();
    dynamics.calcProp();
}

void Model::applyWrenches()
{
    int i = 0;
    std::list<Aerodynamics *>::iterator itAero = dynamics.listOfAerodynamics.begin();
    for (itAero = dynamics.listOfAerodynamics.begin(); itAero != dynamics.listOfAerodynamics.end(); ++itAero)
    {
        //gazebo default link frames are FLU, but forces and torques are calculated in FRD
        // so convert wrenches from FRD to FLU before apply them
        FRDtoFLU((*itAero)->aero_wrenches.drag, (*itAero)->aero_wrenches.fy, (*itAero)->aero_wrenches.lift);
        FRDtoFLU((*itAero)->aero_wrenches.l, (*itAero)->aero_wrenches.m, (*itAero)->aero_wrenches.n);

        apply_wrenches_srv.request.airfoil_forces[i].x = (*itAero)->aero_wrenches.drag;
        apply_wrenches_srv.request.airfoil_forces[i].y = (*itAero)->aero_wrenches.fy;
        apply_wrenches_srv.request.airfoil_forces[i].z = (*itAero)->aero_wrenches.lift;
        apply_wrenches_srv.request.airfoil_torques[i].x = (*itAero)->aero_wrenches.l;
        apply_wrenches_srv.request.airfoil_torques[i].y = (*itAero)->aero_wrenches.m;
        apply_wrenches_srv.request.airfoil_torques[i].z = (*itAero)->aero_wrenches.n;
        i++;
    }

    i=0;
    std::list<Propulsion *>::iterator itProp = dynamics.listOfPropulsion.begin();
    for (itProp = dynamics.listOfPropulsion.begin(); itProp != dynamics.listOfPropulsion.end(); ++itProp)
    {
        apply_wrenches_srv.request.motor_thrust[i] = (*itProp)->prop_wrenches.thrust;
        apply_wrenches_srv.request.motor_torque[i] = (*itProp)->prop_wrenches.torque;
        apply_wrenches_srv.request.motor_omega[i] = (*itProp)->prop_wrenches.omega;
        i++;
    }
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
        // apply_wrench_client.waitForExistence();
        ros::service::waitForService("last_letter_2/apply_model_wrenches_srv");
        apply_wrench_client = nh.serviceClient<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv", true);
    }
}