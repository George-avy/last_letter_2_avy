
Model::Model() : environment(this), dynamics(this)
{
    //set dynamic matrices dimensions based on simulated multirotor
    //if a plane is simulated, skip this.
    //matrices for quadrotor:(4,4), hexarotor:(4,6)
    multirotor_matrix.resize(4, 4);
    multirotor_matrix_inverse.resize(4, 4);
    // Vectors for quadrotor (4), hexarotor (6)
    commands.resize(4);
    input_signal_vector.resize(4);

    // Read the mixer type
    if (!ros::param::getCached("HID/mixerid", mixerid)) { ROS_INFO("No mixing function selected"); mixerid = 0;}
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }
    
    char paramMsg[50];

    switch (mixerid)
    {
    case 0: // No mixing applied
        break;
    case 1: // Airplane mixing

        //Load basic characteristics for each airfoil
        for (i = 0; i < num_wings; ++i)
        {
            sprintf(paramMsg, "airfoil%i/x_axis_turn_chan", i + 1);
            if (!ros::param::getCached(paramMsg, x_axis_turn_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/y_axis_turn_chan", i + 1);
            if (!ros::param::getCached(paramMsg, y_axis_turn_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/z_axis_turn_chan", i + 1);
            if (!ros::param::getCached(paramMsg, z_axis_turn_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/deltax_max", i + 1);
            if (!ros::param::getCached(paramMsg, deltax_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/deltay_max", i + 1);
            if (!ros::param::getCached(paramMsg, deltay_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/deltaz_max", i + 1);
            if (!ros::param::getCached(paramMsg, deltaz_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        }
        break;
    case 2:
        initMultirotorMatrix();
        break;
    default:
        ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
        ros::shutdown();
        break;
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
}

void Model::gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr& msg)
{
    //Get/store model states from gazebo and do the calculation step
    model_states.header=msg->header;
    model_states.base_link_states=msg->base_link_states;
    model_states.airfoil_states=msg->airfoil_states;
    model_states.motor_states=msg->motor_states;
    
    loop_num.data = msg->loop_number.data;
    //so convert gazebo data from FLU to FRD to continue with calculations
    FLUtoFRD(model_states.base_link_states.x, model_states.base_link_states.y, model_states.base_link_states.z);
    FLUtoFRD(model_states.base_link_states.roll, model_states.base_link_states.pitch, model_states.base_link_states.yaw);
    FLUtoFRD(model_states.base_link_states.u, model_states.base_link_states.v, model_states.base_link_states.w);
    FLUtoFRD(model_states.base_link_states.p, model_states.base_link_states.q, model_states.base_link_states.r);
    
    for (i = 0; i < num_wings; i++)
    {
        FLUtoFRD(model_states.airfoil_states[i].x, model_states.airfoil_states[i].y, model_states.airfoil_states[i].z);
        FLUtoFRD(model_states.airfoil_states[i].roll, model_states.airfoil_states[i].pitch, model_states.airfoil_states[i].yaw);
        FLUtoFRD(model_states.airfoil_states[i].u, model_states.airfoil_states[i].v, model_states.airfoil_states[i].w);
        FLUtoFRD(model_states.airfoil_states[i].p, model_states.airfoil_states[i].q, model_states.airfoil_states[i].r);
    }

    for (i = 0; i < num_motors; i++)
    {
        FLUtoFRD(model_states.motor_states[i].x, model_states.motor_states[i].y, model_states.motor_states[i].z);
        FLUtoFRD(model_states.motor_states[i].roll, model_states.motor_states[i].pitch, model_states.motor_states[i].yaw);
        FLUtoFRD(model_states.motor_states[i].u, model_states.motor_states[i].v, model_states.motor_states[i].w);
        FLUtoFRD(model_states.motor_states[i].p, model_states.motor_states[i].q, model_states.motor_states[i].r);
    }
    modelStep();
}

void Model::initMultirotorMatrix()
{
    //init multirotor matrix, that finds roll, pitch, yaw, thrust, based on motor inputs
    float k1 = 0.25;
    float l = 10;
    float k2 = 0.6;
    multirotor_matrix << k1, k1, k1, k1,
                        0, -l * k1, 0, l * k1,
                        l * k1, 0, -l * k1, 0,
                        -k2, k2, -k2, k2;
     //calculate inverse of multirotor matrix. Usefull for future calculations
     multirotor_matrix_inverse = multirotor_matrix.completeOrthogonalDecomposition().pseudoInverse();
}

void Model::modelStep()
{
    getControlInputs();
    getAirdata();
    calcDynamics();
    applyWrenches();
}

// get control inputs for the model from controller node
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
        airfoil_inputs[i].x = deltax_max[i] * control_inputs_msg.response.input_signals[x_axis_turn_chan[i]];
        airfoil_inputs[i].y = deltay_max[i] * control_inputs_msg.response.input_signals[y_axis_turn_chan[i]];
        airfoil_inputs[i].z = deltaz_max[i] * control_inputs_msg.response.input_signals[z_axis_turn_chan[i]];
    }

    switch (mixerid)
    {
    case 0: // No mixing applied
        break;
    case 1: // Airplane mixing
        //store motor inputs
        for (i = 0; i < num_motors; i++)
        {
            motor_input[i] = control_inputs_msg.response.input_signals[4];
        }
        break;
    case 2: // Multirotor mixing
        commands(0) = control_inputs_msg.response.input_signals[4]; //thrust
        commands(1) = control_inputs_msg.response.input_signals[1]; //roll
        commands(2) = control_inputs_msg.response.input_signals[2]; //pitch
        commands(3) = control_inputs_msg.response.input_signals[3]; //yaw
        input_signal_vector = multirotor_matrix_inverse * commands;
        for (i = 0; i < num_motors; i++)
        {
            motor_input[i] = input_signal_vector(i);
            if (motor_input[i] < 0)
                motor_input[i] = 0;
        }
        break;
    default:
        ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
        ros::shutdown();
        break;
    }

    //store button inputs
    for (i = 4; i < 20; i++)
    {
        button_input[i - 4] = control_inputs_msg.response.button_signals[i];
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
    dynamics.calcAero();
    dynamics.calcProp();
}

void Model::applyWrenches()
{
    int i = 0;
    std::list<Aerodynamics *>::iterator itAero = dynamics.listOfAerodynamics.begin();
    for (itAero = dynamics.listOfAerodynamics.begin(); itAero != dynamics.listOfAerodynamics.end(); ++itAero)
    {
        //gazebo default link frames are FLU, but forces and torques are expressed in FRD
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