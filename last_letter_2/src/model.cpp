Model::Model() : environment(this), dynamics(this), tfListener(tfBuffer)
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

    switch (model_type)
    {
    case 1: // Airplane

        //Load basic characteristics for each airfoil
        for (i = 0; i < num_wings; ++i)
        {
            sprintf(paramMsg, "airfoil%i/input_x_chan", i + 1);
            if (!ros::param::getCached(paramMsg, input_x_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/input_y_chan", i + 1);
            if (!ros::param::getCached(paramMsg, input_y_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/input_z_chan", i + 1);
            if (!ros::param::getCached(paramMsg, input_z_chan[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/deltax_max", i + 1);
            if (!ros::param::getCached(paramMsg, deltax_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/deltay_max", i + 1);
            if (!ros::param::getCached(paramMsg, deltay_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
            sprintf(paramMsg, "airfoil%i/deltaz_max", i + 1);
            if (!ros::param::getCached(paramMsg, deltaz_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        }
        break;

    case 4: //quadcopter

        // set dynamic matrices/vectors dimensions
        multirotor_matrix.resize(4, 4);
        multirotor_matrix_inverse.resize(4, 4);
        commands.resize(4);
        input_signal_vector.resize(4);
        initMultirotorMatrix();
        break;

    case 6: // hexacopter

        // set dynamic matrices/vectors dimensions
        multirotor_matrix.resize(4, 6);
        multirotor_matrix_inverse.resize(6, 4);
        commands.resize(4);
        input_signal_vector.resize(6);
        initMultirotorMatrix();
        break;

    default:
        ROS_FATAL("Invalid parameter for -/model/type- in param server!");
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
    // Read the thrust constant
    if (!ros::param::getCached("model/b", b)) { ROS_FATAL("Invalid parameters for -model/b- in param server!"); ros::shutdown(); }
    // Read the motor distance to center of gravity
    if (!ros::param::getCached("model/l", l)) { ROS_FATAL("Invalid parameters for -model/l- in param server!"); ros::shutdown(); }
    // Read the drag factor
    if (!ros::param::getCached("model/d", d)) { ROS_FATAL("Invalid parameters for -model/d- in param server!"); ros::shutdown(); }
    
    switch (model_type)
    {
    case 4: //  quadcopter matrix
        multirotor_matrix <<b,     b,      b,      b,      //thrust row
                            0,    -l * b,  0,      l * b,  //roll row
                            l * b, 0,     -l * b,  0,      //pitch row
                           -d,     d,     -d,      d;      //yaw row
        break;

    case 6: //hexacopter matrix
        multirotor_matrix << b,    b,           b,          b,    b,          b,             //thrust row 
                             0,   -b*l*1.73/2, -b*l*1.73/2, 0,    b*l*1.73/2, b*l*1.73/2,   //roll row
                             b*l,  b*l/2,      -b*l/2,     -b*l, -b*l/2,      b*l/2,        //pitch row
                            -d,    d,          -d,          d,   -d,          d;            //yaw row
        break;
    }
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
        airfoil_inputs[i].x = deltax_max[i] * control_inputs_msg.response.input_signals[input_x_chan[i]];
        airfoil_inputs[i].y = deltay_max[i] * control_inputs_msg.response.input_signals[input_y_chan[i]];
        airfoil_inputs[i].z = deltaz_max[i] * control_inputs_msg.response.input_signals[input_z_chan[i]];
    }

    switch (model_type)
    {
    case 1: // Airplane 

        //store motor inputs
        for (i = 0; i < num_motors; i++)
        {
            motor_input[i] = control_inputs_msg.response.input_signals[4];
        }
        break;

    case 4: 
    case 6:// Multirotor mixing

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
        ROS_FATAL("Invalid parameter for -/model/model_type- in param server!");
        ros::shutdown();
        break;
    }

    //keep all channel matrix
    for (i = 0; i < 20; i++)
    {
        channel_input[i] = control_inputs_msg.response.channel_signals[i];
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

    //tranform airdata from inertial_NWU frame to body_FLU
    v_in.header.frame_id = airdata.header.frame_id;
    v_in.header.stamp = airdata.header.stamp;
    v_in.vector.x = airdata.wind_x;
    v_in.vector.y = airdata.wind_y;
    v_in.vector.z = airdata.wind_z;

    try
    {
        tfBuffer.transform(v_in, v_out, "body_FLU", ros::Time(0), "body_FLU");
    }
    catch (tf2::TransformException &ex)
    {
        // ROS_WARN("Could NOT transform inertial_NWU to body_FLU: %s", ex.what());
    }

    airdata.header.frame_id = v_out.header.frame_id;
    airdata.header.stamp = v_out.header.stamp;
    airdata.wind_x = v_out.vector.x;
    airdata.wind_y = v_out.vector.y;
    airdata.wind_z = v_out.vector.z;
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