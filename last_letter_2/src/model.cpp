// The Model object
// A class that coordinates all processes for a model step

Model::Model() : environment(this), dynamics(this)
{
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }
    //Read the initial state of gazebo
    if (!ros::param::getCached("updatePhysics/paused", start_paused)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }

    char paramMsg[50];

    //Load needed characteristics for each airfoil
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

    //Load needed characteristics for each motor
    for (i = 0; i < num_motors; ++i)
    {
        sprintf(paramMsg, "motor%i/input_chan", i + 1);
        if (!ros::param::getCached(paramMsg, motor_in_chan[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    }

    //Subscriber
    gazebo_sub = nh.subscribe("last_letter_2/model_states", 1, &Model::gazeboStatesClb, this, ros::TransportHints().tcpNoDelay());

    //Services
    ros::service::waitForService("last_letter_2/apply_model_wrenches_srv");
    apply_wrench_client = nh.serviceClient<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv", true);
    ros::service::waitForService("last_letter_2/get_control_inputs_srv");
    get_control_inputs_client = nh.serviceClient<last_letter_2_msgs::get_control_inputs_srv>("last_letter_2/get_control_inputs_srv", true);

    // Start gazebo on pause state
    // Freeze gazebo with service after create the Model object
    // Otherwise it hangs
    ros::service::waitForService("/gazebo/pause_physics");
    ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty emptySrv;
    if (start_paused == 1)
    {
        pauseGazebo.call(emptySrv);
    }

    // Init variables
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

// Callback that listens to /channels topic
// This callback start the dynamics calculation step in ROS
void Model::gazeboStatesClb(const last_letter_2_msgs::model_states::ConstPtr &msg)
{
    model_states.header = msg->header;
    model_states.base_link_states = msg->base_link_states;
    model_states.airfoil_states = msg->airfoil_states;
    model_states.motor_states = msg->motor_states;
    modelStep();
}

void Model::modelStep()
{
    getControlInputs();
    getAirdata();
    calcDynamics();
    applyWrenches();
}

// Get model control inputs from Controller_node
void Model::getControlInputs()
{
    control_inputs_msg.request.header = model_states.header;
    // call get_contol_inputs_srv
    if (get_control_inputs_client.isValid())
    {
        if (get_control_inputs_client.call(control_inputs_msg)) {}
        else
        {
            ROS_ERROR("Failed to call service get_control_inputs_srv\n");
        }
    }
    else
    {
        ROS_ERROR("Service get_control_inputs down, waiting reconnection...");
        ros::service::waitForService("last_letter_2/get_control_inputs_srv");
        get_control_inputs_client = nh.serviceClient<last_letter_2_msgs::get_control_inputs_srv>("last_letter_2/get_control_inputs_srv", true);
    }

    // Store airfoil inputs
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

    // Store motor inputs
    for (i = 0; i < num_motors; i++)
    {
        motor_input[i] = control_inputs_msg.response.channels[motor_in_chan[i]];
    }
}

// Call enviroment class to calculate and send new data
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

    // Rotate wind vector from inertial_NWU frame to body_FLU
    transformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX(model_states.base_link_states.psi,
                                                               model_states.base_link_states.theta,
                                                               model_states.base_link_states.phi),
                                       KDL::Vector(0, 0, 0));
    v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(airdata.wind_x, airdata.wind_y, airdata.wind_z), ros::Time::now(), "body_FLU");

    body_wind.x = v_out[0];
    body_wind.y = v_out[1];
    body_wind.z = v_out[2];
}

void Model::calcDynamics()
{
    dynamics.calcAero();
    dynamics.calcProp();
}

// Send wrenches to gazebo to apply them
void Model::applyWrenches()
{
    i = 0;
    std::list<Aerodynamics *>::iterator itAero = dynamics.listOfAerodynamics.begin();
    for (itAero = dynamics.listOfAerodynamics.begin(); itAero != dynamics.listOfAerodynamics.end(); ++itAero)
    {
        // Gazebo default links frame is FLU, but forces and torques are calculated in FRD
        // Convert wrenches from FRD to FLU before send them to be applied
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

    i = 0;
    std::list<Propulsion *>::iterator itProp = dynamics.listOfPropulsion.begin();
    for (itProp = dynamics.listOfPropulsion.begin(); itProp != dynamics.listOfPropulsion.end(); ++itProp)
    {
        apply_wrenches_srv.request.motor_thrust[i] = (*itProp)->prop_wrenches.thrust;
        apply_wrenches_srv.request.motor_torque[i] = (*itProp)->prop_wrenches.torque;
        apply_wrenches_srv.request.motor_omega[i] = (*itProp)->prop_wrenches.omega;
        i++;
    }
    // Call apply_model_wrenches_srv
    if (apply_wrench_client.isValid())
    {
        if (apply_wrench_client.call(apply_wrenches_srv)) {}
        else
        {
            ROS_ERROR("Failed to call service apply_model_wrenches_srv\n");
        }
    }
    else
    {
        ROS_ERROR("Service apply_wrench down, waiting reconnection...");
        ros::service::waitForService("last_letter_2/apply_model_wrenches_srv");
        apply_wrench_client = nh.serviceClient<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv", true);
    }
}