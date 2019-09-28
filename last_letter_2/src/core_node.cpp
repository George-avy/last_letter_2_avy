#include "model.hpp"
#include "model.cpp"
#include "factory.cpp"
#include "environment.cpp"
#include "dynamics.cpp"
#include "aerodynamics/aerodynamics.cpp"
#include "propulsion/propulsion.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core_node");
    ros::NodeHandle nh;

    Model model; //Create a Model object

    // Start gazebo on pause state
    // Freeze gazebo with service after the initialization of all classes, services and topics
    // Otherwise it hangs
    ros::service::waitForService("/gazebo/pause_physics");

    ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty emptySrv;
    pauseGazebo.call(emptySrv);

    //Start spin
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    ros::shutdown();
}
