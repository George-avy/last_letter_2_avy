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

    Model model; //Create a Model object

    //Start spin
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    ros::shutdown();
}
