#include "model.hpp"
#include "model.cpp"
#include "environment.cpp"
#include "dynamics.cpp"
#include "aerodynamics.cpp"
#include "propulsion/propulsion.cpp"
#include "factory.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core");
    //Create a Model object
    Model model;
    //Start spin
    while (ros::ok())
    {
        ros::AsyncSpinner spinner(1); // Use 1 threads
        spinner.start();
        ros::waitForShutdown();
    }
    ros::shutdown();
}
