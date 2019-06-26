

#include <ros/ros.h>
#include <last_letter_2_msgs/joystick_input.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include<fstream> 

ros::ServiceClient pauseGazebo;
ros::ServiceClient resetSimulation;
ros::ServiceClient spawnModel;
ros::ServiceClient deleteModel;
std_srvs::Empty emptySrv;
gazebo_msgs::SpawnModel spawn_model;
gazebo_msgs::DeleteModel delete_model;
char name_temp[30];
std::string spawn_model_name;
int i,j,prev_j;

void srvServer(last_letter_2_msgs::joystick_input channels)
{
    if (channels.value[17] == 2000)
    {
        resetSimulation.call(emptySrv);
    }
    if (channels.value[6] == 2000)
    {
        sprintf(name_temp, "can%i", i++);
        spawn_model_name.assign(name_temp);
        spawn_model.request.model_name = spawn_model_name;
        spawn_model.request.initial_pose.position.z=-0.2;
        spawn_model.request.reference_frame="plane";
        spawnModel.call(spawn_model);
    }
    if (channels.value[7] == 2000)
    {
        for (j = prev_j; j < i; j++)
        {
            sprintf(name_temp, "can%i", j);
            spawn_model_name.assign(name_temp);
            delete_model.request.model_name = spawn_model_name;
            std::cout<<delete_model.request.model_name <<std::endl;
            deleteModel.call(delete_model);
        }
        prev_j=j;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "buttonFuncions_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("last_letter_2/rawPWM", 1, srvServer, ros::TransportHints().tcpNoDelay());

    //Init Services
    ros::service::waitForService("/gazebo/pause_physics"); //pause gazebo
    pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::service::waitForService("/gazebo/reset_simulation"); //restart simulation
    resetSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    ros::service::waitForService("/gazebo/spawn_urdf_model"); //spanw a model in gazebo world
    spawnModel = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::service::waitForService("/gazebo/delete_model"); //delete a model from gazebo world
    deleteModel = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    if (!ros::param::getCached("can", spawn_model.request.model_xml))
    {
        ROS_INFO("No model for spawning selected");
    }

    i=0;
    j=0;
    prev_j=0;
    // Enter spin
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}
