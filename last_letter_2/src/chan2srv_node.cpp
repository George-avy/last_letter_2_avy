// A node to call gazebo services (eg. spawn_model, delete_model)
// It's neccessary gazebo services to be called from node that runs independently and do not wait gazebo.
// In other case, it hangs

#include <ros/ros.h>
#include <last_letter_2_msgs/channels.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <std_srvs/Empty.h>

ros::ServiceClient spawnModel;
ros::ServiceClient deleteModel;
gazebo_msgs::SpawnModel spawn_model;
gazebo_msgs::DeleteModel delete_model;

std_srvs::Empty emptySrv;
std::string spawn_model_name;
char name_temp[30];
int i, j, prev_j, spawn_can_chan, despawn_cans_chan;

// Manage channel functions. Mainly for calling defalut gazebo services
void srvServer(last_letter_2_msgs::channels channels)
{
    //spawn a small cube under the multirotor
    if (channels.value[spawn_can_chan] == 1)
    {
        sprintf(name_temp, "can%i", i++);
        spawn_model_name.assign(name_temp);
        spawn_model.request.model_name = spawn_model_name;
        spawn_model.request.initial_pose.position.z = -0.2;
        spawn_model.request.reference_frame = "my_model";
        spawnModel.call(spawn_model);
    }

    //delete all cubes from world
    if (channels.value[despawn_cans_chan] == 1)
    {
        for (j = prev_j; j < i; j++)
        {
            sprintf(name_temp, "can%i", j);
            spawn_model_name.assign(name_temp);
            delete_model.request.model_name = spawn_model_name;
            std::cout << delete_model.request.model_name << " despawned" << std::endl;
            deleteModel.call(delete_model);
        }
        prev_j = j; //keep the number of the last cube
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chan2srv_node");

    ros::NodeHandle n;

    // Subscriber
    ros::Subscriber sub = n.subscribe("last_letter_2/channels", 1, srvServer, ros::TransportHints().tcpNoDelay());

    // Services
    ros::service::waitForService("/gazebo/spawn_urdf_model"); //spanw a model in gazebo world
    spawnModel = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::service::waitForService("/gazebo/delete_model"); //delete a model from gazebo world
    deleteModel = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    //Read the channel that spawn the can
    if (ros::param::getCached("channels/spawn_can_chan", spawn_can_chan)) { ROS_INFO("spawn_can_chan loaded"); }
   //Read the that delete all spawned cans
    if (ros::param::getCached("channels/despawn_cans_chan", despawn_cans_chan)) { ROS_INFO("despawn_cans_chan loaded"); }

    // get the urdf of can model from parameter server
    if (!ros::param::getCached("can", spawn_model.request.model_xml))
    {
        ROS_INFO("No model for spawning selected");
    }

    i = 0;
    j = 0;
    prev_j = 0;

    // start spin
    ros::spin();

    return 0;
}
