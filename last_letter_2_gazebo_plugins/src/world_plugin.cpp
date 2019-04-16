#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <ros/ros.h>
#include <gazebo/physics/World.hh>
#include <ignition/math/Vector3.hh>
#include "ros/callback_queue.h"
#include <boost/bind.hpp>
#include <rosgraph_msgs/Clock.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ctime> // for timer
#include <ros/service.h>
#include <thread>
#include <std_srvs/Empty.h>

namespace gazebo
{
class world_plugin : public WorldPlugin
{
  // Pointer to the World
private:
  physics::WorldPtr World;

  ///  A node use for ROS transport
  ros::NodeHandle *rosNode;

public:
  world_plugin() : WorldPlugin() //constructor
  {
  }

  void Load(physics::WorldPtr _World, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
  {                                                         //and after the World has been loaded.Νot be blocking.
    this->World = _World;
    ROS_INFO("world_plugin just started");

    this->rosNode = new ros::NodeHandle; //Create a ros node for transport
    while (!this->rosNode->ok())
    {
      ROS_INFO("Waiting for node to rise");
    }
    publishWorldStaticFrames();
  }

  // publish the relation between the world static frames Inertial_NWU-Inertial_NED 
  void publishWorldStaticFrames()
  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    tf2::Quaternion quat;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "inertial_NWU";
    static_transformStamped.child_frame_id = "inertial_NED";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    quat.setRPY(M_PI, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();

    static_broadcaster.sendTransform(static_transformStamped);
  }

};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(world_plugin)
} // namespace gazebo
