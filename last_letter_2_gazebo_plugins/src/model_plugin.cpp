#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <ros/service.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/link_states.h>
#include <last_letter_2_msgs/apply_model_wrenches_srv.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/bind.hpp>
#include <ctime> // for timer
#include <iostream>

namespace gazebo
{
class model_plugin : public ModelPlugin
{
    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnectionEnd; //A class that encapsulates a connection
    event::ConnectionPtr beforeUpdateConnection; //A class that encapsulates a connection

    ///  A node use for ROS transport
    ros::NodeHandle *rosNode;

    // ROS publisher
    ros::Publisher states_pub;

    // Ros services
    ros::ServiceServer apply_wrenches_server;

    ///  A ROS callbackqueue that helps process messages
    ros::CallbackQueue wrenches_rosQueue;

    ///  A thread the keeps running the rosQueue
    std::thread rosQueueThread1;

    last_letter_2_msgs::link_states base_link_states;
    last_letter_2_msgs::link_states airfoil_states[3];
    last_letter_2_msgs::link_states motor_states[4];

    last_letter_2_msgs::model_states model_states;

    int num_wings, num_motors;
    ignition::math::Vector3d relLinVel;
    ignition::math::Vector3d rotation;
    ignition::math::Vector3d relAngVel;
    ignition::math::Vector3d position;
    int i;
    std::string link_name;
    char link_name_temp[20];
    bool wrenches_applied;

    int loop_number;

  public:
    model_plugin() : ModelPlugin() //constructor
    {
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
    {                                                         //and after the World has been loaded.Νot be blocking.
        this->model = _model;
        ROS_INFO("model_plugin just started");

        this->rosNode = new ros::NodeHandle; //Create a ros node for transport
        while (!this->rosNode->ok())
        {
            ROS_INFO("Waiting for node to rise");
        }

        // Spin up the queue helper thread.
        this->rosQueueThread1 =
            std::thread(std::bind(&model_plugin::QueueThread1, this));

        //Connect a callback to the world update start signal.
        this->beforeUpdateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&model_plugin::BeforeUpdate, this));
        this->updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(std::bind(&model_plugin::OnUpdate, this));
        ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv",
                                                                                                                      boost::bind(&model_plugin::applyWrenchOnModel, this, _1, _2), ros::VoidPtr(), &this->wrenches_rosQueue));
        this->apply_wrenches_server = this->rosNode->advertiseService(so);

        // Publish code
        this->states_pub = this->rosNode->advertise<last_letter_2_msgs::model_states>("last_letter_2/gazebo/model_states", 10,true);

        //Read the number of airfoils
        if (!ros::param::getCached("airfoil/nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
        //Read the number of motors
        if (!ros::param::getCached("motor/nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }

        wrenches_applied=false;
        loop_number=0;
        modelStateInit();
    }

    void modelStateInit()
    {
        //Get initial states from parameter server
        XmlRpc::XmlRpcValue list;

        if (!ros::param::getCached("init/position", list)) { ROS_FATAL("Invalid parameters for init/position in param server!"); ros::shutdown();}
        ignition::math::Vector3d xyz_pose(list[0], list[1], list[2]);
        ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        if (!ros::param::getCached("init/orientation", list)) { ROS_FATAL("Invalid parameters for init/orientation in param server!"); ros::shutdown();}
        ignition::math::Vector3d rpy_pose(list[0], list[1], list[2]);
        if (!ros::param::getCached("init/velLin", list)) { ROS_FATAL("Invalid parameters for init/velLin in param server!"); ros::shutdown();}
        ignition::math::Vector3d velLin(list[0], list[1], list[2]);
        if (!ros::param::getCached("init/velAng", list)) { ROS_FATAL("Invalid parameters for init/velAng in param server!"); ros::shutdown();}
        ignition::math::Vector3d velAng(list[0], list[1], list[2]);

        //Set the initial position and rotation
        ignition::math::Pose3d init_pose;
        init_pose.Set(xyz_pose, rpy_pose);
        this->model->SetWorldPose(init_pose);

        // Tranform linear and angular velocity from body frame to world frame for initialized launching
        KDL::Frame tranformation_matrix;
        tf2::Stamped<KDL::Vector> v_out;

        tranformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX(-rpy_pose[2], -rpy_pose[1], rpy_pose[0]), KDL::Vector(0, 0, 0));
        v_out = tf2::Stamped<KDL::Vector>(tranformation_matrix.Inverse() * KDL::Vector(velLin[0], velLin[1], velLin[2]), ros::Time::now(), "airfoil");

        velLin[0] = v_out[0];
        velLin[1] = v_out[1];
        velLin[2] = v_out[2];

        v_out = tf2::Stamped<KDL::Vector>(tranformation_matrix.Inverse() * KDL::Vector(velAng[0], velAng[1], velAng[2]), ros::Time::now(), "airfoil");

        velAng[0] = v_out[0];
        velAng[1] = v_out[1];
        velAng[2] = v_out[2];

        // Set velocities on model
        this->model->SetLinearVel(velLin);  //NWU frame, keep yaw and pitch at zero
        this->model->SetAngularVel(velAng); //NWU frame
    }

    //  ROS helper function that processes messages
    void QueueThread1()
    {
        ROS_INFO(" i am in QueueThread1 now\n");
        ros::WallRate r(1100);  
        // the sleep rate, increase dramaticaly the preformance
        while (this->rosNode->ok())
        {
            this->wrenches_rosQueue.callAvailable();
            r.sleep();
        }
    }


    //service that apply the calculated aerodynamic and propulsion wrenches on relative links of model
    bool applyWrenchOnModel(last_letter_2_msgs::apply_model_wrenches_srv::Request &req,
                            last_letter_2_msgs::apply_model_wrenches_srv::Response &res)
    {
        //apply wrenches to each airfoil and motor
         for (i = 0; i < num_wings; i++)
        {
            ignition::math::Vector3d force, torque;

            force[0]=req.airfoil_forces[i].x;
            force[1]=req.airfoil_forces[i].y;
            force[2]=req.airfoil_forces[i].z;
            sprintf(link_name_temp, "airfoil%i", i + 1);
            link_name.assign(link_name_temp);
            model->GetLink(link_name)->AddLinkForce(force);

            torque[0]=req.airfoil_torques[i].x;
            torque[1]=req.airfoil_torques[i].y;
            torque[2]=req.airfoil_torques[i].z;
            model->GetLink(link_name)->AddRelativeTorque(torque);
        }

        for (i = 0; i < num_motors; i++)
        {
            ignition::math::Vector3d force, torque;

            force[0] = req.motor_thrust[i];
            force[1] = 0;
            force[2] = 0;
            sprintf(link_name_temp, "motor%i", i + 1);
            link_name.assign(link_name_temp);
            model->GetLink(link_name)->AddLinkForce(force);

            torque[0] = req.motor_torque[i];
            torque[1] = 0;
            torque[2] = 0;
            model->GetLink(link_name)->AddRelativeTorque(torque);
        }
        //unlock gazebo step
        wrenches_applied=true;
        return true;
    }

    void BeforeUpdate()
    {
        //wait until wrenches are ready 
        while(!wrenches_applied && loop_number>24) {   }
        
        //lock gazebo step
        wrenches_applied=false;
    }

    void OnUpdate()
    {
        relLinVel = model->GetLink("body_FLU")->RelativeLinearVel();
        rotation = model->GetLink("body_FLU")->WorldPose().Rot().Euler();
        relAngVel = model->GetLink("body_FLU")->RelativeAngularVel();
        position = model->GetLink("body_FLU")->WorldPose().Pos();

        base_link_states.header.stamp = ros::Time::now();
        base_link_states.header.frame_id = "body_FLU";
        base_link_states.x = position[0];
        base_link_states.y = -position[1];
        base_link_states.z = -position[2];
        base_link_states.roll = rotation[0];
        base_link_states.pitch = -rotation[1];
        base_link_states.yaw = -rotation[2];
        base_link_states.u = relLinVel[0];
        base_link_states.v = -relLinVel[1];
        base_link_states.w = -relLinVel[2];
        base_link_states.p = relAngVel[0];
        base_link_states.q = -relAngVel[1];
        base_link_states.r = -relAngVel[2];

        model_states.base_link_states=base_link_states;

        for (i = 0; i < num_wings; i++)
        {
            sprintf(link_name_temp, "airfoil%i", i + 1);
            link_name.assign(link_name_temp);
            relLinVel = model->GetLink(link_name)->RelativeLinearVel();
            rotation = model->GetLink(link_name)->WorldPose().Rot().Euler();
            relAngVel = model->GetLink(link_name)->RelativeAngularVel();
            position = model->GetLink(link_name)->WorldPose().Pos();

            airfoil_states[i].header.stamp = ros::Time::now();
            airfoil_states[i].header.frame_id = link_name;
            airfoil_states[i].x = position[0];
            airfoil_states[i].y = -position[1];
            airfoil_states[i].z = -position[2];
            airfoil_states[i].roll = rotation[0];
            airfoil_states[i].pitch = -rotation[1];
            airfoil_states[i].yaw = -rotation[2];
            airfoil_states[i].u = relLinVel[0];
            airfoil_states[i].v = -relLinVel[1];
            airfoil_states[i].w = -relLinVel[2];
            airfoil_states[i].p = relAngVel[0];
            airfoil_states[i].q = -relAngVel[1];
            airfoil_states[i].r = -relAngVel[2];

            model_states.airfoil_states[i]=airfoil_states[i];
        }

        for (i = 0; i < num_motors; i++)
        {
            sprintf(link_name_temp, "motor%i", i + 1);
            link_name.assign(link_name_temp);
            relLinVel = model->GetLink(link_name)->RelativeLinearVel();
            rotation = model->GetLink(link_name)->WorldPose().Rot().Euler();
            relAngVel = model->GetLink(link_name)->RelativeAngularVel();
            position = model->GetLink(link_name)->WorldPose().Pos();

            motor_states[i].header.stamp = ros::Time::now();
            motor_states[i].header.frame_id = link_name;
            motor_states[i].x = position[0];
            motor_states[i].y = -position[1];
            motor_states[i].z = -position[2];
            motor_states[i].roll = rotation[0];
            motor_states[i].pitch = -rotation[1];
            motor_states[i].yaw = -rotation[2];
            motor_states[i].u = relLinVel[0];
            motor_states[i].v = -relLinVel[1];
            motor_states[i].w = -relLinVel[2];
            motor_states[i].p = relAngVel[0];
            motor_states[i].q = -relAngVel[1];
            motor_states[i].r = -relAngVel[2];

            model_states.motor_states[i]=motor_states[i];
        }
        loop_number++; // check if start from zero!

        model_states.loop_number.data=loop_number;
        //publish model states, ros starts calculation step
        this->states_pub.publish(model_states);

        //publish tranform between gazebo inertia NWU and body frame FLU
        //if declaration is placed in data area of class, causes problems. So it is placed here.
        static tf2_ros::TransformBroadcaster broadcaster_;
        geometry_msgs::TransformStamped transformStamped_;
        tf2::Quaternion quat_;

        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = "inertial_NWU";
        transformStamped_.child_frame_id = "body_FLU";
        transformStamped_.transform.translation.x = model_states.base_link_states.x;
        transformStamped_.transform.translation.y = model_states.base_link_states.y;
        transformStamped_.transform.translation.z = model_states.base_link_states.z;
        quat_.setRPY(model_states.base_link_states.roll, model_states.base_link_states.pitch, model_states.base_link_states.yaw);
        transformStamped_.transform.rotation.x = quat_.x();
        transformStamped_.transform.rotation.y = quat_.y();
        transformStamped_.transform.rotation.z = quat_.z();
        transformStamped_.transform.rotation.w = quat_.w();

        broadcaster_.sendTransform(transformStamped_);
        
        //publish body static tranformations between body_FLU and body_FRD
        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = "body_FLU";
        transformStamped_.child_frame_id = "body_FRD";
        transformStamped_.transform.translation.x = 0;
        transformStamped_.transform.translation.y = 0;
        transformStamped_.transform.translation.z = 0;
        quat_.setRPY(M_PI, 0, 0);
        transformStamped_.transform.rotation.x = quat_.x();
        transformStamped_.transform.rotation.y = quat_.y();
        transformStamped_.transform.rotation.z = quat_.z();
        transformStamped_.transform.rotation.w = quat_.w();

        broadcaster_.sendTransform(transformStamped_);
    }
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(model_plugin)
} // namespace gazebo
