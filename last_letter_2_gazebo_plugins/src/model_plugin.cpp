#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ros/service.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/link_states.h>
#include <last_letter_2_msgs/apply_model_wrenches_srv.h>
#include <last_letter_2_msgs/joystick_input.h>
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
#include <string>

#include <thread>
#include <mutex>
#include <condition_variable>

bool wrenches_applied;

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

    /// ROS subscriber
    ros::Subscriber channels_sub;

    // Ros services
    ros::ServiceServer apply_wrenches_server;

    ///  A ROS callbackqueue that helps process messages
    ros::CallbackQueue wrenches_rosQueue;

    ///  A thread the keeps running the rosQueue
    std::thread rosQueueThread1;

    // mutex and cond_variable
    std::condition_variable cv;
    std::mutex m;

    last_letter_2_msgs::link_states base_link_states;
    last_letter_2_msgs::link_states airfoil_states[3];
    last_letter_2_msgs::link_states motor_states[6];
    last_letter_2_msgs::model_states model_states;
    ignition::math::Vector3d relLinVel;
    ignition::math::Vector3d rotation;
    ignition::math::Vector3d relAngVel;
    ignition::math::Vector3d position;
    ignition::math::Vector3d force, torque;
   
    int num_wings, num_motors;
    int i ,thread_rate, step_number;
    std::string link_name, joint_name;
    char name_temp[30];
    float omega,rotationDir[6];
    float camera_angle, laser_angle;

public:
    model_plugin() : ModelPlugin() //constructor
    {
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
    {                                                                       //and after the World has been loaded.Νot be blocking.
        this->model = _model;
        ROS_INFO("model_plugin just started");

        this->rosNode = new ros::NodeHandle; //Create a ros node for transport

        while (!this->rosNode->ok())
        {
            ROS_INFO("Waiting for node to rise");
        }

        // Spin up the queue helper thread.
        this->rosQueueThread1 =
            std::thread(std::bind(&model_plugin::QueueThread, this));

        //Connect a callback to the world update start signal.
        this->beforeUpdateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&model_plugin::BeforeUpdate, this));
        this->updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(std::bind(&model_plugin::OnUpdate, this));
       
        ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv",
                                                                                                                      boost::bind(&model_plugin::applyWrenchOnModel, this, _1, _2), ros::VoidPtr(), &this->wrenches_rosQueue));
        this->apply_wrenches_server = this->rosNode->advertiseService(so);

        //Subscriber
        ros::SubscribeOptions soo = (ros::SubscribeOptions::create<last_letter_2_msgs::joystick_input>("last_letter_2/channels", 1, boost::bind(&model_plugin::manageChan, this, _1), ros::VoidPtr(), &this->wrenches_rosQueue));
        this->channels_sub = this->rosNode->subscribe(soo);

        // Publish code
        this->states_pub = this->rosNode->advertise<last_letter_2_msgs::model_states>("last_letter_2/model_states", 1,true);

        //Read the number of airfoils
        if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
        //Read the number of motors
        if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }

        char paramMsg[50];
        for (i=0; i< num_motors; i++)
        {
            sprintf(paramMsg, "motor%i/rotationDir", i+1);
            if (!ros::param::getCached(paramMsg, rotationDir[i])) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
        }
        
        wrenches_applied = false;
        step_number = 0;
        omega=0;
        camera_angle=0;
        laser_angle=0;
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

        // Transform linear and angular velocity from body frame to world frame for initialized launching
        KDL::Frame transformation_matrix;
        tf2::Stamped<KDL::Vector> v_out;

        transformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX(-rpy_pose[2], -rpy_pose[1], rpy_pose[0]), KDL::Vector(0, 0, 0));
        v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(velLin[0], velLin[1], velLin[2]), ros::Time::now(), "airfoil");

        velLin[0] = v_out[0];
        velLin[1] = v_out[1];
        velLin[2] = v_out[2];

        v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(velAng[0], velAng[1], velAng[2]), ros::Time::now(), "airfoil");

        velAng[0] = v_out[0];
        velAng[1] = v_out[1];
        velAng[2] = v_out[2];

        // Set velocities on model
        this->model->SetLinearVel(velLin);  //NWU frame, keep yaw and pitch at zero
        this->model->SetAngularVel(velAng); //NWU frame
    }

    //  ROS helper function that processes messages
    void QueueThread()
    {
        // Set thread rate= 2.2 * simulation frequency
        // After many tries, it seems to be the most efficent rate
        thread_rate= 2.2 * model->GetWorld()->Physics()->GetRealTimeUpdateRate();
        ROS_INFO(" Gazebo queue thread started with rate = %d\n",thread_rate);

        // the sleep rate, increase dramaticaly the preformance
        ros::WallRate r(thread_rate);
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
        std::lock_guard<std::mutex> lk(m);

        //apply wrenches to each airfoil and motor
        for (i = 0; i < num_wings; i++)
        {
            force[0]=req.airfoil_forces[i].x;
            force[1]=req.airfoil_forces[i].y;
            force[2]=req.airfoil_forces[i].z;
            sprintf(name_temp, "airfoil%i", i + 1);
            link_name.assign(name_temp);
            model->GetLink(link_name)->AddLinkForce(force);

            torque[0]=req.airfoil_torques[i].x;
            torque[1]=req.airfoil_torques[i].y;
            torque[2]=req.airfoil_torques[i].z;
            model->GetLink(link_name)->AddRelativeTorque(torque);
        }
        for (i = 0; i < num_motors; i++)
        {
            force[0] = req.motor_thrust[i];
            force[1] = 0;
            force[2] = 0;
            sprintf(name_temp, "motor%i", i + 1);
            link_name.assign(name_temp);
            model->GetLink(link_name)->AddLinkForce(force);

            torque[0] = req.motor_torque[i];
            torque[1] = 0;
            torque[2] = 0;
            model->GetLink(link_name)->AddRelativeTorque(torque);

            omega = req.motor_omega[i];
            sprintf(name_temp, "motor%i_to_axle%i", i + 1, i + 1); // joint name need fix
            joint_name.assign(name_temp);
            model->GetJoint(joint_name)->SetVelocity(0,omega);
        }

        //Handle sensors

        //Check if camera_joint exists.
        // If camera_joitn exists, set the camera_angle, if not skip
        if (model->GetJoint("camera_joint"))
        {
            model->GetJoint("camera_joint")->SetPosition(0, camera_angle);
        }

        //Check if laser_joint exists.
        // If laser_joitn exists, set the laser_angle, if not skip
        if (model->GetJoint("laser_joint"))
        {
            model->GetJoint("laser_joint")->SetPosition(0, laser_angle);
        }

        //unlock gazebo step
        wrenches_applied = true;
        cv.notify_one();
        return true;
    }

    //Manage channel functions 
    void manageChan(const last_letter_2_msgs::joystick_input::ConstPtr &channels)
    {
        //Handle Camera's angle 
        //Camera sensor listen to 5th channel
        if (channels->value[4] == -1 && camera_angle>=-1.9)
        {
            camera_angle-=0.1;
        }
        if (channels->value[4] == 1 && camera_angle<=1.9)
        {
            camera_angle+=0.1;
        }

        //Handle Laser's angle
        //Laser sensor listen to 6th channel
        if (channels->value[5] == 1 && laser_angle>=-0.9)
        {
            laser_angle-=0.1;
        }
        if (channels->value[5] == -1 && laser_angle<=0.9)
        {
            laser_angle+=0.1;
        }
    }

    void BeforeUpdate()
    {
        std::unique_lock<std::mutex> lk(m);
        //wait until wrenches are ready
        if (step_number>50)  // do 50 steps without being stuck, to be sure that everything is ready
        {
            cv.wait(lk, [] { return wrenches_applied == true; });
        }
        //lock gazebo's next step
        wrenches_applied=false;
    }

    void OnUpdate()
    {
        //publish transform between gazebo inertia NWU and body frame FLU
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
        quat_.setRPY(model_states.base_link_states.phi, model_states.base_link_states.theta, model_states.base_link_states.psi);
        transformStamped_.transform.rotation.x = quat_.x();
        transformStamped_.transform.rotation.y = quat_.y();
        transformStamped_.transform.rotation.z = quat_.z();
        transformStamped_.transform.rotation.w = quat_.w();

        broadcaster_.sendTransform(transformStamped_);

        //publish body static transformations between body_FLU and body_FRD
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
        relLinVel = model->GetLink("body_FLU")->RelativeLinearVel();
        rotation = model->GetLink("body_FLU")->WorldPose().Rot().Euler();
        relAngVel = model->GetLink("body_FLU")->RelativeAngularVel();
        position = model->GetLink("body_FLU")->WorldPose().Pos();

        base_link_states.header.stamp = ros::Time::now();
        base_link_states.header.frame_id = "body_FLU";
        base_link_states.x = position[0];
        base_link_states.y = position[1];
        base_link_states.z = position[2];
        base_link_states.phi = rotation[0];
        base_link_states.theta = rotation[1];
        base_link_states.psi = rotation[2];
        base_link_states.u = relLinVel[0];
        base_link_states.v = relLinVel[1];
        base_link_states.w = relLinVel[2];
        base_link_states.p = relAngVel[0];
        base_link_states.q = relAngVel[1];
        base_link_states.r = relAngVel[2];

        model_states.base_link_states=base_link_states;

        for (i = 0; i < num_wings; i++)
        {
            sprintf(name_temp, "airfoil%i", i + 1);
            link_name.assign(name_temp);
            relLinVel = model->GetLink(link_name)->RelativeLinearVel();
            rotation = model->GetLink(link_name)->WorldPose().Rot().Euler();
            relAngVel = model->GetLink(link_name)->RelativeAngularVel();
            position = model->GetLink(link_name)->WorldPose().Pos();

            airfoil_states[i].header.stamp = ros::Time::now();
            airfoil_states[i].header.frame_id = link_name;
            airfoil_states[i].x = position[0];
            airfoil_states[i].y = position[1];
            airfoil_states[i].z = position[2];
            airfoil_states[i].phi = rotation[0];
            airfoil_states[i].theta = rotation[1];
            airfoil_states[i].psi = rotation[2];
            airfoil_states[i].u = relLinVel[0];
            airfoil_states[i].v = relLinVel[1];
            airfoil_states[i].w = relLinVel[2];
            airfoil_states[i].p = relAngVel[0];
            airfoil_states[i].q = relAngVel[1];
            airfoil_states[i].r = relAngVel[2];

            model_states.airfoil_states[i]=airfoil_states[i];
        }

        for (i = 0; i < num_motors; i++)
        {
            sprintf(name_temp, "motor%i", i + 1);
            link_name.assign(name_temp);
            relLinVel = model->GetLink(link_name)->RelativeLinearVel();
            rotation = model->GetLink(link_name)->WorldPose().Rot().Euler();
            relAngVel = model->GetLink(link_name)->RelativeAngularVel();
            position = model->GetLink(link_name)->WorldPose().Pos();

            motor_states[i].header.stamp = ros::Time::now();
            motor_states[i].header.frame_id = link_name;
            motor_states[i].x = position[0];
            motor_states[i].y = position[1];
            motor_states[i].z = position[2];
            motor_states[i].phi = rotation[0];
            motor_states[i].theta = rotation[1];
            motor_states[i].psi = rotation[2];
            motor_states[i].u = relLinVel[0];
            motor_states[i].v = relLinVel[1];
            motor_states[i].w = relLinVel[2];
            motor_states[i].p = relAngVel[0];
            motor_states[i].q = relAngVel[1];
            motor_states[i].r = relAngVel[2];

            model_states.motor_states[i]=motor_states[i];
        }
        step_number++; // check if start from zero!
        //publish model states, ros starts calculation step
        model_states.header.stamp=ros::Time::now();

        this->states_pub.publish(model_states);
    }
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(model_plugin)
} // namespace gazebo
