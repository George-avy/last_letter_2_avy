#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <last_letter_2_msgs/joystick_input.h>

ros::Publisher pub;

int axisIndex[20];
int buttonIndex[20];
double throwIndex[20];

void joy2chan(sensor_msgs::Joy joyMsg)
{
    last_letter_2_msgs::joystick_input channels;
    double input[20];
    int i;
    //put axis and buttons singals in a continuous array to create channels
    for (i = 0; i < 20; i++)
    {
        if (axisIndex[i] != -1)
        { // if an axis is assigned in this channel
            input[i] = 1.0 / throwIndex[i] * joyMsg.axes[axisIndex[i]];
        }
        else if (buttonIndex[i] != -1)
        { // if a button is assigned in this channel
            input[i] = 1.0 / throwIndex[i] * joyMsg.buttons[buttonIndex[i]];
        }
        else
        {
            input[i] = 0.0;
        }
    }
    //load channels with joystick inputs
    for (i = 0; i < 20; i++)
    {
        channels.value[i] = input[i];
    }
    channels.header.stamp = ros::Time::now();
    pub.publish(channels);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_channels_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joy", 1, joy2chan, ros::TransportHints().tcpNoDelay());
    pub = n.advertise<last_letter_2_msgs::joystick_input>("last_letter_2/channels", 1);

    // Read the controller configuration parameters from the HID.yaml file
    XmlRpc::XmlRpcValue listInt, listDouble;
    int i;
    if (!ros::param::getCached("/HID/throws", listDouble))
    {
        ROS_FATAL("Invalid parameters for -/HID/throws- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listDouble.size(); ++i)
    {
        ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble); //Asserts that the provided condition evaluates to true.
        throwIndex[i] = listDouble[i];
    }
    std::cout << "Reading input axes" << std::endl;
    if (!ros::param::getCached("/HID/axes", listInt))
    {
        ROS_FATAL("Invalid parameters for -/HID/axes- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listInt.size(); ++i)
    {
        ROS_ASSERT(listInt[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        axisIndex[i] = listInt[i];
    }
    ROS_INFO("Reading input buttons configuration");
    if (!ros::param::getCached("/HID/buttons", listInt))
    {
        ROS_FATAL("Invalid parameters for -/HID/buttons- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listInt.size(); ++i)
    {
        ROS_ASSERT(listInt[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        buttonIndex[i] = listInt[i];
    }

    // Start spinning
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}
