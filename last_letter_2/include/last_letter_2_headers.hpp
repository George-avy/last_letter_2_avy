#include <ros/ros.h>
#include "last_letter_2_libs/math_lib.hpp"
#include <last_letter_2_msgs/air_data.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/link_states.h>
#include <last_letter_2_msgs/aero_wrenches.h>
#include <last_letter_2_msgs/prop_wrenches.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>
#include <last_letter_2_msgs/apply_model_wrenches_srv.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/transport_hints.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <ctime>
#include <string>
#include <iostream>

class Aerodynamics;
class Propulsion;
class Dynamics;
class Model;
class Factory;
class Environment;
class Polynomial;

