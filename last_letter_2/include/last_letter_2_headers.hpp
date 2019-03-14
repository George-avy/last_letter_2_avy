#include <ros/ros.h>
#include <string>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_2_msgs/get_model_states_srv.h>
#include <last_letter_2_msgs/get_control_signals_srv.h>
#include <last_letter_2_msgs/airdata_srv.h>
#include <last_letter_2_msgs/air_data.h>
#include <last_letter_2_msgs/control_signals.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/aero_wrenches.h>
#include <last_letter_2_msgs/prop_wrenches.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <last_letter_2_msgs/model_wrenches.h>
#include <last_letter_2_msgs/apply_wrench_srv.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <ctime>
#include "last_letter_2_libs/math_lib.hpp"


class Aerodynamics;
class Propulsion;
class Dynamics;
class Model;
class Master;
class Factory;
class Polynomial;