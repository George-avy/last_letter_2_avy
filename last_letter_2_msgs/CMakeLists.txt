cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2_msgs)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  joystick_input.msg
  link_states.msg
  model_states.msg
  air_data.msg
  aero_wrenches.msg
  prop_wrenches.msg
  model_inputs.msg
)

add_service_files(
  FILES
  apply_model_wrenches_srv.srv
  get_airfoil_inputs_srv.srv
  get_motor_input_srv.srv
  get_link_inputs_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(DEPENDS
  CATKIN_DEPENDS  message_runtime
  std_msgs
)
