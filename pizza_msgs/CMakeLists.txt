cmake_minimum_required(VERSION 2.8.3)
project(pizza_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Service definitions generated above
add_service_files(
  FILES
  spread_tomato.srv
  scatter_cheese.srv
  move_to_oven.srv
  change_tool.srv
  place_in_box.srv
  cut.srv
  pack_pizza.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
