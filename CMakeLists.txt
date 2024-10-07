cmake_minimum_required(VERSION 2.8.3)
project(rule_based_gpsr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  )

catkin_python_setup()

# add_message_files(
#   FILES
# )


# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )



catkin_package(
    CATKIN_DEPENDS
)
