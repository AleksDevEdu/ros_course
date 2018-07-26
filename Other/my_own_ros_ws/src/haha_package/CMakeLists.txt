cmake_minimum_required(VERSION 2.8.3)
project(haha_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

 add_message_files(
   FILES
   Haha.msg
   TMsg.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES haha_package
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

set(TALKER_NODE_NAME tinker)
set(LISTENER_NODE_NAME linker)

set(TALKER_NODE_SRC src/talker.cpp src/vectorsample.cpp)
set(LISTENER_NODE_SRC src/listener.cpp)

add_executable(${TALKER_NODE_NAME} ${TALKER_NODE_SRC})
target_link_libraries(${TALKER_NODE_NAME} ${catkin_LIBRARIES})
add_dependencies(${TALKER_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${LISTENER_NODE_NAME} ${LISTENER_NODE_SRC})
target_link_libraries(${LISTENER_NODE_NAME} ${catkin_LIBRARIES})
add_dependencies(${LISTENER_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})









