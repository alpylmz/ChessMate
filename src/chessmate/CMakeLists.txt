# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(chessmate)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg franka_msgs franka_gripper)

## Declare ROS messages and services
add_message_files(
  FILES 
  pose.msg
)
add_service_files(
  FILES 
  franka_on.srv
  chess_opponent_move.srv
  chessboard_to_coord.srv
  see_chessboard.srv
  chess_next_move.srv
  chess_game_state.srv
  franka_control_start_stop.srv
  pick_and_place.srv
  QueryVisionComponent.srv
  getPositionOfPieces.srv
  chesswatch_serv.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(chessmate src/chessmate.cpp)
target_link_libraries(chessmate ${catkin_LIBRARIES})
#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

# %EndTag(FULLTEXT)%