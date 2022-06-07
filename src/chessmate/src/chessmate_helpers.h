#include <string>
#include <cctype> 
#include <iostream>
#include "franka_gripper/GripperCommand.h"
#include <ros/ros.h>
#include "franka_msgs/SetPositionCommand.h"

bool is_square_full(const std::string& fen_string,const std::string& square) {
    char board[64];
    for (int i = 0; i < 64; i++) 
        board[i] = 'E'; 

    const size_t size = fen_string.size();
    size_t iter = 0;
    int index = 0;

    
    for (; (iter < size) and (fen_string[iter] != ' '); iter++) {
        if (fen_string[iter] == '/')
            continue;

        if (isdigit(fen_string[iter]))
            index += (fen_string[iter] - '0'); 

        else {
            board[index] = 'F';
            index++;
        }
    }


    int square_index_in_array = (8 - atoi(&square.at(1))) * 8;

    if (square.at(0) == 'a')
        square_index_in_array += 0;

    if (square.at(0) == 'b')
        square_index_in_array += 1;

    if (square.at(0) == 'c')
        square_index_in_array += 2;

    if (square.at(0) == 'd')
        square_index_in_array += 3;

    if (square.at(0) == 'e')
        square_index_in_array += 4;

    if (square.at(0) == 'f')
        square_index_in_array += 5;

    if (square.at(0) == 'g')
        square_index_in_array += 6;

    if (square.at(0) == 'h')
        square_index_in_array += 7;

    return board[square_index_in_array] == 'F';
}


char get_piece_to_take(const std::string& fen_string,const std::string& square) {
    char board[64];
    for (int i = 0; i < 64; i++) 
        board[i] = 'E'; 

    const size_t size = fen_string.size();
    size_t iter = 0;
    int index = 0;

    
    for (; (iter < size) and (fen_string[iter] != ' '); iter++) {
        if (fen_string[iter] == '/')
            continue;

        if (isdigit(fen_string[iter]))
            index += (fen_string[iter] - '0'); 

        else {
            board[index] = fen_string[iter];
            index++;
        }
    }


    int square_index_in_array = (8 - atoi(&square.at(1))) * 8;

    if (square.at(0) == 'a')
        square_index_in_array += 0;

    if (square.at(0) == 'b')
        square_index_in_array += 1;

    if (square.at(0) == 'c')
        square_index_in_array += 2;

    if (square.at(0) == 'd')
        square_index_in_array += 3;

    if (square.at(0) == 'e')
        square_index_in_array += 4;

    if (square.at(0) == 'f')
        square_index_in_array += 5;

    if (square.at(0) == 'g')
        square_index_in_array += 6;

    if (square.at(0) == 'h')
        square_index_in_array += 7;

    return board[square_index_in_array];
}


void gripper_move(ros::ServiceClient gripper_client, double width, double speed, double force, bool want_to_pick, bool want_to_move, bool homing) {
    franka_gripper::GripperCommand gripper_request;
    gripper_request.request.width = width;
    gripper_request.request.speed = speed;
    gripper_request.request.force = force;
    gripper_request.request.want_to_pick = want_to_pick;
    gripper_request.request.want_to_move = want_to_move;
    gripper_request.request.homing = homing;
    gripper_client.call(gripper_request);
}

bool franka_go(ros::ServiceClient go_client, float x, float y, float z, bool is_relative, bool go_to_init, bool go_to_side_vision_init, bool change_orientation_for_picking, bool execute_now) {
    bool resp;
    float q_x = -0.9133;
    float q_y = 0.4070;
    float q_z = -0.0070;
    float q_w = 0.0116;
    franka_msgs::SetPositionCommand go_request; 
    go_request.request.x = x;
    go_request.request.y = y;
    go_request.request.z = z;
    go_request.request.q_x = q_x;
    go_request.request.q_y = q_y;
    go_request.request.q_z = q_z;
    go_request.request.q_w = q_w;
    go_request.request.is_relative = is_relative;
    go_request.request.go_to_init = go_to_init;
    go_request.request.go_to_side_vision_init = go_to_side_vision_init;
    go_request.request.change_orientation_for_picking = change_orientation_for_picking;
    go_request.request.execute_now = execute_now;
    resp = go_client.call(go_request);
    if(!resp){
        ROS_WARN_STREAM("Error in go to init");
        ros::Duration(0.01).sleep();
        return false;
    }
    ROS_INFO_STREAM("franka go successfull");
    return true;
}

void m_print(std::string line)
{
    std::cout << line << std::endl;
}