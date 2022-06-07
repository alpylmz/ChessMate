#include <ros/ros.h>
#include <iostream>
#include <string>

#include "chessmate/helper.hpp"
#include "chessmate_helpers.h"

// messages
#include "chessmate/pose.h"

// services
#include "chessmate/franka_on.h"
#include "chessmate/see_chessboard.h"
#include "chessmate/franka_control_start_stop.h"
#include "chessmate/chess_next_move.h"
#include "chessmate/pick_and_place.h"
#include "chessmate/chessboard_to_coord.h"
#include "chessmate/QueryVisionComponent.h"
#include "chessmate/chess_opponent_move.h"
#include "chessmate/chess_next_move.h"
#include "chessmate/chess_game_state.h"
#include "chessmate/getPositionOfPieces.h"
#include "chessmate/chesswatch_serv.h"


#include "franka_msgs/SetPositionCommand.h"
#include "franka_msgs/SetChessGoal.h"
#include "franka_msgs/SetTrajectoryCommand.h"
#include "franka_msgs/HRI.h"
#include "franka_gripper/GripperCommand.h"


int main(int argc, char** argv){
    bool resp;

    ros::init(argc, argv, "chessmate");
    ros::NodeHandle n;

    // main control loop preparations

    ros::ServiceClient go_client = n.serviceClient<franka_msgs::SetPositionCommand>("/franka_go");

    ros::ServiceClient chess_piece_position_client = n.serviceClient<chessmate::getPositionOfPieces>("/get_piece_coordinates");
    chessmate::getPositionOfPieces chess_piece_position_request;

    ros::ServiceClient gripper_client = n.serviceClient<franka_gripper::GripperCommand>("/franka_custom_gripper_service");

    ROS_INFO_STREAM(">------------------------------<");
    ROS_INFO_STREAM("PICK AND PLACE TEST");

     //! going to side vision
    ROS_INFO_STREAM("This test will move robot! Be careful!!");
    ROS_INFO_STREAM("Going to side vision position!!");
    ROS_INFO_STREAM("Give random input to start:");
    std::string a = "";

    std::cin >> a;
    
    ROS_INFO_STREAM("Going to side vision position!!");

    resp = franka_go(go_client, 0, 0, 0, false, false, true, false, true);
    if (!resp) {
        ROS_WARN_STREAM("ERROR IN GOING TO INIT!");
        return 0;
    }

    std::string dummy_place = "";
    
    
    // main control starts here!
    while(true){

        std::cout << "Enter a square to go: ";
        std::cin >> dummy_place;
        
        //getline(std::cin,dummy_place);
        float height;
        std::cout << "Enter a height: ";
        std::cin >> height;
        
        chess_piece_position_request.request.from_piece = dummy_place;
        chess_piece_position_request.request.to_piece = dummy_place;

        chess_piece_position_client.call(chess_piece_position_request);

        float square_position_x = chess_piece_position_request.response.from_x;
        float square_position_y = chess_piece_position_request.response.from_y;

        std::cout << "square_position_x: " << square_position_x << std::endl;
        std::cout << "square_position_y: " << square_position_y << std::endl;

        resp = franka_go(go_client, square_position_x, square_position_y, height, false, false, false, true, true);
        //resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        else{
            ROS_INFO_STREAM("go successfull");
        }

        

    }
    
}















