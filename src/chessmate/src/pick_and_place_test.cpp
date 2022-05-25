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

    ros::init(argc, argv, "chessmate");
    ros::NodeHandle n;

    // main control loop preparations

    ros::ServiceClient go_client = n.serviceClient<franka_msgs::SetPositionCommand>("/franka_go");

    ros::ServiceClient chess_piece_position_client = n.serviceClient<chessmate::getPositionOfPieces>("/get_piece_coordinates");
    chessmate::getPositionOfPieces chess_piece_position_request;


    ROS_INFO_STREAM("Main loop starting!");

     //! going to side vision
    ROS_INFO_STREAM("This test will move robot! Be careful!!");
    ROS_INFO_STREAM("Going to side vision position!!");
    ROS_INFO_STREAM("Give random input to start:");
    int a;
    std::cin >> a;
    resp = franka_go(go_client, 0, 0, 0, false, false, true, false);
    if (!resp) {
        ROS_WARN_STREAM("ERROR IN GOING TO INIT!");
        return;
    }

    float pick_height, place_height, above_height;
    std::cout << "Enter a pick height: ";
    std::cin >> pick_height;
    std::cout << "Enter a place height: ";
    std::cin >> place_height;
    std::cout << "Enter the above_height height: ";
    std::cin >> above_height;
    // main control starts here!
    while(true){

        bool resp;
        
        std::string dummy_place_pick;
        std::string dummy_place_place;
        std::cout << "Enter a square to pick: ";
        std::cin >> dummy_place_pick;
        std::cout << "Enter a square to place: ";
        std::cin >> dummy_place_place;
        
        float gripper_width;
        std::cout << "Enter gripper width ";
        std::cin >> gripper_width;

        chess_piece_position_request.request.from_piece = dummy_place_pick;
        chess_piece_position_request.request.to_piece = dummy_place_place;

        chess_piece_position_client.call(chess_piece_position_request);

        float square_position_x_pick = chess_piece_position_request.response.from_x;
        float square_position_y_pick = chess_piece_position_request.response.from_y;

        float square_position_x_place = chess_piece_position_request.response.to_x;
        float square_position_y_place = chess_piece_position_request.response.to_y;

        std::cout << "square_position_x_pick: " << square_position_x_pick << std::endl;
        std::cout << "square_position_y_pick: " << square_position_y_pick << std::endl;

        resp = franka_go(go_client, square_position_x_pick, square_position_y_pick, above_height, false, false, false, true);
        //resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        else{
            ROS_INFO_STREAM("go successfull");
        }

        gripper_move(gripper_client, 0.04, 0.05, 50, false, true, false);

        resp = franka_go(go_client, square_position_x_pick, square_position_y_pick, pick_height, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in second go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        else{
            ROS_INFO_STREAM("go successfull");
        }

        gripper_move(gripper_client, gripper_width, 0.05, 50, true, false, false);

        resp = franka_go(go_client, square_position_x_pick, square_position_y_pick, above_height, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in above go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("above go successfull");

        float square_position_x_place = chess_piece_position_request.response.to_x;
        float square_position_y_place = chess_piece_position_request.response.to_y;

        std::cout << "square_position_x_place: " << square_position_x_place << std::endl;
        std::cout << "square_position_y_place: " << square_position_y_place << std::endl;

        resp = franka_go(go_client, square_position_x_place, square_position_y_place, above_height, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in above go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("above go successfull");

        resp = franka_go(go_client, square_position_x_place, square_position_y_place, place_height, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in above go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("above go successfull");

        gripper_move(gripper_client, 0.04, 0.05, 50, false, true, false);

        ROS_INFO_STREAM("END OF THE LOOP");
        ROS_INFO_STREAM("<---------------------->");

    }
    
}















