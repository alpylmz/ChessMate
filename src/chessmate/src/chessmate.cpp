#include <ros/ros.h>
#include <string>

#include "chessmate/helper.hpp"

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
#include "chessmate/getPositionOfPieces.h"


#include "franka_msgs/SetPositionCommand.h"
#include "franka_gripper/GripperCommand.h"

std::string fen_string = "8/8/8/3K3P/6k1/p6n/8/8 w - - 0 1";

int main(int argc, char** argv){
    ros::init(argc, argv, "chessmate");
    ros::NodeHandle n;
    /*
    // can be commented tomorrow
    // ------------------------------------------------------------------------------------------
    // checks if the system is open, with "/franka_on" service!
    ros::ServiceClient robot_on_client = n.serviceClient<chessmate::franka_on>("/franka_on");
    chessmate::franka_on franka_is_on_client;
    while(!robot_on_client.call(franka_is_on_client)){
        ROS_WARN_STREAM("Franka is not connected to system!");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Franka is connected.");
    */
    // ------------------------------------------------------------------------------------------


    ros::ServiceClient vision_client = n.serviceClient<chessmate::QueryVisionComponent>("/franka_vision");
    chessmate::QueryVisionComponent vision_srv_request;
    vision_srv_request.request.last_state_fen_string = fen_string;
    while(!vision_client.call(vision_srv_request)){
        ROS_WARN_STREAM("Vision is not connected to system!");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Vision is connected.");
    //ROS_INFO_STREAM(vision_srv_request.response.success << " " << vision_srv_request.response.can_see_chessboard << " " << vision_srv_request.response.is_there_movement << " " << vision_srv_request.response.movement_in_fen);
    
    // ------------------------------------------------------------------------------------------

    // can be commented tomorrow
    // ------------------------------------------------------------------------------------------
    // Here franka_control package's franka_control node should be started
    // or we can get into the source code of it, and open and close it with a service, I wrote it here
    // Another solution may be getting that necessary code from that node to here, but I think it will be more clear when we really use this code with robot
    // So I only left this suggestion here
    /*
    ros::ServiceClient start_or_stop_franka_control_node_client = n.serviceClient<chessmate::franka_control_start_stop>("/franka_control_start_stop");
    chessmate::franka_control_start_stop franka_control_start;
    franka_control_start.request.start_or_stop = 0;
    while(!start_or_stop_franka_control_node_client.call(franka_control_start)){
        ROS_WARN_STREAM("Franka control node cannot be started.");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Franka control is started. You can get robot information now.");
    */
    // ------------------------------------------------------------------------------------------

    // First, do not give commands to external things.
    PandaJoints neutral_pose;
    // assign the robot to neutral pose, or save it here and give it!
    float joint0_start = neutral_pose.joint0;

    //searchChessboard();    

    ROS_INFO_STREAM("Chessboard found!");
    
    // from here, neutral pose is the pose that looks to the chessboard, will be used later!!!!

    // main control loop preparations
    // these commands come from our other packages
    // for now we can call them from CLI

    ros::ServiceClient pick_and_place_client = n.serviceClient<chessmate::pick_and_place>("/pick_and_place");
    chessmate::pick_and_place pick_and_place;

    ros::ServiceClient go_client = n.serviceClient<franka_msgs::SetPositionCommand>("/franka_go");
    franka_msgs::SetPositionCommand go_request; 

    ros::ServiceClient gripper_client = n.serviceClient<franka_gripper::GripperCommand>("/franka_custom_gripper_service");
    franka_gripper::GripperCommand gripper_request;


    ROS_INFO_STREAM("Main loop starting!");

    // main control starts here!
    while(true){

        gripper_request.request.width = 0.01;
        gripper_request.request.speed = 0.01;
        gripper_request.request.force = 50;
        gripper_request.request.want_to_pick = false;
        gripper_request.request.want_to_move = true;
        gripper_request.request.homing = false;
        gripper_client.call(gripper_request);

        ROS_INFO_STREAM("gripper successfull");

       
        // check here if franka still on, however I do not know how to do this here TODO

        // error check and cleaning TODO

        // get chessboard info from vision system
        // commented until it is implemented!
        std::string movement_in_fen = "";
        vision_srv_request.request.last_state_fen_string = fen_string;
        auto resp = vision_client.call(vision_srv_request);
        if(!resp){
            ROS_WARN_STREAM("Vision is not connected to system!");
            ros::Duration(1).sleep();
        }
        else{

            if(!vision_srv_request.response.can_see_chessboard){
                ROS_WARN_STREAM("Can not see chessboard!");
                ros::Duration(1).sleep();
                continue;
            }
            else{
                ROS_INFO_STREAM("Chessboard found!");
                if(!vision_srv_request.response.is_there_movement){
                    ROS_WARN_STREAM("There is no movement!");
                    ros::Duration(1).sleep();
                    continue;
                }
                else{
                    ROS_INFO_STREAM("There is movement!");
                    // compare two strings
                    if(vision_srv_request.response.movement_in_fen == ""){
                        ROS_WARN_STREAM("It seems that there are 3 movements done!");
                        ros::Duration(1).sleep();
                        continue;
                    }
                    else{
                        movement_in_fen = vision_srv_request.response.movement_in_fen;
                    }
                }
            }
        }

        // Vision passed all checks, now we can do something with it!
        // Now, update fen string
        ros::ServiceClient update_fen_client = n.serviceClient<chessmate::chess_opponent_move>("/chess_opponent_move");
        chessmate::chess_opponent_move chess_opponent_move;
        chess_opponent_move.request.move = movement_in_fen;
        resp = update_fen_client.call(chess_opponent_move);
        if(!resp){
            ROS_WARN_STREAM("Call to stockfish failed!");
            ros::Duration(0.01).sleep();
            continue;
        }
        // If the call is successfull, take the new fen string, and get the next best move
        fen_string = chess_opponent_move.response.fen_string;
        ROS_INFO_STREAM("Fen string updated!");
        ros::ServiceClient chess_next_move_client = n.serviceClient<chessmate::chess_next_move>("/chess_next_move");
        chessmate::chess_next_move chess_next_move;
        resp = chess_next_move_client.call(chess_next_move);
        if(!resp){
            ROS_WARN_STREAM("Call to stockfish failed!");
            ros::Duration(0.01).sleep();
            continue;
        }
        std::string take_place_square = chess_next_move.response.from_square;
        std::string put_place_square  = chess_next_move.response.to_square;

        // Now, we have the best move, and we can do something with it!
        // Call the vision to get the coordinates of these!

        ros::ServiceClient get_coordinates_client = n.serviceClient<chessmate::getPositionOfPieces>("/get_piece_coordinates");
        chessmate::getPositionOfPieces get_coordinates_request;
        ROS_INFO_STREAM("Take place is: " << take_place_square);
        ROS_INFO_STREAM("Put place is: " << put_place_square);
        get_coordinates_request.request.from_piece = take_place_square;
        get_coordinates_request.request.to_piece = put_place_square;

        resp = get_coordinates_client.call(get_coordinates_request);
        if(!resp){
            ROS_WARN_STREAM("Call to get coordinates failed!");
            ros::Duration(0.01).sleep();
            continue;
        }
        float from_x = get_coordinates_request.response.from_x;
        float from_y = get_coordinates_request.response.from_y;
        float to_x = get_coordinates_request.response.to_x;
        float to_y = get_coordinates_request.response.to_y;
        ROS_INFO_STREAM("From x: " << from_x << " From y: " << from_y << " To x: " << to_x << " To y: " << to_y << std::endl);
        
        int a;
        std::cin >> a;
        go_request.request.x = from_x;
        go_request.request.y = from_y;
        go_request.request.z = 0.4;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = false;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("go successfull");
        

        std::cin >> a;
        gripper_request.request.width = 0.04;
        gripper_request.request.speed = 0.05;
        gripper_request.request.force = 50;
        gripper_request.request.want_to_pick = false;
        gripper_request.request.want_to_move = true;
        gripper_request.request.homing = false;
        gripper_client.call(gripper_request);

        ROS_INFO_STREAM("gripper successfull");

        std::cin >> a;
        go_request.request.x = from_x;
        go_request.request.y = from_y;
        go_request.request.z = 0.17;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = false;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in sec go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("sec go successfull");


        std::cin >> a;
        gripper_request.request.width = 0.001;
        gripper_request.request.speed = 0.05;
        gripper_request.request.force = 50;
        gripper_request.request.want_to_pick = true;
        gripper_request.request.want_to_move = false;
        gripper_request.request.homing = false;
        gripper_client.call(gripper_request);

        ROS_INFO_STREAM("sec gripper successfull");


        std::cin >> a;
        go_request.request.x = from_x;
        go_request.request.y = from_y;
        go_request.request.z = 0.40;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = false;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in third go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("third go successfull");

        std::cin >> a;

        go_request.request.x = to_x;
        go_request.request.y = to_y;
        go_request.request.z = 0.40;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = false;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in fourth go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("foourth go successfull");

        std::cin >> a;

        go_request.request.x = to_x;
        go_request.request.y = to_y;
        go_request.request.z = 0.17;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = false;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in fifth go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("fifth go successfull");

        std::cin >> a;
        gripper_request.request.width = 0.05;
        gripper_request.request.speed = 0.05;
        gripper_request.request.force = 50;
        gripper_request.request.want_to_pick = false;
        gripper_request.request.want_to_move = true;
        gripper_request.request.homing = false;
        gripper_client.call(gripper_request);

        ROS_INFO_STREAM(" gripper release  successfull");
        std::cin >> a;

        go_request.request.x = from_x;
        go_request.request.y = from_y;
        go_request.request.z = 0.4;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = true;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("go to init successfull");
        /*
        if(!resp){
            ROS_WARN_STREAM("Error in first gripper request");
            ros::Duration(0.01).sleep();
            continue;
        }
        */






        // Now, we have the coordinates of the pieces, we can do something with it!

        /*

        // send the chessboard info to stockfish
        while(!chess_next_move_client.call(chess_move)){
            ROS_WARN_STREAM("Chess next move call unsuccessful!");
            ros::Duration(0.01).sleep();
        }
        */
        /*
        // If player cheated, send the info to the HRI, and let HRI to finish its movement!!
        // If player plays correctly, send the info to vision, get the coordinates, send them to motion planner node
        // USER CHEATED!
        if(chess_move.response.is_state_valid == 0){
            ROS_WARN_STREAM("Player cheated!");
            pick_and_place.request.is_something_hacky = 1;
            while(!pick_and_place_client.call(pick_and_place)){
                ROS_WARN_STREAM("Pick and place call unsuccessful!");
                ros::Duration(0.01).sleep();
            }
        }
        else{
        */
            /*
            chessmate::chessboard_to_coord chessboard_to_coord;
            chessboard_to_coord.request.take_place_x = chess_move.response.take_place_x;
            chessboard_to_coord.request.take_place_y = chess_move.response.take_place_y;
            chessboard_to_coord.request.put_place_x = chess_move.response.put_place_x;
            chessboard_to_coord.request.put_place_y = chess_move.response.put_place_y;
            */
            /*
            while(!find_coordinates_from_chessboard_client.call(chessboard_to_coord)){
                ROS_WARN_STREAM("find coordinates from chessboard call unsuccessful!");
                ros::Duration(0.01).sleep();
            }
            */
            /*
            pick_and_place.request.take_coord_x = next_move.response.take_coord_x;
            pick_and_place.request.take_coord_y = next_move.response.take_coord_y;
            pick_and_place.request.take_coord_z = next_move.response.take_coord_z;
            pick_and_place.request.put_coord_x = next_move.response.put_coord_x;
            pick_and_place.request.put_coord_y = next_move.response.put_coord_y;
            pick_and_place.request.put_coord_z = next_move.response.put_coord_z;
            pick_and_place.request.is_something_hacky = 0;
            while(!pick_and_place_client.call(pick_and_place)){
                ROS_WARN_STREAM("Pick and place call unsuccessful!");
                ros::Duration(0.01).sleep();
            }
            */
        //}

        // Here we need to check if HRI part or motion planner part completed their movements.
        // However, I am not sure which way is the best since we did not implement these parts.
        // So I left it here...
    }


}