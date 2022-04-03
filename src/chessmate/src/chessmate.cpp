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

std::string fen_string = "";

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


    ros::ServiceClient vision_client = n.serviceClient<chessmate::QueryVisionComponent>("/franka_vision/query_component");
    chessmate::QueryVisionComponent vision_srv_request;
    vision_srv_request.request.last_state_fen_string = fen_string;
    while(!vision_client.call(vision_srv_request)){
        ROS_WARN_STREAM("Vision is not connected to system!");
        ros::Duration(1).sleep();
    }
    

    // ------------------------------------------------------------------------------------------

    // can be commented tomorrow
    // ------------------------------------------------------------------------------------------
    // Here franka_control package's franka_control node should be started
    // or we can get into the source code of it, and open and close it with a service, I wrote it here
    // Another solution may be getting that necessary code from that node to here, but I think it will be more clear when we really use this code with robot
    // So I only left this suggestion here

    ros::ServiceClient start_or_stop_franka_control_node_client = n.serviceClient<chessmate::franka_control_start_stop>("/franka_control_start_stop");
    chessmate::franka_control_start_stop franka_control_start;
    franka_control_start.request.start_or_stop = 0;
    while(!start_or_stop_franka_control_node_client.call(franka_control_start)){
        ROS_WARN_STREAM("Franka control node cannot be started.");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Franka control is started. You can get robot information now.");
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
    ros::ServiceClient chess_next_move_client = n.serviceClient<chessmate::chess_next_move>("/chess_next_move");
    chessmate::chess_next_move chess_move;
    ros::ServiceClient pick_and_place_client = n.serviceClient<chessmate::pick_and_place>("/pick_and_place");
    chessmate::pick_and_place pick_and_place;

    ROS_INFO_STREAM("Main loop starting!");

    // main control starts here!
    while(true){
        // check here if franka still on, however I do not know how to do this here TODO

        // error check and cleaning TODO

        // get chessboard info from vision system
        // commented until it is implemented!
        std::string god_knows_where_this_comes_from;
        vision_srv_request.request.last_state_fen_string = god_knows_where_this_comes_from;
        auto resp = vision_client.call(vision_srv_request);
        if(!resp){
            ROS_WARN_STREAM("Vision is not connected to system!");
            ros::Duration(1).sleep();
        }
        else{
            // I think this string is the new fen string, but I am really not sure, and I have really no idea what Burak's code does.
            //TODOauto new_str = vision_srv_request.unkown_str;
        
            // TODOauto some_new_string_that_will_be_used_in_commands_or_new_fen_string = new_str;
        }
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
            chessmate::chessboard_to_coord chessboard_to_coord;
            chessboard_to_coord.request.take_place_x = chess_move.response.take_place_x;
            chessboard_to_coord.request.take_place_y = chess_move.response.take_place_y;
            chessboard_to_coord.request.put_place_x = chess_move.response.put_place_x;
            chessboard_to_coord.request.put_place_y = chess_move.response.put_place_y;
            
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