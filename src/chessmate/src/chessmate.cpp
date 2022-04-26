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
#include "chessmate/getPositionOfPieces.h"


#include "franka_msgs/SetPositionCommand.h"
#include "franka_msgs/SetChessGoal.h"
#include "franka_gripper/GripperCommand.h"

std::string fen_string = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b - - 0 1";

/* Vision function return codes */
const int CHESSBOARD_NOT_DETECTED=0;
const int NO_DIFFERENCE_DETECTED=2000;
const int ONE_DIFFERENCE_DETECTED=4000;
const int TWO_DIFFERENCE_DETECTED=3000;
const int MORE_THAN_TWO_DIFFERENCE_DETECTED=5000;
const int EXCEPTION_IN_THE_VISION_LOOP=6000;




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
        ROS_INFO_STREAM("Main loop starts!");
        int a;
        std::cin >> a;    

        gripper_request.request.width = 0.0;
        gripper_request.request.speed = 0.04;
        gripper_request.request.force = 50;
        gripper_request.request.want_to_pick = false;
        gripper_request.request.want_to_move = true;
        gripper_request.request.homing = false;
        gripper_client.call(gripper_request);

        ROS_INFO_STREAM("input until gripper");
        std::cin >> a;
        ROS_INFO_STREAM("gripper successfull");

       
        // check here if franka still on, however I do not know how to do this here TODO

        // error check and cleaning TODO
        bool resp;
        std::cin >> a;
        go_request.request.x = 0;
        go_request.request.y = 0;
        go_request.request.z = 0.40;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = true;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in zeroth go request go to init");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("init go successfull");


        ROS_INFO_STREAM("Wait for opponent move.");
        std::cin >> a;
        // get chessboard info from vision system
        // commented until it is implemented!
        std::string movement_in_fen = "";
        vision_srv_request.request.last_state_fen_string = fen_string;
        resp = vision_client.call(vision_srv_request);

        if(!resp){
            ROS_WARN_STREAM("Vision is not connected to system!");
            ros::Duration(1).sleep();
        }
        else{
            
            int return_code = vision_srv_request.response.return_code;
            

            switch (return_code) {
                case CHESSBOARD_NOT_DETECTED:
                    ROS_WARN_STREAM("Can not see chessboard!");
                    ros::Duration(1).sleep();
                    break;
                case NO_DIFFERENCE_DETECTED:
                    ROS_WARN_STREAM("No difference detected.");
                    ros::Duration(1).sleep();
                    break;
                case ONE_DIFFERENCE_DETECTED:
                    ROS_WARN_STREAM("One difference detected.");
                    ros::Duration(1).sleep();
                    break;
                case TWO_DIFFERENCE_DETECTED:
                    movement_in_fen = vision_srv_request.response.movement_in_fen;
                    std::cout << "Detected movement is : " << movement_in_fen << std::endl;
                    break;
                case MORE_THAN_TWO_DIFFERENCE_DETECTED:
                    ROS_WARN_STREAM("More than two difference detected.");
                    ros::Duration(1).sleep();
                    break;

                case EXCEPTION_IN_THE_VISION_LOOP:
                    ROS_WARN_STREAM("Exception in the vision loop.");
                    ros::Duration(1).sleep();
                    break;
                default:
                    ROS_WARN_STREAM("Default in the switch statement. We should not be here.");
                    ros::Duration(1).sleep();
                    
        }


        if (movement_in_fen == "") {
            ROS_WARN_STREAM("Could not track movement. We have to enter it manually.");
            ROS_WARN_STREAM("If you want to enter it manually, please enter it now. Otherwise, enter 0.");
            std::cin >> movement_in_fen;
            if (movement_in_fen == "0") {
                ROS_WARN_STREAM("We will not enter it manually.");
                ros::Duration(1).sleep();
                continue;
            }
            std::cout << movement_in_fen << " movement has been done by the opponent." << std::endl;   
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

        // First, we need to look at put place square.
        // If it is full, then we will eat that piece and then do the movement. 
        bool is_put_place_full = is_square_full(fen_string,put_place_square);


        // Update fen. Because we will play best move.
        fen_string = chess_next_move.response.fen_string;

        if (is_put_place_full) {
            // TODO , first eat piece.
        }
        // Otherwise, move the piece to the put place square.
        

        // Now, we have the best move, and we can do something with it!
        // Call the vision to get the coordinates of these!

        
        
        ros::ServiceClient joint_client = n.serviceClient<franka_msgs::SetChessGoal>("/franka_go_chess");
        franka_msgs::SetChessGoal joint_request;
        joint_request.request.chess_place = take_place_square + "above";
        ROS_INFO_STREAM("Starting movement!");
        int a;
        std::cin >> a;
        resp = joint_client.call(joint_request);
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
        joint_request.request.chess_place = take_place_square;
        resp = joint_client.call(joint_request);
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
        joint_request.request.chess_place = take_place_square + "above";
        resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in thirdy go request");
            ros::Duration(0.01).sleep();
            continue;
        }

        // GO TO INIT
        std::cin >> a;
        go_request.request.x = 0;
        go_request.request.y = 0;
        go_request.request.z = 0.40;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = true;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in third go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("third go successfull");


        std::cin >> a;
        joint_request.request.chess_place = put_place_square + "above";
        resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in fourth go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("foourth go successfull");

        
        std::cin >> a;
        joint_request.request.chess_place = put_place_square;
        resp = joint_client.call(joint_request);
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

        go_request.request.x = 0;
        go_request.request.y = 0;
        go_request.request.z = 0.4;
        go_request.request.is_relative = false;
        go_request.request.go_to_init = true;
        resp = go_client.call(go_request);
        if(!resp){
            ROS_WARN_STREAM("Error in last go request go to init");
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


}