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
#include "franka_msgs/HRI.h"
#include "franka_gripper/GripperCommand.h"

std::string fen_string = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b - - 0 1";

/* Vision function return codes */
const int CHESSBOARD_NOT_DETECTED=0;
const int NO_DIFFERENCE_DETECTED=2000;
const int ONE_DIFFERENCE_DETECTED=4000;
const int TWO_DIFFERENCE_DETECTED=3000;
const int MORE_THAN_TWO_DIFFERENCE_DETECTED=5000;
const int EXCEPTION_IN_THE_VISION_LOOP=6000;
const int SIDE_VISION_SUCCESS=7000;
const int SIDE_VISION_UNSUCCESS=8000;
const int GO_LEFT=9000;
const int GO_RIGHT=10000;
const int GO_UP=11000;
const int GO_DOWN=12000;
const int FACE_ALIGNED=13000;
const int FACE_NOT_DETECTED=14000;
const int HAPPY_FACE=15000;
const int UNHAPPY_FACE=16000;
const int NEUTRAL_FACE=17000;


/** get_HRI_trajectory():    
    returns HRI trajectory with respect to the game state

    @param game_status what is the game status between 0-100 (50 is balanced game, lower values are for robot is likely to lose, higher values are for robot is in the lead)
    @param breath do you want the breathing trajectory
    @param hri_client HRI service client
    @return FollowJointTrajectoryGoal resulting trajectory
*/
control_msgs::FollowJointTrajectoryGoal get_HRI_trajectory(int game_status, bool breath, ros::ServiceClient& hri_client)
{
    //call this function to get the trajectory
    franka_msgs::HRI hri_srv_request;
    hri_srv_request.request.breathing = breath;
    hri_srv_request.request.game_status = game_status;
    
    bool resp;
    resp = hri_client.call(hri_srv_request);

    std::cout << "HRI service call is " << resp << std::endl;

    control_msgs::FollowJointTrajectoryGoal result_joint_traj = hri_srv_request.response.trajGoal;

    //retuns the wanted joint trajectory
    return result_joint_traj;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "chessmate");
    ros::NodeHandle n;


    // Test vision is connected or not.
    ros::ServiceClient vision_client = n.serviceClient<chessmate::QueryVisionComponent>("/franka_vision");
    chessmate::QueryVisionComponent vision_srv_request;
    vision_srv_request.request.query_type = "test";
    vision_srv_request.request.last_state_fen_string = fen_string;
    while(!vision_client.call(vision_srv_request)){
        ROS_WARN_STREAM("Vision is not connected to system!");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Vision is connected.");


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

    // the hri service executable is at franka_example_controllers/scripts/hri_component
    ros::ServiceClient hri_client = n.serviceClient<franka_msgs::HRI>("/hri_traj");
    //! example call to hri func
    // control_msgs::FollowJointTrajectoryGoal trajectory_to_follow = get_HRI_trajectory(game_status, want_to_breath, hri_client);

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

        // Init should be updated as side vision place.
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



        std::string movement_in_fen = "";
        vision_srv_request.request.last_state_fen_string = fen_string;
        vision_srv_request.request.query_type = "side";
        resp = vision_client.call(vision_srv_request);


        if(!resp){
            ROS_WARN_STREAM("Vision is not connected to system!");
            ros::Duration(1).sleep();
        } else{
            
            int return_code = vision_srv_request.response.return_code;


            switch (return_code) {
                case SIDE_VISION_SUCCESS:
                    ROS_WARN_STREAM("Side vision is successfull.");
                    movement_in_fen = vision_srv_request.response.movement_in_fen;
                    std::cout << "Detected movement is : " << movement_in_fen << std::endl;
                    break;
                case SIDE_VISION_UNSUCCESS:
                    ROS_WARN_STREAM("Side vision is unsuccessfull.");
                    ros::Duration(1).sleep();
                    break;
                default:
                    ROS_WARN_STREAM("Default in the switch statement. We should not be here.");
                    ros::Duration(1).sleep();
                }
        }

        if (movement_in_fen == "") {
            /*
                Here, we need to go top view to get the prediction from top view.
            */


            vision_srv_request.request.last_state_fen_string = fen_string;
            vision_srv_request.request.query_type = "top";
            resp = vision_client.call(vision_srv_request);
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

        // If player cheats, do something.
        if (chess_opponent_move.response.fen_string == "cheat") {
            ROS_INFO_STREAM("Cheat detected. HRI do angry movement.");
            // HRI will do something here.
        }

        else if (chess_opponent_move.response.game_state == "lose") {
            ROS_INFO_STREAM("We lost the game. HRI do sad movement.");
            // HRI will do something here.
        }

        // If movement is valid, continue..
        else {
            fen_string = chess_opponent_move.response.fen_string;
            ROS_INFO_STREAM("Fen string updated!");
        }


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

        float pick_width = 0.0;
        if (is_put_place_full) {
            // TODO , first take other player's piece.
            char piece_to_take = get_piece_to_take(fen_string, put_place_square);
            
            ROS_WARN_STREAM("YOU ARE TRYING TO TAKE A PIECE BUT IT IS NOT IMPLEMENTED!");
            continue;
        }
        // Otherwise, move the piece to the put place square.
        
        char piece_to_take = get_piece_to_take(fen_string, take_place_square);
        switch(piece_to_take){
            case 'q':
            case ('Q'):{
                ROS_INFO_STREAM("We will take a queen.");
                pick_width = 0.01;
                break;
            }
            case('k'):
            case ('K'):{
                ROS_INFO_STREAM("We will take a king.");
                pick_width = 0.01;
                break;
            }
            default:{
                ROS_INFO_STREAM("We will take others.");
                pick_width = 0.004;
                break;
            }
        }

        // Update fen. Because we will play best move.
        fen_string = chess_next_move.response.fen_string;
        
        // Now, we have the best move, and we can do something with it!
        // Call the vision to get the coordinates of these!
        
        ros::ServiceClient joint_client = n.serviceClient<franka_msgs::SetChessGoal>("/franka_go_chess");
        franka_msgs::SetChessGoal joint_request;
        
        // go to above of take piece 
        joint_request.request.chess_place = take_place_square + "above";
        ROS_INFO_STREAM("Starting movement!");
        
        std::cin >> a;
        std::cout << "exs";
        resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("go successfull");
        
        // prepare gripper
        std::cin >> a;
        gripper_request.request.width = 0.04;
        gripper_request.request.speed = 0.05;
        gripper_request.request.force = 50;
        gripper_request.request.want_to_pick = false;
        gripper_request.request.want_to_move = true;
        gripper_request.request.homing = false;
        gripper_client.call(gripper_request);

        ROS_INFO_STREAM("gripper successfull");

        // go to take piece
        std::cin >> a;
        joint_request.request.chess_place = take_place_square;
        resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in sec go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("sec go successfull");

        // pick piece
        std::cin >> a;
        gripper_request.request.width = pick_width;
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

        // GO TO INTER
        std::cin >> a;
        joint_request.request.chess_place = "inter";
        resp = joint_client.call(joint_request);
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


        /*
            Here, we need to go side vision place instead of init.
            Side vision will take photo and update previous image.
        */
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
        ROS_INFO_STREAM("go to init successfull"); // Should be side vision place. Will be determined.



        // Firstly, check whether we won the game or not.
        if (chess_next_move.response.game_state == "win") {
            ROS_INFO_STREAM("We won the game. HRI do happy movement.");
            // HRI will do something here.
        }


        // If we did not win, go to side vision place and continue the game.
        else {
            /* Here, go to side vision place.*/

            /* Update previous image and go on. */
            vision_srv_request.request.last_state_fen_string = fen_string;
            vision_srv_request.request.query_type = "update_prev";
            resp = vision_client.call(vision_srv_request);

        }

    

        // Here we need to check if HRI part or motion planner part completed their movements.
        // However, I am not sure which way is the best since we did not implement these parts.
        // So I left it here...
    }
    
}















