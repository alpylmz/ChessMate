#include <ros/ros.h>
#include <iostream>
#include <string>

#include "chessmate/helper.hpp"
#include "chessmate_helpers.h"
#include <stdlib.h>
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

std::string fen_string = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b KQkq - 0 1";
//std::string fen_string = "rnbqkbnr/pppppppp/8/8/8/8/8/4K3 b KQkq - 0 1";
 
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
/* Vision function return codes */


const float ARDUINO_CHECK_SLEEP = 2.0;
/* Arduino driver return codes */
const int ROBOT_PLAY=100000;
const int OPPONENT_PLAY=100001;
const int WIN=100002;
const int LOSS=100003;
const int IDLE=100004;
/* Arduino driver return codes */

const float ABOVE_ROBOT_HEIGHT = 0.40;
//const float BOARD_PICK_HEIGHT = 0.275;
//const float BOARD_PLACE_HEIGHT = 0.285;
const float BOARD_PICK_HEIGHT = 0.285;
const float BOARD_PLACE_HEIGHT = 0.290;

const float DUMP_BOX_X = 0.55;
const float DUMP_BOX_Y = 0.41;

const float RELEASE_GRIPPER_WIDTH = 0.038f;

bool definitely_lose = false;

/** get_HRI_trajectory():    
    returns HRI trajectory with respect to the game state

    @param game_status what is the game status, stockfish returns wdl (win-draw-lose) stats
    @param breath do you want the breathing trajectory
    @param hri_client HRI service client
    @return FollowJointTrajectoryGoal resulting trajectory
*/
void get_HRI_trajectory(std::string move, ros::ServiceClient& hri_client)
{
    //call this function to get the trajectory
    franka_msgs::HRI hri_srv_request;
    hri_srv_request.request.move = move;
    
    bool resp;
    resp = hri_client.call(hri_srv_request);

    //std::cout << "HRI service returned " << resp << std::endl;
    ROS_INFO_STREAM("HRI service returned");

    //control_msgs::FollowJointTrajectoryGoal result_joint_traj = hri_srv_request.response.trajGoal;

    //retuns the wanted joint trajectory
   return;
}

void random_hri_move(ros::ServiceClient& hri_client)
{   
    ROS_WARN_STREAM("RANDOM HRI MOVEMENT");

    if(!definitely_lose)
    {
        int rnd = rand() % 3;
        switch (rnd)
        {
        case 0:
            get_HRI_trajectory("think", hri_client);
            break;
        case 1:
            get_HRI_trajectory("positive", hri_client);
            break;
        case 2:
            get_HRI_trajectory("hesitation", hri_client);
            break;
        default:
            break;
        }  
    }
    else
    {
        int rnd = rand() % 4;
        switch (rnd)
        {
        case 0:
            get_HRI_trajectory("think", hri_client);
            break;
        case 1:
            get_HRI_trajectory("positive", hri_client);
            break;
        case 2:
            get_HRI_trajectory("hesitation", hri_client);
            break;
        case 3:
            get_HRI_trajectory("mock", hri_client);
            break;
        default:
            break;
        }  
    }
    
}

int get_lose_chance(ros::ServiceClient& chess_game_state_client)
{
    chessmate::chess_game_state chess_game_state;
    bool resp;
    resp = chess_game_state_client.call(chess_game_state);

    std::cout << "Game state service is returned " << resp << std::endl;
    
    std::cout << "win " << chess_game_state.response.win << std::endl;
    std::cout << "draw " << chess_game_state.response.draw << std::endl;
    std::cout << "lose " << chess_game_state.response.lose << std::endl;

    return chess_game_state.response.lose;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "chessmate");
    ros::NodeHandle n;

    ros::ServiceClient vision_client = n.serviceClient<chessmate::QueryVisionComponent>("/franka_vision");
    chessmate::QueryVisionComponent vision_srv_request;
    // uncomment
    /*
    // Test vision is connected or not.
    vision_srv_request.request.query_type = "test";
    vision_srv_request.request.last_state_fen_string = fen_string;
    while(!vision_client.call(vision_srv_request)){
        ROS_WARN_STREAM("Vision is not connected to system!");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Vision is connected.");
    */

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

    ros::ServiceClient gripper_client = n.serviceClient<franka_gripper::GripperCommand>("/franka_custom_gripper_service");

    ros::ServiceClient chesswatch_client = n.serviceClient<chessmate::chesswatch_serv>("/chess_clock");
    chessmate::chesswatch_serv chesswatch_request;

    ros::ServiceClient trajectory_client = n.serviceClient<franka_msgs::SetTrajectoryCommand>("/franka_trajectory");
    franka_msgs::SetTrajectoryCommand trajectory_request;

    ros::ServiceClient chess_piece_position_client = n.serviceClient<chessmate::getPositionOfPieces>("/get_piece_coordinates");
    chessmate::getPositionOfPieces chess_piece_position_request;


    
    ros::ServiceClient chess_game_state_client = n.serviceClient<chessmate::chess_game_state>("/chess_game_state");
    //! example call to get_win_chance
    // float win_chance = get_win_chance(chess_game_state_client);

    // the hri service executable is at franka_example_controllers/scripts/hri_component
    ros::ServiceClient hri_client = n.serviceClient<franka_msgs::HRI>("/hri_traj");
    //! example call to hri func
    // control_msgs::FollowJointTrajectoryGoal trajectory_to_follow = get_HRI_trajectory(game_status, want_to_breath, hri_client);



    ROS_INFO_STREAM("Main loop starting!");
    bool first_loop = true;
    float rec_opponent_win_prob = 0.5f;
    int no_hri_counter = 0;
    // main control starts here!
    while(true){
        bool resp;
        /*
        std::string dummy_place;
        std::cout << "Enter a place: ";
        std::cin >> dummy_place;
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
        resp = franka_go(go_client, square_position_x, square_position_y, height, false, false, false, true);
        //resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("go successfull");
        continue;
        */



        ROS_INFO_STREAM("Main loop starts!");
        int a;  

        //gripper_move(gripper_client, 0.0, 0.04, 50, false, true, false);
        ROS_INFO_STREAM("gripper successfull");

       
        // check here if franka still on, however I do not know how to do this here TODO

        // error check and cleaning TODO

        // Init should be updated as side vision place.
        //bool resp;
        //std::cin >> a;
        resp = franka_go(go_client, 0, 0, 0, false, false, true, false);
        if (!resp) {
            continue;
        }
        if(first_loop)
        {
            /*while(true)
            {
                random_hri_move(hri_client);
            }*/
            //vision_srv_request.request.last_state_fen_string = fen_string;
            //vision_srv_request.request.query_type = "read_fingers";
            //resp = vision_client.call(vision_srv_request);
            //int finger_count =  vision_srv_request.response.return_code;
            //std::cout << "This is finger count : " << finger_count << std::endl; 

            get_HRI_trajectory("salut", hri_client);
            first_loop = false;        
        }


        /* Here, go to side vision place.*/
        /* Update previous image and go on. */
        vision_srv_request.request.last_state_fen_string = fen_string;
        vision_srv_request.request.query_type = "update_prev";
        resp = vision_client.call(vision_srv_request);


        /* Here arduino driver logic will be written. */
        /* Send signal to arduino to indicate player can play.*/
        ROS_INFO_STREAM("Wait for opponent move.");

        // send signal to arduino to indicate player can play.
        chesswatch_request.request.request = "change";
        resp = chesswatch_client.call(chesswatch_request);
        if(!resp){
            ROS_WARN_STREAM("Error in chesswatch request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("chesswatch request successfull");
        while(true) {
            chesswatch_request.request.request = "get";
            resp = chesswatch_client.call(chesswatch_request);
            if(!resp){
                ROS_WARN_STREAM("Error in chesswatch request");
                ros::Duration(0.01).sleep();
                continue;
            }
            bool flag = false;
            switch (chesswatch_request.response.return_code)
            {
            case ROBOT_PLAY:
                ROS_INFO_STREAM("Now robot's turn");
                flag = true;
                ros::Duration(ARDUINO_CHECK_SLEEP).sleep();
                break;
            case OPPONENT_PLAY:
                get_HRI_trajectory("breath", hri_client);
                break;
            case IDLE:
                ros::Duration(ARDUINO_CHECK_SLEEP).sleep();
                break;
                
            case WIN:
                /* code */
                get_HRI_trajectory("win", hri_client);

                ROS_ERROR_STREAM("You win!");
                return 0;
                break;
            case LOSS:
                /* code */
                get_HRI_trajectory("lose", hri_client);

                ROS_ERROR_STREAM("You lose!");
                return 0;
                break;
            default:
                break;
            }
            if (flag) {
                break;
            }
        }





        std::string movement_in_fen = "";
        vision_srv_request.request.last_state_fen_string = fen_string;
        vision_srv_request.request.query_type = "side";
        resp = vision_client.call(vision_srv_request);

        
        if(!resp){
            ROS_WARN_STREAM("Vision is not connected to system!");
            ros::Duration(1).sleep();
        } else{
            
            int return_code = vision_srv_request.response.return_code;
            std::string rspns;


            switch (return_code) {
                case SIDE_VISION_SUCCESS:
                    ROS_WARN_STREAM("Side vision is successfull.");
                    movement_in_fen = vision_srv_request.response.movement_in_fen;
                    std::cout << "Detected movement is : " << movement_in_fen << std::endl;
                    std::cout << "Please confirm y or n" << std::endl;
                    std::cin >> rspns;
                    if(rspns != "y")
                    {
                        movement_in_fen = "";
                    }
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
            resp = franka_go(go_client, 0, 0, 0, false, true, false, false);
            if (!resp) {
                continue;
            }

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
            get_HRI_trajectory("nod", hri_client);

            //auto trajectory = get_HRI_trajectory(0.0, false ,hri_client);
            //trajectory_request.request.trajGoal = trajectory;
            //trajectory_client.call(trajectory_request);
            // HRI will do something here.
            //continue;
            continue;
        }

        else if (chess_opponent_move.response.game_state == "lose") {
            ROS_INFO_STREAM("We lost the game. HRI do sad movement.");
            get_HRI_trajectory("lose", hri_client);
            return 0;
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

        // Otherwise, move the piece to the put place square.
        
        float pick_width = 0.0;

        // Now, we have the best move, and we can do something with it!
        // Call the vision to get the coordinates of these!
        
        //! HRI DECIDE MOVEMENT

        int opponent_lose_prob = get_lose_chance(chess_game_state_client);

        if(opponent_lose_prob > 900 && !definitely_lose)
        {
            definitely_lose = true;
            get_HRI_trajectory("mock",hri_client);
        }
        else
        {
            if(no_hri_counter == 1)
            {
                random_hri_move(hri_client);
                no_hri_counter = 0;
            }
            else
            {
                no_hri_counter++;
            }
        }

        


        //!-----------------------
        
        // go to above of take piece 
        //joint_request.request.chess_place = take_place_square + "above";
        ROS_INFO_STREAM("Starting movement!");
        //std::cin >> a;
        

        chess_piece_position_request.request.from_piece = take_place_square;
        chess_piece_position_request.request.to_piece = put_place_square;
        chess_piece_position_client.call(chess_piece_position_request);
        float take_place_square_position_x = chess_piece_position_request.response.from_x;
        float take_place_square_position_y = chess_piece_position_request.response.from_y;
        float put_place_square_position_x = chess_piece_position_request.response.to_x;
        float put_place_square_position_y = chess_piece_position_request.response.to_y;


        // First, we need to look at put place square.
        // If it is full, then we will eat that piece and then do the movement. 
        bool is_put_place_full = is_square_full(fen_string, put_place_square);

        if (is_put_place_full) {
            char piece_to_take = get_piece_to_take(fen_string, put_place_square);
            switch(piece_to_take){
            case 'q':
            case ('Q'):{
                ROS_INFO_STREAM("We will take a queen.");
                pick_width = 0.008;
                break;
            }
            case('k'):
            case ('K'):{
                ROS_INFO_STREAM("We will take a king.");
                pick_width = 0.008;
                break;
            }
            default:{
                ROS_INFO_STREAM("We will take others.");
                pick_width = 0.004;
                break;
            }
        }
            // TODO , first take other player's piece.
            //char piece_to_take = get_piece_to_take(fen_string, put_place_square);



            ROS_INFO_STREAM("Put place is full. We will take the piece and then do the movement.");
            //std::cin >> a;
            //joint_request.request.chess_place = put_place_square + "above";
            //resp = joint_client.call(joint_request);
            resp = franka_go(go_client, put_place_square_position_x, put_place_square_position_y, ABOVE_ROBOT_HEIGHT, false, false, false, true);
            if(!resp){
                ROS_WARN_STREAM("Error in fourth go request");
                ros::Duration(0.01).sleep();
                continue;
            }
            ROS_INFO_STREAM("foourth go successfull");


            gripper_move(gripper_client, RELEASE_GRIPPER_WIDTH, 0.05, 50, false, true, false);
            
            //std::cin >> a;
            //joint_request.request.chess_place = put_place_square;
            //resp = joint_client.call(joint_request);
            resp = franka_go(go_client, put_place_square_position_x, put_place_square_position_y, BOARD_PICK_HEIGHT, false, false, false, false);
            if(!resp){
                ROS_WARN_STREAM("Error in fifth go request");
                ros::Duration(0.01).sleep();
                continue;
            }
            ROS_INFO_STREAM("fifth go successfull");

            //std::cin >> a;
            gripper_move(gripper_client, pick_width, 0.05, 50, true, false, false);
            ROS_INFO_STREAM(" gripper pick  successfull");
            ros::Duration(1).sleep();
            //std::cin >> a;

            //joint_request.request.chess_place = put_place_square;
            //resp = joint_client.call(joint_request);
            resp = franka_go(go_client, put_place_square_position_x, put_place_square_position_y, ABOVE_ROBOT_HEIGHT, false, false, false, false);
            if(!resp){
                ROS_WARN_STREAM("Error in above go request");
                ros::Duration(0.01).sleep();
                continue;
            }
            ROS_INFO_STREAM("above go successfull");

            //std::cin >> a;
            //joint_request.request.chess_place = put_place_square;
            //resp = joint_client.call(joint_request);
            resp = franka_go(go_client, DUMP_BOX_X, DUMP_BOX_Y, ABOVE_ROBOT_HEIGHT, false, false, false, false);
            if(!resp){
                ROS_WARN_STREAM("Error in above go request");
                ros::Duration(0.01).sleep();
                continue;
            }
            ROS_INFO_STREAM("dump above go successfull");

            ROS_INFO_STREAM("DUMPING THE PIECE. ARE YOU READY?");
            //std::cin >> a;

            gripper_move(gripper_client, RELEASE_GRIPPER_WIDTH, 0.05, 50, false, true, false);
            
            ROS_WARN_STREAM("DUMPED THE PIECE SUCCESSFULLY");
            //ROS_WARN_STREAM("YOU ARE TRYING TO TAKE A PIECE BUT IT IS NOT IMPLEMENTED!");
        }
        
        char piece_to_take = get_piece_to_take(fen_string, take_place_square);
        switch(piece_to_take){
            case 'q':
            case ('Q'):{
                ROS_INFO_STREAM("We will take a queen.");
                pick_width = 0.008;
                break;
            }
            case('k'):
            case ('K'):{
                ROS_INFO_STREAM("We will take a king.");
                pick_width = 0.008;
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

        if (is_put_place_full){
            resp = franka_go(go_client, take_place_square_position_x, take_place_square_position_y, ABOVE_ROBOT_HEIGHT, false, false, false, false); 
        }
        else{
            resp = franka_go(go_client, take_place_square_position_x, take_place_square_position_y, ABOVE_ROBOT_HEIGHT, false, false, false, true);
        }
        //resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in first go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("go successfull");
        
        // prepare gripper
        //std::cin >> a;
        gripper_move(gripper_client, RELEASE_GRIPPER_WIDTH, 0.05, 50, false, true, false);

        ROS_INFO_STREAM("gripper successfull");

        // go to take piece
        //std::cin >> a;
        //joint_request.request.chess_place = take_place_square;
        //resp = joint_client.call(joint_request);
        resp = franka_go(go_client, take_place_square_position_x, take_place_square_position_y, BOARD_PICK_HEIGHT, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in sec go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("sec go successfull");

        // pick piece
        //std::cin >> a;
        gripper_move(gripper_client, pick_width, 0.05, 50, true, false, false);
        ROS_INFO_STREAM("sec gripper successfull");
        ros::Duration(1).sleep();

        // lift chess piece
        //std::cin >> a;
        //joint_request.request.chess_place = take_place_square + "above";
        //resp = joint_client.call(joint_request);
        resp = franka_go(go_client, take_place_square_position_x, take_place_square_position_y, ABOVE_ROBOT_HEIGHT, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in thirdy go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        

        /*
        // GO TO INTER
        //std::cin >> a;
        joint_request.request.chess_place = "inter";
        resp = joint_client.call(joint_request);
        if(!resp){
            ROS_WARN_STREAM("Error in third go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("third go successfull");
        */

        //std::cin >> a;
        //joint_request.request.chess_place = put_place_square + "above";
        //resp = joint_client.call(joint_request);
        resp = franka_go(go_client, put_place_square_position_x, put_place_square_position_y, ABOVE_ROBOT_HEIGHT, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in fourth go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("foourth go successfull");

        
        //std::cin >> a;
        //joint_request.request.chess_place = put_place_square;
        //resp = joint_client.call(joint_request);
        resp = franka_go(go_client, put_place_square_position_x, put_place_square_position_y, BOARD_PLACE_HEIGHT, false, false, false, false);
        if(!resp){
            ROS_WARN_STREAM("Error in fifth go request");
            ros::Duration(0.01).sleep();
            continue;
        }
        ROS_INFO_STREAM("fifth go successfull");

        //std::cin >> a;
        gripper_move(gripper_client, RELEASE_GRIPPER_WIDTH, 0.05, 50, false, true, false);
        ROS_INFO_STREAM(" gripper release  successfull");
        //std::cin >> a;


        
        // Firstly, check whether we won the game or not.
        if (chess_next_move.response.game_state == "win") {
            ROS_INFO_STREAM("We won the game. HRI do happy movement.");
            get_HRI_trajectory("win", hri_client);
            return 0;
            
            // HRI will do something here.
        }



        // Here we need to check if HRI part or motion planner part completed their movements.
        // However, I am not sure which way is the best since we did not implement these parts.
        // So I left it here...
    }
    
}















