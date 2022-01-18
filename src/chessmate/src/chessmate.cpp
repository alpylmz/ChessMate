#include <ros/ros.h>
#include "chessmate/helper.h"

// messages
#include "chessmate/pose.h"

// services
#include "chessmate/franka_on.h"
#include "chessmate/see_chessboard.h"
#include "chessmate/franka_control_start_stop.h"
#include "chessmate/chess_next_move.h"
#include "chessmate/next_move.h"
#include "chessmate/pick_and_place.h"



int main(int argc, char** argv){
    ros::init(argc, argv, "chessmate");
    ros::NodeHandle n;

    // checks if the system is open, with "/franka_on" service!
    ros::ServiceClient robot_on_client = n.serviceClient<chessmate::franka_on>("/franka_on");
    chessmate::franka_on franka_is_on_client;
    while(!robot_on_client.call(franka_is_on_client)){
        ROS_WARN_STREAM("Franka is not connected to system!");
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Franka is connected.");
    
    // Here franka_control package's franka_control node should be started
    // or we can get into the source code of it, and open and close it with a service, I wrote it here
    // Another solution may be getting that necessary code from that node to here, but I think it will be more clear when we really use this code with robot
    // So I only left this suggestion here

    ros::ServiceClient start_or_stop_franka_control_node_client = n.serviceClient<chessmate::franka_control_start_stop>("/franka_control_start_stop")
    chessmate::franka_control_start_stop franka_control_start;
    franka_control_start.start_or_stop = 0;
    while(!start_or_stop_franka_control_node_client.call(franka_control_start)){
        ROS_WARN_STREAM("Franka control node cannot be started.")
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Franka control is started. You can get robot information now.");


    // First, do not give commands to external things.
    PandaJoints neutral_pose;
    // assign the robot to neutral pose, or save it here and give it!
    float joint0_start = neutral_pose.joint0;

    // check if we can see the chessboard, to initialize!
    ros::ServiceClient can_see_chessboard_client = n.serviceClient<chessmate::see_chessboard>("/can_see_chessboard");
    chessmate::see_chessboard can_see_chessboard;
    chessmate::pose chessboard_left_bottom_pose;
    chessmate::pose chessboard_left_top_pose;
    chessmate::pose chessboard_right_bottom_pose;
    chessmate::pose chessboard_right_top_pose;
    
    // Search for chessboard
    bool rotate_orientation = true;
    while(true){
        auto call_success = can_see_chessboard_client.call(can_see_chessboard);
        
        if(!call_success){
            ROS_WARN_STREAM("Vision system is not open yet!");
            ros::Duration(1).sleep();
        }
        if(can_see_chessboard.response.can_see_chessboard == 1){
            chessboard_left_bottom_pose  = can_see_chessboard.response.chessboard_left_bottom_pose;
            chessboard_left_top_pose     = can_see_chessboard.response.chessboard_left_top_pose;
            chessboard_right_bottom_pose = can_see_chessboard.response.chessboard_right_bottom_pose;
            chessboard_right_top_pose    = can_see_chessboard.response.chessboard_right_top_pose;
            break;
        }
        else if(can_see_chessboard.response.can_see_chessboard == 0){
            ROS_WARN_STREAM("Cannot find the chessboard yet!");
            
            // send following info to command topic!
            if(rotate_orientation){
                if(neutral_pose.joint0 < 0){
                    // start rotating to other side
                    rotate_orientation = !rotate_orientation;
                }
                else{
                    neutral_pose.joint0 -= 1;
                    // send here!
                }
            }
            else{
                if(neutral_pose.joint0 > 360){
                    // start rotating to other side
                    rotate_orientation = !rotate_orientation;
                }
                else{
                    neutral_pose.joint0 += 1;
                    // send here!
                }
            }

            ros::Duration(0.2).sleep();
        }
        
    }

    // from here, neutral pose is the pose that looks to the chessboard, will be used later!!!!

    // main control loop preparations
    ros::ServiceClient chess_next_move_client = n.serviceClient<chessmate::chess_next_move>("/chess_next_move");
    chessmate::chess_next_move chess_move;
    ros::ServiceClient next_move_client = n.serviceClient<chessmate::next_move>("/next_move");
    chessmate::next_move next_move;
    ros::ServiceClient pick_and_place_client = n.serviceClient<chessmate::pick_and_place>("/pick_and_place");
    chessmate::pick_and_place pick_and_place;

    // main control starts here!
    while(true){
        // check here if franka still on, however I do not know how to do this here TODO

        // error check and cleaning TODO

        // I assume that the stockfish node takes the chessboard info itself
        while(!chess_next_move_client.call(chess_move)){
            ROS_WARN_STREAM("Chess next move call unsuccessful!");
            ros::Duration(0.01).sleep();
        }

        // If player cheated, send the info to the HRI, and let HRI to finish its movement!!
        // If player plays correctly, send the info to vision, get the coordinates, send them to motion planner node
        // USER CHEATED!
        if(chess_move.is_state_valid == 0){
            ROS_WARN_STREAM("Player cheated!");
            pick_and_place.is_something_hacky = 1;
            while(!pick_and_place_client.call(pick_and_place)){
                ROS_WARN_STREAM("Pick and place call unsuccessful!");
                ros::Duration(0.01).sleep();
            }
        }
        else{
            next_move.take_place_x = chess_move.take_place_x;
            next_move.take_place_y = chess_move.take_place_y;
            next_move.put_place_x = chess_move.put_place_x;
            next_move.put_place_y = chess_move.put_place_y;
            while(!next_move_client.call(next_move)){
                ROS_WARN_STREAM("Next move call unsuccessful!");
                ros::Duration(0.01).sleep();
            }

            pick_and_place.take_coord_x = next_move.take_coord_x;
            pick_and_place.take_coord_y = next_move.take_coord_y;
            pick_and_place.take_coord_z = next_move.take_coord_z;
            pick_and_place.put_coord_x = next_move.put_coord_x;
            pick_and_place.put_coord_y = next_move.put_coord_y;
            pick_and_place.put_coord_z = next_move.put_coord_z;
            pick_and_place.is_something_hacky = 0;
            while(!pick_and_place_client.call(pick_and_place)){
                ROS_WARN_STREAM("Pick and place call unsuccessful!");
                ros::Duration(0.01).sleep();
            }
        }

        // Here we need to check if HRI part or motion planner part completed their movements.
        // However, I am not sure which way is the best since we did not implement these parts.
        // So I left it here...
    }


}