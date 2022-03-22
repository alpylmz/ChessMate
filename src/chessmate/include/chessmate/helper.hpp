#include <ros/ros.h>
#include "chessmate/see_chessboard.h"

// This function waits until chessboard can be seen
// if chessboard cannot be seen,
 
void canSeeChessboard(){
    ros::ServiceClient can_see_chessboard_client = n.serviceClient<chessmate::see_chessboard>("/can_see_chessboard");
    while(!can_see_chessboard_client.call(can_see_chessboard)){
        ROS_WARN_STREAM("Cannot see chessboard!");
        ros::Duration(1).sleep();
    }
}

// This function searches until chessboard is found
void searchChessboard(){
    chessmate::see_chessboard can_see_chessboard;
    chessmate::pose chessboard_left_bottom_pose;
    chessmate::pose chessboard_left_top_pose;
    chessmate::pose chessboard_right_bottom_pose;
    chessmate::pose chessboard_right_top_pose;
    bool rotate_orientation = true;
    
    // If the client call is not successfull
    // Assume that the vision system is not ready yet

    while(true){
        while(!can_see_chessboard_client.call(can_see_chessboard)){
            ROS_WARN_STREAM("Vision system is not open yet or sth is very wrong!");
            ros::Duration(1).sleep();
        }
        if(can_see_chessboard.response.can_see_chessboard == 0){
            chessboard_left_bottom_pose  = can_see_chessboard.response.chessboard_left_bottom_pose;
            chessboard_left_top_pose     = can_see_chessboard.response.chessboard_left_top_pose;
            chessboard_right_bottom_pose = can_see_chessboard.response.chessboard_right_bottom_pose;
            chessboard_right_top_pose    = can_see_chessboard.response.chessboard_right_top_pose;
            break;
        }
        else if(can_see_chessboard.response.can_see_chessboard == 1){
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
}

struct PandaJoints{
    float joint0;
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
};