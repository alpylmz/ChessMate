#include <ros/ros.h>
#include "chessmate/see_chessboard.h"
#include "chessmate/QueryVisionComponent.h"

/*
// This function searches until chessboard is found
void searchChessboard(ros::NodeHandle n){
    ros::ServiceClient vision_client = n.serviceClient<franka_msgs::QueryVisionComponent>("/franka_vision/query_component");
    franka_msgs::QueryVisionComponent vision_srv_request;
    vision_srv_request.request.fen_string = fen_string;
    bool rotate_orientation = true;
    
    while(true){
        while(!can_see_chessboard_client.call(vision_srv_request)){
            ROS_WARN_STREAM("Vision system is not open yet or sth is very wrong!");
            ros::Duration(1).sleep();
        }
        // need to change can_see_chessboard!!!!!!!!!
        if(can_see_chessboard.response.can_see_chessboard == 0){
            break;
        }
        // need to change can_see_chessboard!!!!!!!
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
*/
struct PandaJoints{
    float joint0;
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
};