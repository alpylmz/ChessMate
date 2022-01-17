#include <ros/ros.h>
#include "chessmate/franka_on.h"
#include "chessmate/see_chessboard.h"
#include "chessmate/pose.h"
#include "chessmate/helper.h"


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



}