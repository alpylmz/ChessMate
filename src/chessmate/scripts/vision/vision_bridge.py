import rospy

from vision import Vision
from chessmate.srv import QueryVisionComponentResponse, QueryVisionComponent


# Since the main loop is in C++, I need to make vision.py a ROS service
if __name__ == "__main__":
        
    vision_system = Vision()

    # init ros node
    rospy.init_node('vision_bridge')


    # queryvisioncomponent handler, executes vision_system.get_movement()
    def queryvisioncomponent_handler(req):
        can_see_chessboard, is_there_movement, movement_in_fen = vision_system.get_movement(req.fen_string)
        return QueryVisionComponentResponse(True, can_see_chessboard, is_there_movement, movement_in_fen)

    # start a ros service
    rospy.Service('/franka_vision', QueryVisionComponent, vision_system.query_vision_component)

    rospy.spin()