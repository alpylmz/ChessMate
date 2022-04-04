import rospy

from vision import Vision
from chessmate.srv import QueryVisionComponentResponse, QueryVisionComponent, getPositionOfPieces, getPositionOfPiecesResponse

TOP_LEFT_X = 0.5410416700640096
TOP_LEFT_Y = 0.320580303627435

# Since the main loop is in C++, I need to make vision.py a ROS service
if __name__ == "__main__":
        
    vision_system = Vision(TOP_LEFT_X, TOP_LEFT_Y)

    # init ros node
    rospy.init_node('vision_bridge')

    # queryvisioncomponent handler, executes vision_system.get_movement()
    def queryvisioncomponent_handler(req):
        can_see_chessboard, is_there_movement, movement_in_fen = vision_system.get_movement(req.last_state_fen_string)
        return QueryVisionComponentResponse(True, can_see_chessboard, is_there_movement, movement_in_fen)

    def getPieceCoordinates(req):
        from_coords, to_coords = vision_system.get_piece_coordinates(req.from_piece, req.to_piece)
        return getPositionOfPiecesResponse(True, from_coords[0], from_coords[1], to_coords[0], to_coords[1])

    # start a ros service
    rospy.Service('/franka_vision', QueryVisionComponent, queryvisioncomponent_handler)

    rospy.Service('/get_piece_coordinates', getPositionOfPieces, getPieceCoordinates)

    rospy.spin()