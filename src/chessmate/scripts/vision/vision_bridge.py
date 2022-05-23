import rospy
from camera import Camera
from coordinate import Coordinate
from top_vision import TopVision
from side_vision import SideVision
from face_tracer import FaceTracer
from chessmate.srv import QueryVisionComponentResponse, QueryVisionComponent, getPositionOfPieces, getPositionOfPiecesResponse
from return_codes import *

SQUARE_WIDTH = 0.041
TOP_LEFT_X = 0.5410416700640096
TOP_LEFT_Y = 0.320580303627435


def camera_release(camera):
    camera.Stop()    



# Since the main loop is in C++, I need to make vision.py a ROS service
if __name__ == "__main__":
    camera = Camera()
    coordinate_class = Coordinate(TOP_LEFT_X, TOP_LEFT_Y, SQUARE_WIDTH)
    top_vision = TopVision(camera)
    side_vision = SideVision()
    face_tracer = FaceTracer(camera)
    side_vision_prev_image = None


    # init ros node
    rospy.init_node('vision_bridge')


    # queryvisioncomponent handler, executes vision_system.get_movement()
    def queryvisioncomponent_handler(req):
        if req.query_type == "get_emotion":
            return_code = face_tracer.get_emotion()
            return QueryVisionComponentResponse(return_code,"")
            
        if req.query_type == "align_face":
            return_code = face_tracer.is_face_aligned()
            return QueryVisionComponentResponse(return_code, "")

        if req.query_type == "test":
            return QueryVisionComponentResponse(1,"")

        if req.query_type == "update_prev":
            side_vision_prev_image,_,_ = camera.GetImage()
            return QueryVisionComponentResponse(1,"")

        if req.query_type == "side":
            current_image,_,_ = camera.GetImage()
            return_code, movement_in_fen = side_vision.get_movement(current_image,side_vision_prev_image,req.last_state_fen_string)

        if req.query_type == "top":
            return_code, movement_in_fen = top_vision.get_movement(req.last_state_fen_string)



        return QueryVisionComponentResponse(return_code, movement_in_fen)




    def getPieceCoordinates(req):
        from_coords, to_coords = coordinate_class.get_piece_coordinates(req.from_piece, req.to_piece)
        return getPositionOfPiecesResponse(True, from_coords[0], from_coords[1], to_coords[0], to_coords[1])



    # start a ros service
    rospy.Service('/franka_vision', QueryVisionComponent, queryvisioncomponent_handler)

    rospy.Service('/get_piece_coordinates', getPositionOfPieces, getPieceCoordinates)

    
    rospy.on_shutdown(camera_release(camera))


    rospy.spin()