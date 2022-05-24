import rospy
from camera import Camera
from coordinate import Coordinate
from top_vision import TopVision
from side_vision import SideVision
from face_tracer import FaceTracer
from chessmate.srv import QueryVisionComponentResponse, QueryVisionComponent, getPositionOfPieces, getPositionOfPiecesResponse
from return_codes import *
from functools import partial

A8_X = 0.7187915752571079
A8_Y = 0.22051247797150725
H1_X = 0.3984956529830614
H1_Y = -0.10476628814960992



side_vision_prev_image = None


def camera_release(camera):
    camera.Stop()    



# Since the main loop is in C++, I need to make vision.py a ROS service
if __name__ == "__main__":
    coordinate_class = Coordinate(A8_X, A8_Y, H1_X, H1_Y)

    # init ros node
    rospy.init_node('vision_bridge')

    def getPieceCoordinates(req):
        from_coords, to_coords = coordinate_class.get_piece_coordinates(req.from_piece, req.to_piece)
        return getPositionOfPiecesResponse(True, from_coords[0], from_coords[1], to_coords[0], to_coords[1])


    rospy.Service('/get_piece_coordinates', getPositionOfPieces, getPieceCoordinates)


    camera = Camera()
    top_vision = TopVision(camera)
    side_vision = SideVision()
    face_tracer = FaceTracer(camera)


    # queryvisioncomponent handler, executes vision_system.get_movement()
    def queryvisioncomponent_handler(req):
        global side_vision_prev_image
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



    # start a ros service
    rospy.Service('/franka_vision', QueryVisionComponent, queryvisioncomponent_handler)


    
    rospy.on_shutdown(partial(camera_release, camera))

    rospy.spin()
