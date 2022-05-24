import rospy
from camera import Camera
from coordinate import Coordinate
from top_vision import TopVision
from side_vision import SideVision
from face_tracer import FaceTracer
from chessmate.srv import QueryVisionComponentResponse, QueryVisionComponent, getPositionOfPieces, getPositionOfPiecesResponse
from return_codes import *
from functools import partial

A8_X = 0.65078219
A8_Y = 0.19939440
H1_X = 0.32620260
H1_Y = -0.1092154

side_vision_prev_image = None


def camera_release(camera):
    camera.Stop()    



# Since the main loop is in C++, I need to make vision.py a ROS service
if __name__ == "__main__":
    camera = Camera()
    coordinate_class = Coordinate(A8_X, A8_Y, H1_X, H1_Y)
    top_vision = TopVision(camera)
    side_vision = SideVision()
    face_tracer = FaceTracer(camera)



    # init ros node
    rospy.init_node('vision_bridge')


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




    def getPieceCoordinates(req):
        from_coords, to_coords = coordinate_class.get_piece_coordinates(req.from_piece, req.to_piece)
        return getPositionOfPiecesResponse(True, from_coords[0], from_coords[1], to_coords[0], to_coords[1])



    # start a ros service
    rospy.Service('/franka_vision', QueryVisionComponent, queryvisioncomponent_handler)

    rospy.Service('/get_piece_coordinates', getPositionOfPieces, getPieceCoordinates)

    
    rospy.on_shutdown(partial(camera_release, camera))


    rospy.spin()
