import rospy
import cv2
import os
from camera import Camera
from coordinate import Coordinate
from top_vision import TopVision
from side_vision import SideVision
#from face_tracer import FaceTracer
from chessmate.srv import QueryVisionComponentResponse, QueryVisionComponent, getPositionOfPieces, getPositionOfPiecesResponse
from return_codes import *
from functools import partial

A1_X = 0.4014470920338918
A1_Y = 0.22229826375982784
A8_X = 0.7272811452210761
A8_Y = 0.22010953155514001
H1_X = 0.40305517526720513
H1_Y = -0.09984058193422411
H8_X = 0.7263877184058034
H8_Y = -0.10278467775327396




current_dir = os.path.dirname(os.path.realpath(__file__))
MISPREDICTED_IMAGE_PATH = current_dir + "/mispredicted_images/"
mispredicted_image_counter = 0
side_vision_prev_image = None


def camera_release(camera):
    camera.Stop()    



# Since the main loop is in C++, I need to make vision.py a ROS service
if __name__ == "__main__":
    print("this is vision bridge")
    coordinate_class = Coordinate(A1_X, A1_Y, A8_X, A8_Y, H1_X, H1_Y, H8_X, H8_Y)

    # init ros node8
    rospy.init_node('vision_bridge')

    def getPieceCoordinates(req):
        from_coords, to_coords = coordinate_class.get_piece_coordinates(req.from_piece, req.to_piece)
        return getPositionOfPiecesResponse(True, from_coords[0], from_coords[1], to_coords[0], to_coords[1])


    rospy.Service('/get_piece_coordinates', getPositionOfPieces, getPieceCoordinates)


    camera = Camera()
    top_vision = TopVision(camera)
    side_vision = SideVision()
    #face_tracer = FaceTracer(camera)


    # queryvisioncomponent handler, executes vision_system.get_movement()
    def queryvisioncomponent_handler(req):
        global side_vision_prev_image
        global MISPREDICTED_IMAGE_PATH
        global mispredicted_image_counter
        #if req.query_type == "get_emotion":
        #    return_code = face_tracer.get_emotion()
        #    return QueryVisionComponentResponse(return_code,"")
            
        #if req.query_type == "align_face":
        #    return_code = face_tracer.is_face_aligned()
        #    return QueryVisionComponentResponse(return_code, "")

        if req.query_type == "test":
            return QueryVisionComponentResponse(1,"")

        if req.query_type == "update_prev":
            side_vision_prev_image,_,_ = camera.GetImage()
            return QueryVisionComponentResponse(1,"")

        if req.query_type == "side":
            current_image,_,_ = camera.GetImage()
            return_code, movement_in_fen = side_vision.get_movement(current_image,side_vision_prev_image,req.last_state_fen_string)
            if return_code == SIDE_VISION_UNSUCCESS:
                cv2.imwrite(MISPREDICTED_IMAGE_PATH + "/image-" + str(mispredicted_image_counter) + ".png", side_vision_prev_image)
                mispredicted_image_counter += 1
                cv2.imwrite(MISPREDICTED_IMAGE_PATH + "/image-" + str(mispredicted_image_counter) + ".png", current_image)
                mispredicted_image_counter += 1


        if req.query_type == "top":
            current_image,_,_ = camera.GetImage()
            return_code, movement_in_fen = top_vision.get_movement(req.last_state_fen_string)
            if return_code != TWO_DIFFERENCE_DETECTED:
                cv2.imwrite(MISPREDICTED_IMAGE_PATH + "/image-" + str(mispredicted_image_counter) + ".png", current_image)
                mispredicted_image_counter += 1


        return QueryVisionComponentResponse(return_code, movement_in_fen)



    # start a ros service
    rospy.Service('/franka_vision', QueryVisionComponent, queryvisioncomponent_handler)


    
    rospy.on_shutdown(partial(camera_release, camera))

    rospy.spin()
