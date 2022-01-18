import rospy

from chessmate.srv import franka_on, franka_onResponse
from chessmate.srv import franka_control_start_stop, franka_control_start_stopResponse
from chessmate.srv import chess_next_move, chess_next_moveResponse
from chessmate.srv import next_move, next_moveResponse
from chessmate.srv import pick_and_place, pick_and_placeResponse
from chessmate.srv import see_chessboard, see_chessboardResponse

from chessmate.msg import pose

rospy.init_node('mock_services')

def franka_on_func(req):
    return franka_onResponse()

s1 = rospy.Service("/franka_on", franka_on, franka_on_func)

def franka_control_start_stop_func(req):
    return franka_control_start_stopResponse()

s2 = rospy.Service("/franka_control_start_stop", franka_control_start_stop, franka_control_start_stop_func)

def can_see_chessboard_func(req):
    b = see_chessboardResponse()
    return b

s3 = rospy.Service("/can_see_chessboard", see_chessboard, can_see_chessboard_func)

def chess_next_move_func(req):
    print("chess next move call done")
    return chess_next_moveResponse(1, 0, 0, 0, 0)

s4 = rospy.Service("/chess_next_move", chess_next_move, chess_next_move_func)

def next_move_func(req):
    print("next move call done")
    return next_moveResponse(0, 0, 0, 0, 0, 0)

s5 = rospy.Service("/next_move", next_move, next_move_func)

def pick_and_place_func(req):
    print("pick and place call done")
    return pick_and_placeResponse()

s6 = rospy.Service("/pick_and_place", pick_and_place, pick_and_place_func)

rospy.spin()