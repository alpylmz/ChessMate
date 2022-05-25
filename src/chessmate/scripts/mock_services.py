import rospy

from chessmate.srv import franka_on, franka_onResponse
from chessmate.srv import franka_control_start_stop, franka_control_start_stopResponse
from chessmate.srv import next_move, next_moveResponse
from chessmate.srv import pick_and_place, pick_and_placeResponse
from chessmate.srv import see_chessboard, see_chessboardResponse
from franka_msgs.srv import SetPositionCommand, SetPositionCommandResponse

from chessmate.msg import pose

rospy.init_node('mock_services')
rospy.wait_for_service("/franka_go")
franka_go = rospy.ServiceProxy("/franka_go", SetPositionCommand)

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


def next_move_func(req):
    print("next move call done")
    return next_moveResponse(0, 0, 0, 0, 0, 0)

s5 = rospy.Service("/next_move", next_move, next_move_func)

def pick_and_place_func(req):
    # later on, this will be a call to the real pick and place service
    # for now I just used a mock call, with franka_go!!
    # request /franka_go service with message type SetPositionCommand
    franka_go.call(SetPositionCommand(req.x, req.y, req.z))

    print("pick and place call done")
    return pick_and_placeResponse()

s6 = rospy.Service("/pick_and_place", pick_and_place, pick_and_place_func)

rospy.spin()