import rospy
import stockfish
from chessmate.srv import chess_next_move, chess_next_moveResponse
from chessmate.srv import chess_opponent_move, chess_opponent_moveResponse

# need stockfish binary path!
_stockfish = stockfish.Stockfish(path="temp/path")

def chess_next_move_func(req):
    best_move = _stockfish.get_best_move()
    # convert string representation to integer representation
    first_x = ord(best_move[0]) - ord('a')
    first_y = ord(best_move[1]) - ord('1')
    second_x = ord(best_move[2]) - ord('a')
    second_y = ord(best_move[3]) - ord('1')
    return chess_next_moveResponse(1, first_x, first_y, second_x, second_y)

def chess_opponent_move_func(req):
    opponent_move = req.move
    _stockfish.set_position(moves=[opponent_move])
    return chess_opponent_moveResponse()

if __name__ == "__main__":
    rospy.init_node("stockfish_driver")
    s = rospy.Service("/chess_next_move", chess_next_move, chess_next_move_func)

    s2 = rospy.Service("/chess_opponent_move", chess_opponent_move, chess_opponent_move_func)

    rospy.spin()

