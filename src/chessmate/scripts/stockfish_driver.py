import rospy
import stockfish
from chessmate.srv import chess_next_move, chess_next_moveResponse
from chessmate.srv import chess_opponent_move, chess_opponent_moveResponse

# need stockfish binary path!
_stockfish = stockfish.Stockfish(path="/home/alp/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")

def chess_next_move_func(req):
    global _stockfish
    best_move = _stockfish.get_best_move()
    return chess_next_moveResponse(1, best_move[:2], best_move[2:])

def chess_opponent_move_func(req):
    global _stockfish
    opponent_move = req.move
    _stockfish.set_position(moves=[opponent_move])
    return chess_opponent_moveResponse()

if __name__ == "__main__":
    rospy.init_node("stockfish_driver")
    s = rospy.Service("/chess_next_move", chess_next_move, chess_next_move_func)

    s2 = rospy.Service("/chess_opponent_move", chess_opponent_move, chess_opponent_move_func)

    rospy.spin()

