import rospy
import stockfish
from chessmate.srv import chess_next_move, chess_next_moveResponse
from chessmate.srv import chess_opponent_move, chess_opponent_moveResponse


# need stockfish binary path!   
_stockfish = stockfish.Stockfish(path="/home/alp/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")
#_stockfish = stockfish.Stockfish(path="/home/dogukan/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")


# Set starting state of the board. 
INITIAL_FEN_STRING = "1k4N1/8/8/2p1K3/B7/3Pb2N/8/2r5 w - - 0 1"
_stockfish.set_fen_position(INITIAL_FEN_STRING)



def chess_next_move_func(req):
    global _stockfish
    best_move = _stockfish.get_best_move()
    _stockfish.make_moves_from_current_position(moves=[best_move])
    return chess_next_moveResponse(1, best_move[:2], best_move[2:],_stockfish.get_fen_position())




def chess_opponent_move_func(req):
    global _stockfish
    opponent_move = req.move
    _stockfish.make_moves_from_current_position(moves=[opponent_move])
    return chess_opponent_moveResponse(_stockfish.get_fen_position())




if __name__ == "__main__":
    rospy.init_node("stockfish_driver")
    s = rospy.Service("/chess_next_move", chess_next_move, chess_next_move_func)

    s2 = rospy.Service("/chess_opponent_move", chess_opponent_move, chess_opponent_move_func)

    rospy.spin()

