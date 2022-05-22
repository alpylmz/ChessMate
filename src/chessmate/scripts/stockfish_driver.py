import rospy
import stockfish
from chessmate.srv import chess_next_move, chess_next_moveResponse
from chessmate.srv import chess_opponent_move, chess_opponent_moveResponse
from chessmate.srv import chess_game_state, chess_game_stateResponse


# need stockfish binary path!   
_stockfish = stockfish.Stockfish(path="/home/alp/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")
#_stockfish = stockfish.Stockfish(path="/home/dogukan/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")


# Set starting state of the board. 
INITIAL_FEN_STRING = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b - - 0 1"
_stockfish.set_fen_position(INITIAL_FEN_STRING)

# Skill level
SKILL_LEVEL = 20
_stockfish.set_skill_level(SKILL_LEVEL)


def chess_next_move_func(req):
    global _stockfish
    best_move = _stockfish.get_best_move()
    _stockfish.make_moves_from_current_position(moves=[best_move])
    print(_stockfish.get_board_visual())
    if _stockfish.get_best_move() is None:
        return chess_next_moveResponse(1,"","","","win")
    else:    
        return chess_next_moveResponse(1, best_move[:2], best_move[2:],_stockfish.get_fen_position(),"continue")

def chess_game_state_func(req):
    global _stockfish
    evaluation = _stockfish.get_wdl_stats()
    print(evaluation)
    return chess_game_stateResponse(True,evaluation[0], evaluation[1], evaluation[2])


def chess_opponent_move_func(req):
    global _stockfish
    opponent_move = req.move
    if _stockfish.is_move_correct(opponent_move):
        _stockfish.make_moves_from_current_position(moves=[opponent_move])
        print(_stockfish.get_board_visual())
        if _stockfish.get_best_move() is None:
            return chess_opponent_moveResponse("","lose")
        else:
            return chess_opponent_moveResponse(_stockfish.get_fen_position(),"continue")

    else:
        print("Illegal move by the player : ", opponent_move)
        return chess_opponent_moveResponse("cheat","continue")

    




if __name__ == "__main__":
    rospy.init_node("stockfish_driver")
    s = rospy.Service("/chess_next_move", chess_next_move, chess_next_move_func)

    s2 = rospy.Service("/chess_opponent_move", chess_opponent_move, chess_opponent_move_func)

    s3 = rospy.Service("/chess_game_state", chess_game_state, chess_game_state_func)

    rospy.spin()

