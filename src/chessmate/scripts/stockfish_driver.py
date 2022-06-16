import rospy
import stockfish
from chessmate.srv import chess_next_move, chess_next_moveResponse
from chessmate.srv import chess_opponent_move, chess_opponent_moveResponse
from chessmate.srv import chess_game_state, chess_game_stateResponse


# need stockfish binary path!   
#_stockfish = stockfish.Stockfish(path="/home/alp/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")
_stockfish = stockfish.Stockfish(path="/home/dogukan/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64")


# Set starting state of the board.
INITIAL_FEN_STRING = "3rr1k1/pp3ppp/3b4/2p5/2Q5/6qP/PPP1B1P1/R1B2K1R b KQkq - 0 1"
# hard fen INITIAL_FEN_STRING = "3rr1k1/pp3ppp/3b4/2p5/2Q5/6qP/PPP1B1P1/R1B2K1R b KQkq - 0 1"
# easy fen INITIAL_FEN_STRING = "k5rr/Ppp5/8/4Q3/1P1P4/3q3P/5PP1/R3R1K1 b KQkq - 0 1" 
# basic fen INITIAL_FEN_STRING = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b KQkq - 0 1"
#INITIAL_FEN_STRING = "rnbqkbnr/pppppppp/8/8/8/8/8/4K3 b KQkq - 0 1"

print("initializing with fen string")
_stockfish.set_fen_position(INITIAL_FEN_STRING)
print("initialized fen string")

print("initializing skill level")
# Skill level
SKILL_LEVEL = 20
try:
    _stockfish.set_skill_level(SKILL_LEVEL)
    #_stockfish.update_engine_parameters({"Hash": 2048, "UCI_Chess960": "true","Skill Level": 1}) # Gets stockfish to use a 2GB hash table, and also to play Chess960.
except Exception as e:
    print(e)
    import traceback
    traceback.print_exc()
    print("Stockfish skill level cannot be initialized")
print("initialized skill level")


def chess_next_move_func(req):
    global _stockfish
    best_move = ""
    top_moves = _stockfish.get_top_moves(10)
    
    if SKILL_LEVEL == 1:

        for move in reversed(top_moves):
            if len(move['Move']) == 4 and move['Move'] != "e1g1" and move['Move'] != "e1c1":
                best_move = move['Move']
                break

    if SKILL_LEVEL == 20:
        for move in top_moves:
            if len(move['Move']) == 4 and move['Move'] != "e1g1" and move['Move'] != "e1c1":
                best_move = move['Move']
                break


    _stockfish.make_moves_from_current_position(moves=[best_move])
    print(_stockfish.get_board_visual())
    if _stockfish.get_best_move() is None:
        return chess_next_moveResponse(1,best_move[:2], best_move[2:],"","win")
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

    
    print("this is stockfish driver")
    rospy.init_node("stockfish_driver")
    s = rospy.Service("/chess_next_move", chess_next_move, chess_next_move_func)

    s2 = rospy.Service("/chess_opponent_move", chess_opponent_move, chess_opponent_move_func)

    s3 = rospy.Service("/chess_game_state", chess_game_state, chess_game_state_func)

    rospy.spin()

