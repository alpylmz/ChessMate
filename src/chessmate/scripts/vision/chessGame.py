from curses.ascii import isdigit
import chess
import numpy as np

class chessGame():
    def __init__(self):
        self.board = chess.Board()

    def getBoard(self):
        fen = self.board.fen().split(" ")[:-5]
        fen = fen[0].split("/")
        ret = np.zeros((8,8))

        for i,row in enumerate(fen):
            col = 0
            for p in row:
                if isdigit(p):
                    col += int(p)
                else :
                    
                    ret[i][col] = 1
                    col += 1

        
        return ret

    def possibleMoves(self):
        ret = []
        for legal_move in self.board.legal_moves:
            self.board.push(legal_move)
            ret.append((self.getBoard(),legal_move))
            self.board.pop()
        return ret



