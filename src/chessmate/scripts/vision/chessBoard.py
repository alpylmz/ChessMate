from curses.ascii import isdigit
from turtle import pos
import numpy as np
from copy import deepcopy
import chess



class ChessBoard():


    def getBoard(self , fen):
        fen = fen.split(" ")[:-5]
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

    def GetpossibleMoves(self , fen):
        ret = []
        board = chess.Board(fen)
        for legal_move in board.legal_moves:
            board.push(legal_move)
            ret.append((self.getBoard(board.fen()),legal_move))
            board.pop()
        return ret

    def GetSquare(self, board , square):
        return board[int(square / 8)][square % 8]

    def updateSquare(self , array ,square  ):
        if array[int(square / 8)][square % 8] == 1:
            array[int(square / 8)][square % 8] = 0
        else:
            array[int(square / 8)][square % 8] = 1
        
        return array
        
        
    def FindMove(self , squares , fen):
        
        possibleArrays = []
        board_tmp = self.getBoard(fen)

    
        for squaref in squares : 
            square_1 = self.GetSquare(board_tmp,squaref)
            for squaret in squares :
                square_2 = self.GetSquare(board_tmp,squaret)
                if squaret == squaref :
                    continue

                elif square_1 == 0 and square_2 == 1 :
                    tmp = deepcopy(board_tmp)
                    tmp = self.updateSquare(tmp,squaref)
                    tmp = self.updateSquare(tmp,squaret)
                    possibleArrays.append(tmp)
                
                elif square_1 == 1 and square_2 == 1:

                    tmp = deepcopy(board_tmp)
                    tmp = self.updateSquare(tmp,squaret)

                    possibleArrays.append(tmp)

                    tmp1 = deepcopy(board_tmp)
                    tmp1 = self.updateSquare(tmp1,squaref)
            
                    possibleArrays.append(tmp1)
  
         
        

        possibleMoves = self.GetpossibleMoves(fen)

   

        
        ret = []
        
        for arrays in possibleArrays:
            for moves in  possibleMoves:

                if np.all(arrays == moves[0]):
                    add = True
                    for tt in ret:
                        if moves[1] == tt[1]:
                            add = False
                            break
                    if add:
                        ret.append(moves)
             


        
            

        if len(ret) == 1:

            return ret[0][1]

        else:

            print(len(ret))
            return[]


    