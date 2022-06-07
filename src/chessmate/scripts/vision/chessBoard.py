from curses.ascii import isdigit, isupper
from turtle import pos
import numpy as np
from copy import deepcopy
import chess



class ChessBoard():


    def getBoard(self , fen):
        fen = fen.split(" ")[:-4]
        white = fen[1] == "w"
        fen = fen[0]
        fen = fen.split("/")
        ret = np.zeros((8,8))

        
        for i,row in enumerate(fen):
            col = 0
            for p in row:
                if isdigit(p):
                    col += int(p)
                else :
                    if(isupper(p)):
                        ret[i][col] = 1
                    else:
                        ret[i][col] = -1
                    col += 1


        return ret , white

    def GetpossibleMoves(self , fen):
        ret = []
        board = chess.Board(fen)
        for legal_move in board.legal_moves:
            board.push(legal_move)
            ret.append((self.getBoard(board.fen())[0],legal_move))
            board.pop()
        return ret

    def GetSquare(self, board , square):
        return board[int(square / 8)][square % 8]

    def updateSquare(self , array ,square , turn ):
        if array[int(square / 8)][square % 8] == 1 and turn:
            array[int(square / 8)][square % 8] = 0
        
        elif array[int(square / 8)][square % 8] == -1 and turn == False:
            array[int(square / 8)][square % 8] = 0

        else:

            if turn:
                array[int(square / 8)][square % 8] = 1
            else:
                array[int(square / 8)][square % 8] = -1
        
        return array
        
        
    def FindMove(self , squares , fen):
        
        possibleArrays = []
        board_tmp , turn = self.getBoard(fen)



    
        for squaref in squares : 
            square_1 = self.GetSquare(board_tmp,squaref)
            for squaret in squares :
                square_2 = self.GetSquare(board_tmp,squaret)
                if squaret == squaref :
                    continue

                elif (square_1 == 0 and square_2 == 1) or (square_1 == 0 and square_2 == -1)  :
                    tmp = deepcopy(board_tmp)
                    tmp = self.updateSquare(tmp,squaref , turn)
                    tmp = self.updateSquare(tmp,squaret , turn)
                    possibleArrays.append(tmp)
                
                elif (square_1 == 1 and square_2 == -1) or (square_1 == -1 and square_2 == 1):

                    tmp = deepcopy(board_tmp)
                    tmp = self.updateSquare(tmp,squaret , turn)
                    tmp = self.updateSquare(tmp,squaref , turn)

                    
            
                    possibleArrays.append(tmp)
                    
  
         
        

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

            print("error :" , len(ret))

            return[]
