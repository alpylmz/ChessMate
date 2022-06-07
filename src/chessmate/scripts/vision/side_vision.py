from board import Board
from chessBoard import ChessBoard
from return_codes import *
import numpy as np
import cv2
import time
import copy



class SideVision(Board):


    def __init__(self):
        self.cB = ChessBoard() 
        self.rook = True


    
    def get_movement(self , curr_image , prev_image , fen):
        curr_state , _ = self.CalculateSquareColors(curr_image)
        prev_state , _ = self.CalculateSquareColors(prev_image)
        diff1 , diff2 , diff3 = self.CalculateDiff(curr_state , prev_state)

        print(diff2 , diff1 , diff3)
        if self.cB.FindMove(diff2 , fen) != []:

            move = str(self.cB.FindMove(diff2 , fen))
            if (move == "h8f8" or move == "a8d8" or move == "h8g8") and self.rook:
                move = ""
                self.rook = False

            return SIDE_VISION_SUCCESS, str(self.cB.FindMove(diff2 , fen))
        if self.cB.FindMove(diff1 , fen) != []:
            move = str(self.cB.FindMove(diff2 , fen))
            if (move == "h8f8" or move == "a8d8" or move == "h8g8") and self.rook:
                move = ""
                self.rook = False
            return SIDE_VISION_SUCCESS, str(self.cB.FindMove(diff1 , fen))
        if self.cB.FindMove(diff3 , fen) != []:
            move = str(self.cB.FindMove(diff2 , fen))
            if (move == "h8f8" or move == "a8d8" or move == "h8g8") and self.rook:
                move = ""
                self.rook = False
            return SIDE_VISION_SUCCESS, str(self.cB.FindMove(diff3 , fen))
        else :
            return SIDE_VISION_UNSUCCESS, ""



    def CalculateDiff(self , curr_state , prev_state):
        diff = np.average(abs(curr_state - prev_state),axis = 1)
        diff2 = np.average(abs(curr_state - prev_state),axis = 1)
        diff3 = np.average(abs(curr_state - prev_state),axis = 1)
        

        ret = []
        for i in range(4):
            max = np.argmax(diff)
            print(max , diff[max])


            if diff[max] < 0.03 and len(ret) > 1:
                break
            
            ret.append(max)
            diff[max] = 0
        
        ret.sort(reverse= True)
        print("asd" , ret)
        return ret , self.CalculateDiff2(copy.deepcopy(ret),diff2),self.CalculateDiff3(diff3)



    def CalculateDiff2(self , ret , diff2):
        
        end = True
        while end:
            end = False
            for i in ret:
                if i + 8 in ret :
                    ret.remove(i)
                    end = True
                    break
        return ret
   
   
   
    def CalculateDiff3(self , diff):
        ret = []
        for i in range(4):
            max = np.argmax(diff)
            ret.append(max)
            diff[max] = 0


        return ret 



    def calibration(self, img):
        corners = np.array([[600,185] , [1250,180] , [1360,820] , [505,840] ])

        
            
        for i in corners:
            cv2.circle(img,i,5,(255,0,0),1)
        cv2.imshow("test",img)



    def saveGame(self , img, path ,i):
        
        cv2.imwrite(path + str(i) + ".png", img)

            
        

    



        