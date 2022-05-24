from cgi import test
from cmath import sqrt
import re
from tkinter.tix import X_REGION
from cv2 import circle
import numpy as np


import cv2
from board_localization import BoardLocalization


class Board(BoardLocalization):
    

    
      

    def GetEdges(self,corners):

        h_edges,v_edges =  self.GetEdgesHelper(corners)

        tmp_corners = []
        # 2 to 3
        for corner in corners:
            tmp_corners.append( [int(corner[0]), int(corner[1])] )

        v_edges.append((tmp_corners[3],tmp_corners[2]))
        v_edges.insert(0,(tmp_corners[0],tmp_corners[1]))

        return h_edges,v_edges

    
    def GetSquares(self,edges):

        rows = []


        for left,right in edges:
            row = self.SquarePoints(left,right)
            rows.append(row)

    
        return np.array(rows)




    
    def SquarePoints(self,left,right):


        left_x , left_y = left
        right_x , right_y = right
        
        x_incr = int((right_x - left_x) / 8)
        y_incr = int((right_y - left_y) / 8)

        points = []

        for i in range(9):
            points.append( (left_x + i * x_incr  , left_y + i * y_incr) )

        return points

    

    def Predict(self,img):
        
        board, *_ = self.recognizer.predict(img, self.args.color)
        return board
     

    def GetOccupucancy(self,img):
        return self.recognizer._classify_occupancy(img, chess.WHITE, self.GetCorners(img))


    


    def paint(self , squarest , img , inde):

        corners = np.array([[198,68] , [495,54] , [533,359] , [157,361] ])
        #corners = self.GetCorners(img)
        h_edges,v_edges = self.GetEdges(corners)
        rows = self.GetSquares(v_edges)
        squares = []
        for i in range(8):
            
            for j in range(8):
                diff1 = (rows[i][j] - rows[i][j+1]) / 12
                diff1 = diff1.astype(int)
                diff2 = (rows[i+1][j] - rows[i+1][j+1]) / 12
                diff2 = diff2.astype(int)
                diff3 =  (rows[i][j] - rows[i+1][j]) / 12
                diff3 = diff3.astype(int)
                
                squares.append([rows[i][j] - diff1 - diff3 * 2  , rows[i][j+1] + diff1 * 2 - diff3 * 2  , rows[i+1][j+1] + diff2 * 2 + diff3, rows[i+1][j]] - diff2 + diff3)
                #squares.append([rows[i][j]   , rows[i][j+1]   , rows[i+1][j+1] , rows[i+1][j]] )


        for sq in squarest:
            square = squares[sq]
            k = 0
            for i in square:
                cv2.circle(img,i,5,(k,k,0),1)
                k+=40
        cv2.imwrite("images/" + str(inde) +".png",img)

    def CalculateSquareColors(self,img ):
        
  
  
        corners = np.array([[198,68] , [495,54] , [533,359] , [157,361]])
        #corners = self.GetCorners(img)
        h_edges,v_edges = self.GetEdges(corners)
        rows = self.GetSquares(v_edges)
        squares = []
        for i in range(8):
            
            for j in range(8):
                diff1 = (rows[i][j] - rows[i][j+1]) / 12
                diff1 = diff1.astype(int)
                diff2 = (rows[i+1][j] - rows[i+1][j+1]) / 12
                diff2 = diff2.astype(int)
                diff3 =  (rows[i][j] - rows[i+1][j]) / 12
                diff3 = diff3.astype(int)
                
                squares.append([rows[i][j] - diff1 - diff3 * 2  , rows[i][j+1] + diff1 * 2 - diff3 * 2  , rows[i+1][j+1] + diff2 * 2 + diff3, rows[i+1][j]] - diff2 + diff3)
                #squares.append([rows[i][j]   , rows[i][j+1]   , rows[i+1][j+1] , rows[i+1][j]] )


        square_colors = []

        for square in squares:
            k = 0
            for i in square:
                cv2.circle(img,i,5,(k,k,0),1)
                k+=40
        cv2.imwrite("0.png",img)

        
        

        i = 0
        for square in squares:

            mask = np.zeros(img.shape, dtype=np.uint8)
            roi_corners = np.array([square], dtype=np.int32)
            ignore_mask_color = (255,) * 3
            cv2.fillPoly(mask, roi_corners, ignore_mask_color)
            masked_image = cv2.bitwise_and(img, mask)
            (B, G, R) = cv2.split(masked_image)
            

            square_colors.append(np.array([np.average(B),np.average(G),np.average(R)]))

        
        return np.array(square_colors) , squares



    

    

    

    
        





    
    


        

