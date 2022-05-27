import re
import numpy as np
import os
import cv2
from cgi import test
from cmath import sqrt
from board_localization import BoardLocalization
from tkinter.tix import X_REGION
from cv2 import circle
from vision_calibration_parameters import *


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

        corners = np.array([SIDE_VISION_TOP_LEFT, SIDE_VISION_TOP_RIGHT, SIDE_VISION_BOTTOM_RIGHT, SIDE_VISION_BOTTOM_LEFT])
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
        current_dir = os.path.dirname(os.path.realpath(__file__))
        #cv2.imwrite(current_dir + "/images/" + str(inde) +".png",img)

    def CalculateSquareColors(self,img ):
        
  
  
        corners = np.array([SIDE_VISION_TOP_LEFT, SIDE_VISION_TOP_RIGHT, SIDE_VISION_BOTTOM_RIGHT, SIDE_VISION_BOTTOM_LEFT])
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
        current_dir = os.path.dirname(os.path.realpath(__file__))        
        #cv2.imwrite(current_dir + "/0.png",img)

        
        

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



    

    

    

    
        





    
    


        

