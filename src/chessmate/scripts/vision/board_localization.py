import cv2 
import numpy as np
from copy import deepcopy

class BoardLocalization():

    def __init__(self):

        self.ALLCORNERSVERTICAL = []
        self.ALLCORNERSHORIZANTAL = []

    def GetEdgesHelper(self,corners):

        self.ALLCORNERSVERTICAL = []
        self.ALLCORNERSHORIZANTAL = []
        self.FindVertical(corners,3)
        self.FindHorizantal(corners)

        return self.ALLCORNERSHORIZANTAL,self.ALLCORNERSVERTICAL

    def FindHorizantal(self,corners):
        buttom = (corners[2] - corners[3]) / 8
        top  = (corners[1] - corners[0]) / 8

        for k in range(1,8):
            b = corners[3] + buttom * k
            t = corners[0] + top * k

            self.ALLCORNERSHORIZANTAL.append(([int(b[0]) ,int(b[1]) ]   , [ int(t[0]) ,int(t[1])  ] ))

    def FindVertical(self,corners , i):
        left = self.FindVerticalLeft(corners)
        right = self.FindVerticalRight(corners)
        self.ALLCORNERSVERTICAL.append((left,right))
    
        i -= 1
        if i <= 0:
            self.ALLCORNERSVERTICAL.sort(key=lambda x: x[1])

            return
        else:
            corners1 = deepcopy(corners)
            corners2 = deepcopy(corners)

            corners1[0] = left
            corners1[1] = right

            corners2[3] = left
            corners2[2] = right

            self.FindVertical(corners1,i)
            self.FindVertical(corners2,i)

    def FindVerticalLeft(self,corners  ):

        vector2 = corners[0] - corners[3]

        unit_vector_2 = vector2 / np.linalg.norm(vector2)

        oran = np.linalg.norm(corners[2] - corners[3]) / np.linalg.norm(corners[0] - corners[1])

        length = np.linalg.norm(corners[0] - corners[3])

        y = length / (1+oran)

        middle = unit_vector_2 * y

        return [int(corners[0][0] - middle[0]) , int(corners[0][1] - middle[1])]

    def FindVerticalRight(self,corners  ):

        vector2 = corners[1] - corners[2]

        unit_vector_2 = vector2 / np.linalg.norm(vector2)

        oran = np.linalg.norm(corners[2] - corners[3]) / np.linalg.norm(corners[0] - corners[1])

        length = np.linalg.norm(corners[1] - corners[2])

        y = length / (1+oran)

        middle = unit_vector_2 * y
        return [int(corners[1][0] - middle[0]) , int(corners[1][1] - middle[1])]




