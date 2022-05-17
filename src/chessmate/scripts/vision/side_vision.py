import cv2
import numpy as np
import os
import copy
from skimage.metrics import structural_similarity
from return_codes import *
from stockfish import Stockfish
from camera import Camera






class SideVision():
    def __init__(self,camera_object):
        self.camera = camera_object


    def get_movement(self,last_state_fen_string):
        ## Just give predicted movement.
        predicted = True
        if predicted:
            return SIDE_VISION_SUCCESS,"a8a5"
        else:
            return SIDE_VISION_UNSUCCESS,""

    def update_prev_image(self):
        pass

if __name__ == '__main__':
    camera = Camera()
    side_vision = SideVision(camera)
    print(side_vision.get_movement())
