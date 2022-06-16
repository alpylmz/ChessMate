import cv2
import pyrealsense2 as real_sense
import numpy as np
import os
import copy
from return_codes import *
from camera import Camera
from vision_calibration_parameters import *



class ColorTopVision():
    def __init__(self, camera_object=Camera):
        self.camera = camera_object
        self.current_state = np.ones((8, 8), dtype=str)
        self.last_state = np.ones((8, 8), dtype=str)

    def get_square_as_string(self, i, j):
        row = 8 - i
        column = ' '
        if j == 0:
            column = "a"
        if j == 1:
            column = "b"
        if j == 2:
            column = "c"
        if j == 3:
            column = "d"
        if j == 4:
            column = "e"
        if j == 5:
            column = "f"
        if j == 6:
            column = "g"
        if j == 7:
            column = "h"

        return column + f'{row}'


    def get_color_of_square(self, image):
        black_cp1 = copy.deepcopy(image)

        lower = np.array(BLACK_LOWER)
        upper = np.array(BLACK_UPPER)
        hsv = cv2.cvtColor(black_cp1, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(black_cp1, black_cp1, mask=mask)
        result_bgr = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        black_average = np.average(result_bgr)
        if np.average(result_bgr) > 1.0:
            print(np.average(result_bgr))
            return 'B'

        return 'E'


    def is_castling_happened(self,last_state, current_state):
        number_of_differences = 0
        for i in range(8):
            for j in range(8):
                if last_state[i][j] != current_state[i][j]:
                    number_of_differences += 1

        if number_of_differences == 4:
            if last_state[0][0] != 'E' and current_state[0][0] == 'E' and \
                    last_state[0][2] == 'E' and current_state[0][2] != 'E' and \
                    last_state[0][3] == 'E' and current_state[0][3] != 'E' and \
                    last_state[0][4] != 'E' and current_state[0][4] == 'E':
                return True, "e8c8"

            if last_state[0][4] != 'E' and current_state[0][4] == 'E' and \
                    last_state[0][5] == 'E' and current_state[0][5] != 'E' and \
                    last_state[0][6] == 'E' and current_state[0][6] != 'E' and \
                    last_state[0][7] != 'E' and current_state[0][7] == 'E':
                return True, "e8g8"

        return False, ""


    def decide_movement_with_comparing_states(self, last_state, current_state):
        is_castling, movement = self.is_castling_happened(last_state,current_state)
        if is_castling:
            return 2,movement

        first_square = ""
        second_square = ""
        number_of_differences = 0

        for i in range(8):
            for j in range(8):
                if last_state[i][j] == current_state[i][j]:
                    continue
                else:
                    number_of_differences += 1
                    if last_state[i][j] == 'B':
                        first_square = self.get_square_as_string(i, j)
                    else:
                        second_square = self.get_square_as_string(i, j)

        if number_of_differences == 2:
            return 2, first_square + second_square

        return number_of_differences, ""


    def get_last_state(self, fen_string):
        cp_fen_string = copy.deepcopy(fen_string)
        last_state = np.ones((8, 8), dtype=str)
        for i in range(8):
            for j in range(8):
                last_state[i][j] = 'E'

        cp_fen_string = cp_fen_string.split(" ")[0]
        fen_list = cp_fen_string.split("/")
        for i in range(8):
            current_index = 0

            for j in range(len(fen_list[i])):
                try:
                    char = int(fen_list[i][j])
                    current_index += char
                except ValueError:
                    if fen_list[i][j].isupper():
                        last_state[i][current_index] = 'W'
                    else:
                        last_state[i][current_index] = 'B'
                    current_index += 1

        return last_state


    def get_empty_full_information(self, board_image, square_width, square_height, x_pixel, y_pixel):
        offset = 7
        for i in range(8):
            for j in range(8):
                square_image = board_image[y_pixel + int(square_height) * i + offset: y_pixel + int(square_height) * (i + 1) - offset,x_pixel + int(square_width) * j + offset: x_pixel + int(square_width) * (j + 1) - offset]

                if self.get_color_of_square(square_image) == 'B':
                    self.current_state[i][j] = 'B'

                else:
                    if self.last_state[i][j] == 'B':
                        self.current_state[i][j] = 'E'
                    else:
                        self.current_state[i][j] = self.last_state[i][j]


    def get_movement(self, last_state_fen_string):
        try:
            counter = 0
            while True:
                if counter < 5:
                    counter += 1
                    continue

                color_frame, depth_frame, depth_scale = self.camera.GetImage()
                square_width, square_height, x_pixel, y_pixel = TOP_VISION_SQUARE_WIDTH,TOP_VISION_SQUARE_HEIGHT,TOP_VISION_SQUARE_X_PIXEL,TOP_VISION_SQUARE_Y_PIXEL

                self.last_state = self.get_last_state(last_state_fen_string)
                self.get_empty_full_information(color_frame, square_width, square_height, x_pixel, y_pixel)


                ## No movement detected.
                if (self.last_state == self.current_state).all():
                    return NO_DIFFERENCE_DETECTED, ""

                ## Movement detected.
                number_of_differences, movement = self.decide_movement_with_comparing_states(self.last_state, self.current_state)

                if number_of_differences == 2:
                    return TWO_DIFFERENCE_DETECTED, movement

                else:
                    return MORE_THAN_TWO_DIFFERENCE_DETECTED, ""


        except Exception as e:
            print(e)
            return EXCEPTION_IN_THE_LOOP,""



if __name__ == '__main__':
    camera = Camera()
    top_vision = ColorTopVision(camera)
    print(top_vision.get_movement("rnbqk2r/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b - - 0 1"))
