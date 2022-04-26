import cv2
import pyrealsense2 as real_sense
import numpy as np
import os
import copy
from skimage.metrics import structural_similarity
from return_codes import *
from stockfish import Stockfish


#### STOCKFISH executable path ####
STOCKFISH_PATH = "/home/burak/Downloads/stockfish_14.1_linux_x64/stockfish_14.1_linux_x64"


# Get current directory
current_dir = os.path.dirname(os.path.realpath(__file__))
EMPTY_IMAGE_PATH = current_dir + "/monopol-square-photos/"

# Chessboard square information.
SQUARE_WIDTH = 0.041


#### Corner finding parameters. #####
QUALITY_LEVEL = 0.15  ## Increase this to prevent noisy corners.
MIN_DISTANCE = 10
BLOCK_SIZE = 3
GRADIENT_SIZE = 3
USE_HARRIS_DETECTOR = False
K = 0.04
#### Corner finding parameters. #####


class Vision():
    def __init__(self, top_left_x_coordinate, top_left_y_coordinate):
        self.top_left_x_coordinate = top_left_x_coordinate
        self.top_left_y_coordinate = top_left_y_coordinate
        self.square_information = np.ones((8, 8), dtype=str)
        self.empty_images_array = np.ones((8, 8), dtype=list)
        self.read_empty_images()
        self.get_square_coordinates()
        self.initialize_realsense()



    def get_square_as_string(self,i, j):
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



    def decide_movement_with_comparing_states(self,last_state, current_state):
        first_square = str()
        second_square = str()
        number_of_differences = 0
        for i in range(8):
            for j in range(8):
                if last_state[i][j] == current_state[i][j]:
                    continue
                else:
                    number_of_differences += 1
                    if last_state[i][j] == 'F':
                        first_square = self.get_square_as_string(i, j)
                    else:
                        second_square = self.get_square_as_string(i, j)

        return number_of_differences,first_square + second_square



    def get_last_state(self,fen_string):
        cp_fen_string = copy.deepcopy(fen_string)
        last_state = np.ones((8, 8), dtype=str)
        for i in range(8):
            for j in range(8):
                last_state[i][j] = 'E'

        cp_fen_string = cp_fen_string[:len(cp_fen_string) - 10]
        fen_list = cp_fen_string.split("/")
        for i in range(8):
            current_index = 0

            for j in range(len(fen_list[i])):
                try:
                    char = int(fen_list[i][j])
                    current_index += char
                except ValueError:
                    last_state[i][current_index] = 'F'
                    current_index += 1

        return last_state



    def find_corners(self,image):

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray_image, 300, QUALITY_LEVEL, MIN_DISTANCE, None, blockSize=BLOCK_SIZE,
                                          gradientSize=GRADIENT_SIZE, useHarrisDetector=USE_HARRIS_DETECTOR, k=K)

        right_most = 0
        r_x, r_y = 0, 0
        left_most = float('inf')
        l_x, l_y = 0, 0
        upper_most = float('inf')
        u_x, u_y = 0, 0

        for i in range(corners.shape[0]):
            if corners[i, 0, 0] > right_most:
                right_most = corners[i, 0, 0]
                r_x, r_y = corners[i, 0, 0], corners[i, 0, 1]

            if corners[i, 0, 0] < left_most:
                left_most = corners[i, 0, 0]
                l_x, l_y = corners[i, 0, 0], corners[i, 0, 1]

            if corners[i, 0, 1] < upper_most:
                upper_most = corners[i, 0, 1]
                u_x, u_y = corners[i, 0, 0], corners[i, 0, 1]

        candidate_lefts = list()
        for i in range(corners.shape[0]):
            if abs(corners[i, 0, 0] - l_x) < 8:
                candidate_lefts.append(tuple((corners[i, 0, 0], corners[i, 0, 1])))

        candidate_rights = list()
        for i in range(corners.shape[0]):
            if abs(corners[i, 0, 0] - r_x) < 8:
                candidate_rights.append(tuple((corners[i, 0, 0], corners[i, 0, 1])))


        candidate_lefts = sorted(candidate_lefts, key=lambda x: x[1])
        candidate_lefts.reverse()
        l_x, l_y = candidate_lefts[0][0], candidate_lefts[0][1]

        candidate_rights = sorted(candidate_rights, key=lambda x: x[1])
        candidate_rights.reverse()
        r_x, r_y = candidate_rights[0][0], candidate_rights[0][1]

        width = r_x - l_x
        height = l_y - u_y



        l_x = int(l_x)
        u_y = int(u_y)
        width = int(width)
        height = int(height)



        upper_height_offset = 12    # 12,10,13,12 monopol, 20, 12,20,20 kağıt. 25,22,16,15 star-oyun
        below_height_offset = 10    # 25,20,23,25 prestij
        left_width_offset = 13     # 25,20,25,20 sister - set.
        right_width_offset = 12

        height = height - (upper_height_offset + below_height_offset)
        width = width - (left_width_offset + right_width_offset)
        square_width = width / 8
        square_height = height / 8
        l_x = l_x + left_width_offset
        u_y = u_y + upper_height_offset

        return int(square_width), int(square_height), int(l_x), int(u_y)



    def get_similarity_score(self,first_image, second_image):
        first_image_gray = cv2.cvtColor(first_image, cv2.COLOR_BGR2GRAY)
        second_image_gray = cv2.cvtColor(second_image, cv2.COLOR_BGR2GRAY)

        (score, diff) = structural_similarity(first_image_gray, second_image_gray, full=True)

        return score



    def initialize_realsense(self):
        self.pipeline = real_sense.pipeline()
        self.config = real_sense.config()

        pipeline_wrapper = real_sense.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        self.config.enable_stream(real_sense.stream.color, 640, 480, real_sense.format.bgr8, 30)

        profile = self.pipeline.start(self.config)

        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align_stream = real_sense.align(real_sense.stream.color)



    def read_empty_images(self):
        for i in range(8):
            for j in range(8):
                empty_square_image = cv2.imread(EMPTY_IMAGE_PATH + str(i) + str(j) + ".png")
                self.empty_images_array[i][j] = empty_square_image



    def get_empty_full_information(self, board_image, square_width, square_height, x_pixel, y_pixel):
        offset = 7
        for i in range(8):
            for j in range(8):
                square_image = board_image[y_pixel + int(square_height) * i + offset: y_pixel + int(square_height) * (i + 1) - offset,
                                   x_pixel + int(square_width) * j + offset: x_pixel + int(square_width) * (j + 1) - offset]

                if square_image.shape[0] == 0 or square_image.shape[1] == 0:
                    continue

                resized_square_image = cv2.resize(square_image,(self.empty_images_array[i][j].shape[1],self.empty_images_array[i][j].shape[0]),interpolation=cv2.INTER_AREA)
                difference_score = self.get_similarity_score(resized_square_image,self.empty_images_array[i][j])


                if difference_score > 0.90 :
                    cv2.putText(img=square_image,
                                text="Empty",
                                org=(0,0),
                                fontFace=cv2.FONT_ITALIC,
                                fontScale=1,
                                color=(0, 255, 0),
                                thickness=1
                                )
                    self.square_information[i][j] = 'E'
                else:
                    cv2.putText(img=square_image,
                                text="Full",
                                org=(0,0),
                                fontFace=cv2.FONT_ITALIC,
                                fontScale=1,
                                color=(0, 0,255),
                                thickness=1
                                )
                    self.square_information[i][j] = 'F'



    def get_square_coordinates(self):
        # coordinates["a8"] = [x, y]
        # coordinates["f4"] = [x, y]
        coordinates = dict()
        for i in range(8):
            x_coordinate = self.top_left_x_coordinate - SQUARE_WIDTH * i
            y_coordinate = self.top_left_y_coordinate
            for j in range(8):
                coordinate_list = [x_coordinate,y_coordinate]
                coordinates[self.get_square_as_string(i,j)] = coordinate_list
                y_coordinate -= SQUARE_WIDTH

        self.coordinates = coordinates



    def get_piece_coordinates(self, from_square, to_square):
        from_square_coordinates = self.coordinates[from_square]
        to_square_coordinates = self.coordinates[to_square]

        return from_square_coordinates, to_square_coordinates



    def get_possible_movements(self,last_state_fen_string=str,base_square=str):
        stockfish_engine = Stockfish(path=STOCKFISH_PATH)
        possible_movements = list()
        valid_movements = list()
        last_state_as_array = self.get_last_state(last_state_fen_string)
        for i in range(8):
            for j in range(8):
                if last_state_as_array[i][j] == 'F':
                    possible_movements.append(base_square + self.get_square_as_string(i,j))

        last_state_fen_string = last_state_fen_string.replace("w","b")
        stockfish_engine.set_fen_position(last_state_fen_string)

        for movement in possible_movements:
            if stockfish_engine.is_move_correct(movement):
                valid_movements.append(movement)


        return valid_movements



    def get_movement(self,last_state_fen_string):
        try:
            counter = 0
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align_stream.process(frames)

                color_frame = aligned_frames.get_color_frame()

                if not color_frame:
                    continue

                if counter < 5:
                    counter += 1
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                square_width, square_height, x_pixel, y_pixel = self.find_corners(color_image)


                ## Chessboard not properly detected.
                if square_width < 20 or square_height < 20 or abs(square_width - square_height) > 5 :
                    return CHESSBOARD_NOT_DETECTED,""



                self.get_empty_full_information(color_image, square_width, square_height, x_pixel, y_pixel)
                last_state = self.get_last_state(last_state_fen_string)


                ## No movement detected.
                if (last_state == self.square_information).all() :
                    return NO_DIFFERENCE_DETECTED, ""


                ## Movement detected.
                number_of_differences, movement = self.decide_movement_with_comparing_states(last_state,self.square_information)


                ## In proper case, difference should not be greater than 2.
                ## Piece is moved from empty square to full square. Difference is 2 in this case.
                ## Piece ate the other piece. Difference is 1 in this case.
                ## More than 2 difference means we could not understand game state correctly.

                if number_of_differences == 2:
                    return TWO_DIFFERENCE_DETECTED, movement


                ## In this case, piece ate the other piece, so difference is one. .
                ## Movement string is like "b8" now instead of "b8c5".
                ## I will look at the number of moves that can be made from that position.
                ## If number is one, I return that move, otherwise I return one difference detected.
                elif number_of_differences == 1:
                    valid_movements = self.get_possible_movements(last_state_fen_string,movement)
                    if len(valid_movements) == 1:
                        return TWO_DIFFERENCE_DETECTED, valid_movements[0]

                    else:
                        return ONE_DIFFERENCE_DETECTED, ""

                else:
                    return MORE_THAN_TWO_DIFFERENCE_DETECTED, ""


        except Exception as e:
            print(e)
            return EXCEPTION_IN_THE_VISION_LOOP, ""




if __name__ == '__main__':
    vision = Vision(0.5410416700640096,0.320580303627435)
    print(vision.get_movement('8/K7/8/8/1kq4B/1r6/8/8 w - - 0 1'))
    #print(vision.get_movement('8/5Pp1/p7/6K1/8/3B1Pkp/1r3R2/8 w - - 0 1'))

