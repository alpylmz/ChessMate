import cv2
import pyrealsense2 as real_sense
import numpy as np
import copy
import os
from skimage.metrics import structural_similarity
from datetime import datetime
from camera import Camera


# get current directory
current_dir = os.path.dirname(os.path.realpath(__file__))

EMPTY_IMAGE_PATH = current_dir + "/monopol-square-photos/"
TEST_PATH = current_dir + "/monopol-test-photos/"


class Difference():

    def __init__(self,camera_object):
        self.camera = camera_object
        self.square_information = np.ones((8, 8), dtype=str)
        self.empty_images_array = np.ones((8, 8), dtype=list)
        self.read_empty_images()
        self.square_width = 45
        self.square_height = 45
        self.x_pixel = 173
        self.y_pixel = 45



    def generate_empty_squares(self,board_image, square_width, square_height, x_pixel, y_pixel,is_generate):
        offset = 7
        for i in range(8):
            for j in range(8):
                empty_image = board_image[y_pixel + int(square_height) * i + offset: y_pixel + int(square_height) * (i + 1) - offset,
                                  x_pixel + int(square_width) * j + offset: x_pixel + int(square_width) * (j + 1) - offset]

                if is_generate:
                    print("Generated.")
                    cv2.imwrite(EMPTY_IMAGE_PATH + str(i) + str(j) + ".png",empty_image)

                cv2.rectangle(board_image,
                              (x_pixel + int(square_width) * j + offset, y_pixel + int(square_height) * i + offset),
                              (x_pixel + int(square_width) * (j + 1) - offset, y_pixel + int(square_height) * (i + 1) - offset),
                              (0, 255, 0),
                              1)



    def find_corners(self,image):
        qualityLevel = 0.15  # 0.05 for paper.
        minDistance = 10
        blockSize = 3
        gradientSize = 3
        useHarrisDetector = False
        k = 0.04

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray_image, 300, qualityLevel, minDistance, None, blockSize=blockSize,
                                          gradientSize=gradientSize, useHarrisDetector=useHarrisDetector, k=k)
        radius = 4

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

        for candidate in candidate_lefts:
            cv2.circle(image, (int(candidate[0]), int(candidate[1])), radius, (256, 0, 0), cv2.FILLED)

        candidate_rights = list()
        for i in range(corners.shape[0]):
            if abs(corners[i, 0, 0] - r_x) < 8:
                candidate_rights.append(tuple((corners[i, 0, 0], corners[i, 0, 1])))

        for candidate in candidate_rights:
            cv2.circle(image, (int(candidate[0]), int(candidate[1])), radius, (256, 0, 0), cv2.FILLED)

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
        cv2.rectangle(image, (l_x, u_y), (l_x + width, u_y + height), (255, 255, 255), 1)


        upper_height_offset = 12   # 12,10,13,12 monopol, 20, 12,20,20 kağıt. 25,22,16,15 star-oyun
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



    def read_empty_images(self):
        for i in range(8):
            for j in range(8):
                empty_square_image = cv2.imread(EMPTY_IMAGE_PATH + str(i) + str(j) + ".png")
                self.empty_images_array[i][j] = empty_square_image



    def get_empty_full_information(self, board_image, square_width, square_height, x_pixel, y_pixel):
        offset = 7 ## To increase sensitivity, reduce this. Ideally, value should be same with generating.
        for i in range(8):
            for j in range(8):
                square_image = board_image[y_pixel + int(square_height) * i + offset: y_pixel + int(square_height) * (i + 1) - offset,
                                   x_pixel + int(square_width) * j + offset: x_pixel + int(square_width) * (j + 1) - offset]

                resized_square_image = cv2.resize(square_image,(self.empty_images_array[i][j].shape[1],self.empty_images_array[i][j].shape[0]),interpolation=cv2.INTER_AREA)
                difference_score = self.get_similarity_score(resized_square_image,self.empty_images_array[i][j])



                if difference_score > 0.87:
                    cv2.putText(img=square_image,
                                text="Empty",
                                org=(0,0),
                                fontFace=cv2.FONT_ITALIC,
                                fontScale=1,
                                color=(0, 255, 0),
                                thickness=1
                                )
                    self.square_information[i][j] = 'E'
                    print(difference_score)
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
                    #print(difference_score)



    def is_time_passed(self,past_time,time):
        now = datetime.now()
        if (now - past_time).total_seconds() > time:
            return True

        return False


    def start(self):
        past_time = datetime.now()
        COUNTER = 200
        try:
            while True:
                color_image, depth_frame, depth_scale = self.camera.GetImage()
                #color_image = cv2.imread(TEST_PATH + "image-" + str(COUNTER) + ".png")
                #self.square_width, self.square_height, self.x_pixel, self.y_pixel = self.find_corners(color_image)
                self.square_width, self.square_height, self.x_pixel, self.y_pixel = 47,47,169,51
                #self.get_empty_full_information(color_image, self.square_width, self.square_height, self.x_pixel, self.y_pixel)
                #self.generate_empty_squares(color_image, self.square_width, self.square_height, self.x_pixel, self.y_pixel,False)


                key = cv2.waitKey(1)
                if key & 0xFF == ord('c'):
                    [self.square_width, self.square_height, self.x_pixel, self.y_pixel] = [int(x) for x in input().split()]


                if key & 0xFF == ord('g'):
                    self.generate_empty_squares(color_image, self.square_width, self.square_height, self.x_pixel, self.y_pixel,True)


                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break


                cv2.imshow('Image', color_image)


        finally:
            self.camera.Stop()




if __name__ == "__main__":
    camera = Camera()
    detect_differences = Difference(camera)
    detect_differences.start()


