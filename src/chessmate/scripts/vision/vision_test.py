import cv2
import pyrealsense2 as real_sense
import numpy as np
import copy
from skimage.metrics import structural_similarity
import os

# get current directory
current_dir = os.path.dirname(os.path.realpath(__file__))

EMPTY_IMAGE_PATH = current_dir + "/empty_square_photos/"


class Difference():

    def __init__(self):
        self.pipeline = real_sense.pipeline()
        self.config = real_sense.config()

        pipeline_wrapper = real_sense.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        self.config.enable_stream(real_sense.stream.color, 640, 480, real_sense.format.bgr8, 30)

        profile = self.pipeline.start(self.config)

        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        self.align_stream = real_sense.align(real_sense.stream.color)
        self.square_information = np.ones((8, 8), dtype=str)
        self.empty_images_array = np.ones((8, 8), dtype=list)
        self.read_empty_images()



    def generate_empty_squares(self,board_image, square_width, square_height, x_pixel, y_pixel):
        print("Generated.")
        offset = 8
        for i in range(8):
            for j in range(8):
                empty_image = board_image[y_pixel + int(square_height) * i + offset: y_pixel + int(square_height) * (i + 1) - offset,
                                  x_pixel + int(square_width) * j + offset: x_pixel + int(square_width) * (j + 1) - offset]
                #cv2.imwrite(EMPTY_IMAGE_PATH + str(i) + str(j) + ".png",empty_image)
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

        height_offset = 20
        width_offset = 20
        height = height - height_offset * 2
        width = width - width_offset * 2
        square_width = width / 8
        square_height = height / 8
        l_x = l_x + width_offset
        u_y = u_y + height_offset

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
        offset = 8 ## To increase sensitivity, reduce this. Ideally, value should be same with generating.
        for i in range(8):
            for j in range(8):
                square_image = board_image[y_pixel + int(square_height) * i + offset: y_pixel + int(square_height) * (i + 1) - offset,
                                   x_pixel + int(square_width) * j + offset: x_pixel + int(square_width) * (j + 1) - offset]

                resized_square_image = cv2.resize(square_image,(self.empty_images_array[i][j].shape[1],self.empty_images_array[i][j].shape[0]),interpolation=cv2.INTER_AREA)
                difference_score = self.get_similarity_score(resized_square_image,self.empty_images_array[i][j])

                if difference_score > 0.80:
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
                    print(difference_score)



    def start(self):
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align_stream.process(frames)


                color_frame = aligned_frames.get_color_frame()

                if not color_frame:
                    continue


                color_image = np.asanyarray(color_frame.get_data())
                color_image = color_image[:, :500] # This could be change, we should crop gripper in the image.

                square_width, square_height, x_pixel, y_pixel = self.find_corners(color_image)
                #self.get_empty_full_information(color_image, square_width, square_height, x_pixel, y_pixel)
                self.generate_empty_squares(color_image, square_width, square_height, x_pixel, y_pixel)
                
                key = cv2.waitKey(1)
                if key & 0xFF == ord('g'):
                    self.generate_empty_squares(color_image,square_width,square_height,x_pixel,y_pixel)


                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
                


                cv2.imshow('Image', color_image)


        finally:
            self.pipeline.stop()




if __name__ == "__main__":
    detect_differences = Difference()
    detect_differences.start()
