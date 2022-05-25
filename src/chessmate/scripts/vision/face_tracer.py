import cv2
import os
from camera import Camera
from return_codes import *
from deepface import DeepFace

# Get current directory
current_dir = os.path.dirname(os.path.realpath(__file__))
XML_PATH = current_dir + "/xml_files/"




class FaceTracer():
    def __init__(self,camera_object=Camera):
        self.camera = camera_object
        self.face_classifier = cv2.CascadeClassifier(XML_PATH + "face.xml")
        self.detected_face = 0


    def get_distance(self, depth_data, x_pixel, y_pixel, width, height):
        depth_region = depth_data[y_pixel: y_pixel + int(height), x_pixel: x_pixel + int(width)].astype(float)
        depth_region = depth_region * self.camera.depth_scale
        depth, _, _, _ = cv2.mean(depth_region)
        return depth


    def get_face_locations(self,frame):
        gray_frame = cv2.cvtColor(src=frame, code=cv2.COLOR_BGR2GRAY)
        humans = self.face_classifier.detectMultiScale(image=gray_frame, scaleFactor=1.1, minNeighbors=4)
        return humans


    def is_face_aligned(self):
        try:
            while True:
                color_image, depth_frame, _ = self.camera.GetImage()
                faces = self.get_face_locations(color_image)
                nearest_distance_to_camera = float('inf')
                distance_to_center_in_x = 0
                distance_to_center_in_y = 0
                x_coordinate,y_coordinate,width,height = 0,0,0,0

                is_face_detected = False

                for face in faces:
                    (c_x_coordinate, c_y_coordinate, c_width, c_height) = face
                    distance = self.get_distance(depth_frame,c_x_coordinate,c_y_coordinate,c_width,c_height)
                    if distance < nearest_distance_to_camera and distance < 1 and c_width > 80 and c_height > 80:
                        is_face_detected = True
                        x_coordinate, y_coordinate, width, height = c_x_coordinate, c_y_coordinate, c_width, c_height
                        nearest_distance_to_camera = distance
                        distance_to_center_in_x = x_coordinate + width / 2 - color_image.shape[1] / 2
                        distance_to_center_in_y = y_coordinate + height / 2 - color_image.shape[0] / 2


                if is_face_detected:
                    if distance_to_center_in_x < -30:
                        return GO_RIGHT

                    elif distance_to_center_in_x > 30:
                        return GO_LEFT

                    elif distance_to_center_in_y < -30:
                        return GO_DOWN

                    elif distance_to_center_in_y > 30:
                        return GO_UP

                    else:
                        return FACE_ALIGNED

                else :
                    return FACE_NOT_DETECTED

        finally:
            return FACE_NOT_DETECTED


    def get_emotion(self):
        try:
            while True:
                color_image, depth_frame, depth_scale = self.camera.GetImage()
                faces = self.get_face_locations(color_image)
                nearest_distance_to_camera = float('inf')
                x_coordinate, y_coordinate, width, height = 0, 0, 0, 0

                is_face_detected = False

                for face in faces:
                    (c_x_coordinate, c_y_coordinate, c_width, c_height) = face
                    distance = self.get_distance(depth_frame, c_x_coordinate, c_y_coordinate, c_width, c_height)
                    if distance < nearest_distance_to_camera and distance < 1 and c_width > 80 and c_height > 80:
                        is_face_detected = True
                        x_coordinate, y_coordinate, width, height = c_x_coordinate, c_y_coordinate, c_width, c_height
                        nearest_distance_to_camera = distance


                if is_face_detected:
                    color_image = color_image[y_coordinate : y_coordinate + height , x_coordinate : x_coordinate + width]
                    try:
                        analyze = DeepFace.analyze(color_image,actions = ('emotion', 'gender'),enforce_detection=False)
                        if analyze['dominant_emotion'] == "happy":
                            return HAPPY_FACE

                        elif analyze['dominant_emotion'] == "neutral":
                            return NEUTRAL_FACE

                        else:
                            return UNHAPPY_FACE

                    except Exception as e:
                        return NEUTRAL_FACE


                else:
                    return NEUTRAL_FACE

        finally:
            return NEUTRAL_FACE




if __name__ == "__main__":
    camera = Camera()
    face_tracer = FaceTracer(camera)
    #face_tracer.is_face_aligned()
    print(face_tracer.get_emotion())
