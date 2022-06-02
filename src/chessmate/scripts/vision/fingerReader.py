from itertools import count
import cv2
import os, time
import HandTrackingModule as htm

class fingerReader() :
    def __init__(self):
        path = "FingerImages"                               # Folder path name str
        myLst = os.listdir(path)                            # Returns array of images listed inside dir

        overlayImgs = []
        for ImgPath in myLst:
            fingerImg = cv2.imread(f"{path}/{ImgPath}")     # Read single finger (.jpg) image inside dir
            overlayImgs.append(fingerImg)                   # Store read finger image
        path = "FingerImages"                               # Folder path name str
        myLst = os.listdir(path)                            # Returns array of images listed inside dir

        overlayImgs = []
        for ImgPath in myLst:
            fingerImg = cv2.imread(f"{path}/{ImgPath}")     # Read single finger (.jpg) image inside dir
            overlayImgs.append(fingerImg)                   # Store read finger image


        self.tipsIDs = [8, 12, 16, 20]   # Array to store each finger tip ID

        self.detector = htm.handDetector(detectionCon=0.75)      # Creates a basic Hand detector

        self.prev_finger = -1
        self.count = -30

    def readFinger(self , img):


       
        outputMatrix = [] 

        # FINGER COUNTER MAIN PROCESS #
        img = self.detector.findHands(img)                           # Detects the Hands
        Landmarks = self.detector.findPosition(img, draw=False)      # Trace landmarks or position

        # If Landmarks list isn't empty
        if len(Landmarks) != 0:
            for i in range(len(self.tipsIDs)):

                # -> Say 100 is 6 and 0 is 8 -> So if 0 is below 100 then 'Finger is closed' -> ELSE -> if 0 is above 100 'Finger is opened'
                # IF PARTICULAR FINGER IS OPENED
                if Landmarks[self.tipsIDs[i]][2] < Landmarks[self.tipsIDs[i] - 2][2]:
                    outputMatrix.append(1)

                # IF PARTICULAR FINGER IS CLOSED    
                if Landmarks[self.tipsIDs[i]][2] > Landmarks[self.tipsIDs[i] - 2][2]:
                    outputMatrix.append(0)

            # print(outputMatrix)
            finger = outputMatrix.count(1)
            
            if(self.prev_finger == finger):
                self.count += 1
            elif(finger == None) :
                self.count -=1
            elif(finger != self.prev_finger):
                if(self.count == -30):
                    self.prev_finger = finger
                else:
                    self.count -= -1
        
            if(self.count == 20):
                self.finger = -1
                self.count = -30
                return finger
            else:
                return -1
        self.count -=1
        if self.count < -30:
            self.finger = -1
            self.count = -30
        return -1




if __name__ == '__main__':
    
    a = fingerReader()


    # define a video capture object
    vid = cv2.VideoCapture(0)
    end = 0
    while(True):
        
        # Capture the video frame
        # by frame
        ret, frame = vid.read()

        t = a.readFinger(frame)
        if(t != -1):
            print(t)
            if(end == 4):
                break

    
        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

