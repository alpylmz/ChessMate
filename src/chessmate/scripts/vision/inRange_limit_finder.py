import cv2
import numpy as np
from camera import Camera

def nothing(x):
    pass

# White pieces
# hMin = 0
# sMin = 66
# vMin = 85
# hMax = sMax = vMax = 0

# Black pieces
# hMin = 0
# sMin = 0
# vMin = 0
# hMax = sMax = 0
# vMax = 39

camera = Camera()

# Create a window
cv2.namedWindow('image')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = 0
sMin = 0
vMin = 0
hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

while 1:
    #image = cv2.imread("/home/burak/Desktop/21-22-Fall-Ders-Materyalleri/491/Project-OpenCV-Studies/difference-studies/monopol-test-photos/image-228.png")
    image, depth_frame, depth_scale = camera.GetImage()
    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')
    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')

    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)

    if (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax):
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax


   # cv2.imshow('image', result)
    result_bgr = cv2.cvtColor(result,cv2.COLOR_HSV2BGR)
    #result_bgr = result_bgr[210:210+28,201:201+28]
    #print(np.average(result_bgr[183:183+28,206:206+28]))
    #print(np.average(result_bgr[295:295+28,430:430+28]))
    cv2.imshow('image-2', result_bgr)

    if cv2.waitKey(1) == ord('q'):
        print(lower, upper)
        break


    if cv2.waitKey(1) == ord('p'):
        print(lower,upper)






cv2.destroyAllWindows()



























