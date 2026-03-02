import brickpi3
import time
import cv2 
import numpy as np
from picamera2 import Picamera2
 
BP = brickpi3.BrickPi3()
 
 
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
 
picam2.start()
 
starttime = time.time()
 
white = (255,255,255)
font = cv2.FONT_HERSHEY_SIMPLEX 

for i in range(1000):
    img = picam2.capture_array()
 
    # Convert to HSV colour space    
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
 
    
    # Apply colour thresholding: for red this is done in two steps
    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red) 
    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    # join my masks
    mask = mask0+mask1
    # This is a thresholded version of the image which you can display if
    # you want to check what the colour thresholding does
    result = cv2.bitwise_and(img, img, mask=mask)
 
 
    cv2.imwrite("demo.jpg", result)
    print("drawImg:" + "/home/pi/prac-files/demo.jpg")
    print("Captured image", i, "at time", time.time() - starttime)
 
picam2.stop()
