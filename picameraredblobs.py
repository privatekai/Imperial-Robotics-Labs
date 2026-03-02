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
 
    # Calculate connected components: colour thresholded "blob" regions 
    output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32F)
    (numLabels, labels, stats, centroids) = output
    
    # Find the properties of the detected blobs
    for i in range(0, numLabels):
        # i=0 is the background region so ignore it
        if i != 0:
            # Extract the connected component statistics and centroid
            # Here you can get the limits of the blob if you need them
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            (cu, cv) = centroids[i]
            # Print out the properties of blobs above a certain size
            if (area > 150):
                print("Component", i, "area", area, "Centroid", cu, cv)
                cuint = int(cu)
                cvint = int(cv)
                # Draw a little circle to show each detected blob
                img = cv2.circle(img, (cuint, cvint), 5, white, 3)
                # Also print its coordinates on the image!
                pstring = "(" + str(cuint) + "," + str(cvint) + ")"
                img = cv2.putText(img, pstring, (cuint + 8,cvint), font, 0.5, white, 1, cv2.LINE_AA)
 
    cv2.imwrite("demo.jpg", img)
    print("drawImg:" + "/home/pi/prac-files/demo.jpg")
    print("Captured image", i, "at time", time.time() - starttime)
 
picam2.stop()
