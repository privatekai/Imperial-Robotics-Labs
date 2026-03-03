import brickpi3
import time
import cv2 
import numpy as np
from picamera2 import Picamera2
 
BP = brickpi3.BrickPi3()
WHITE = (255,255,255)
GREEN = (0,255,0)
FONT = cv2.FONT_HERSHEY_SIMPLEX 
 
def displayImg(img):
    cv2.imwrite("demo.jpg", img)
    print("drawImg:" + "/home/pi/prac-files/demo.jpg")

def captureCanCentroids(picam, starttime=0.0):
    img = picam.capture_array()
 
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
    # result = cv2.bitwise_and(img, img, mask=mask)

    # Calculate connected components: colour thresholded "blob" regions 
    output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32F)
    (numLabels, _, stats, _) = output

    centroids = []
    # centroid: (x, y, w, h, area, lowest_point)

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

            lowest_point = (x + 0.5*w, y + h) # y value increases as going down

            if (area > 200):
                centroids.append((x, y, w, h, area, lowest_point))
            
    # Delete all centroids corresponding to the same coke tower
    # Also draw centroid coordinates
    centroids.sort(key=lambda c: -c[-1][1])
    i = 0
    while i < len(centroids):
        (x, y, w, h, area, lowest_point) = centroids[i]
        lowest_x, lowest_y = int(lowest_point[0]), int(lowest_point[1])

        # Deleting directly above centroids
        j = i+1
        while j < len(centroids):
            (_, _, _, _, _, (other_lowest_x, _)) = centroids[j]
            if other_lowest_x > x and other_lowest_x < x+w: # offset?
                centroids.pop(j)
            else:
                j += 1
    
        i += 1

    # Draw Rectangles
    for (x, y, w, h, area, lowest_point) in centroids:
        img = cv2.rectangle(img, (x, y), (x+w, y+h), GREEN, 5)

    # Draw image on web interface
    # cv2.imwrite("demo.jpg", img)
    # print("drawImg:" + "/home/pi/prac-files/demo.jpg")
    # print("Captured image", i, "at time", time.time() - starttime)

    return (img, centroids)

if __name__ == "__main__":
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(preview_config)
    picam2.start()
    starttime = time.time()

    for i in range(1000):
        (img, _) = captureCanCentroids(picam2)
        displayImg(img)
    
    picam2.stop()
