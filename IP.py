import cv2
import numpy as np

# Read image as grayscale (Hough needs a single channel image)
j = cv2.imread('WithCoins.jpg', 0)
p = cv2.imread('WithCoins.jpg')

# Apply median blur to the grayscale image
j = cv2.medianBlur(j, 5)

# Convert grayscale image back to BGR for visualization
centrej = cv2.cvtColor(j, cv2.COLOR_GRAY2BGR)

# Find circles using the Hough Transform
circles = cv2.HoughCircles(j, cv2.HOUGH_GRADIENT, dp=1, minDist=5, param1=100, param2=15, minRadius=7, maxRadius=10)

if circles is not None:
    circles = np.uint16(np.around(circles))
    circlesW = []
    circlesB = []
    redEnds = []

    # Show circles in green and centers in red
    # Also classify coins based on the colors at their centers (values by impixel(i) calibration)
    for i in circles[0, :]:
        cv2.circle(centrej, (i[0], i[1]), i[2], (0, 255, 0), 2)  # Circle outline
        cv2.circle(centrej, (i[0], i[1]), 2, (0, 0, 255), 3)  # Circle center
        if 170 < p[i[1], i[0], 2] < 250 and 50 < p[i[1], i[0], 1] < 110 and 30 < p[i[1], i[0], 0] < 80:
            redEnds.append([i[0], i[1]])
        elif 120 < j[i[1], i[0]] < 160:
            circlesW.append([i[1], i[0]])
        elif 45 < j[i[1], i[0]] < 90:
            circlesB.append([i[1], i[0]])

    cv2.imshow('Detected circles and centers', centrej)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("White Circles:", circlesW)
    print("Black Circles:", circlesB)

    # Show the coin classification (red for white, blue for black and unclassified remain green)
    for i in circlesW:
        cv2.circle(centrej, (i[1], i[0]), 2, (0, 0, 255), 3)  # White coin
    for i in circlesB:
        cv2.circle(centrej, (i[1], i[0]), 2, (255, 0, 0), 3)  # Black coin

    cv2.imshow('Classified circles', centrej)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

else:
    print("No circles detected")
