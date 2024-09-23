import cv2
import numpy as np

def runPipeline(image, llrobot):
    # image to hsv
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # color ranges
    redMin1 = (0, 165, 127)
    redMax1 = (10, 255, 255)
    redMin2 = (170, 165, 127)
    redMax2 = (180, 255, 255)

    blueMin = (95, 255, 70)
    blueMax = (130, 255, 250)

    yellowMin = (10, 80, 90)
    yellowMax = (30, 255, 255)

    # threshold for yellow, blue, and red
    img_threshold_blue = cv2.inRange(img_hsv, blueMin, blueMax)
    img_threshold_yellow = cv2.inRange(img_hsv, yellowMin, yellowMax)

    # combine both red ranges to one threshold
    img_threshold_red1 = cv2.inRange(img_hsv, redMin1, redMax1)
    img_threshold_red2 = cv2.inRange(img_hsv, redMin2, redMax2)
    img_threshold_red = cv2.bitwise_or(img_threshold_red1, img_threshold_red2)

    kernel_dilate = np.ones((15, 15), np.uint8)
    dilated_blue = cv2.dilate(img_threshold_blue, kernel_dilate, iterations=1)
    dilated_yellow = cv2.dilate(img_threshold_yellow, kernel_dilate, iterations=1)
    dilated_red = cv2.dilate(img_threshold_red, kernel_dilate, iterations=1)

    kernel_erode = np.ones((8, 8), np.uint8)
    eroded_blue = cv2.erode(dilated_blue, kernel_erode, iterations=3)
    eroded_yellow = cv2.erode(dilated_yellow, kernel_erode, iterations=3)
    eroded_red = cv2.erode(dilated_red, kernel_erode, iterations=3)

    # apply Canny edge detection to extract the lines of the rectangles
    edges_blue = cv2.Canny(eroded_blue, 50, 150)
    edges_yellow = cv2.Canny(eroded_yellow, 50, 150)
    edges_red = cv2.Canny(eroded_red, 50, 150)

    # find contours from the detected edges
    contours_blue, _ = cv2.findContours(edges_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv2.findContours(edges_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # process contours for all colors
    for contours in [contours_blue, contours_yellow, contours_red]:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 400:
                # approximate contour to reduce points
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)

                if len(approx) == 4:
                    # draw the lines of rectangle
                    for i in range(4):
                        pt1 = tuple(approx[i][0])
                        pt2 = tuple(approx[(i+1) % 4][0])  # connect to next line
                        cv2.line(image, pt1, pt2, (0, 255, 0), 3)  # draw  line

    output = image  # return rectangle

    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    return largestContour, output, llpython



