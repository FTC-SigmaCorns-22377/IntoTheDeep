import cv2
import numpy as np
#Called every frame
def runPipeline(image, llrobot):
    #Convert image to hsv(hs1) = Hue, Saturation, Lightness/Value
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    redMin = (-20,165,127)
    redMax = (10,255,255)

    blueMin = (95, 255, 70)
    blueMax = (130,255,250)

    yellowMin = (10,80,90)
    yellowMax = (30,255,255)

    #Threshold for yellow and blue in hsv that opencv will look at
    img_threshold_blue = cv2.inRange(img_hsv,blueMin,blueMax)
    img_threshold_yellow = cv2.inRange(img_hsv,yellowMin,yellowMax)
    img_threshold_red = cv2.inRange(img_hsv,redMin,redMax)

    kernel_dialate = np.ones((15, 15), np.uint8)
    dilatation_blue = cv2.dilate(img_threshold_blue, kernel_dialate, iterations=1)
    dilatation_yellow = cv2.dilate(img_threshold_yellow, kernel_dialate, iterations=1)
    dilatation_red = cv2.dilate(img_threshold_red, kernel_dialate, iterations=1)
    kernel_erode = np.ones((8, 8), np.uint8)
    erosion_blue = cv2.erode(dilatation_blue, kernel_erode, iterations=3)
    erosion_yellow = cv2.erode(dilatation_yellow, kernel_erode, iterations=3)
    erosion_red = cv2.erode(dilatation_red, kernel_erode, iterations = 3)
    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    output = None
    step = 0

    contours_blue, _ = cv2.findContours(erosion_blue,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours_yellow, _ = cv2.findContours(erosion_yellow,
                                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(erosion_red,
                                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the contours, this is the black dots/blobs
    #cv2.drawContours(image, contours, -1, 0, 1)
    for contour in contours_blue:
        area = cv2.contourArea(contour)

        # Checks the area of the box it found and if it is bigger than some value
        # it is drawn
        if 400 < area:
            # Draw contour
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 255, 0), 10)
    for contour in contours_yellow:
        area = cv2.contourArea(contour)

        # Checks the area of the box it found and if it is bigger than some value
        # it is drawn
        if 400 < area:
            # Draw contour
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 255, 0), 10)

    for contour in contours_red:
        area = cv2.contourArea(contour)

        # Checks the area of the box it found and if it is bigger than some value
        # it is drawn
        if 400 < area:
            # Draw contour
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 255, 0), 10)

    if step==0:
        output = image
    if step==1:
        output = cv2.cvtColor(img_threshold_blue, cv2.COLOR_GRAY2RGB)
    if step==2:
        output = cv2.cvtColor(dilatation_dst, cv2.COLOR_GRAY2RGB)
    if step==3:
        output = cv2.cvtColor(img_erosion, cv2.COLOR_GRAY2RGB)

    rout = cv2.cvtColor(img_threshold_red, cv2.COLOR_GRAY2RGB)
    return largestContour, output, llpython