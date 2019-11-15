import time
import numpy as np
import cv2

def processimage(img):

    h, w = img.shape[:2]
    #cv2.imwrite('./captures/'+time.strftime("%H%M%S")+"_140.png", image)
    blur = cv2.blur(img,(6,6))
    _,thresh1 = cv2.threshold(blur,140,255,cv2.THRESH_BINARY)
    hsv = cv2.cvtColor(thresh1, cv2.COLOR_RGB2HSV)

    hsv = hsv[:int(0.9*h), :] #Crop img[y:y+h, x:x+w]

    #cv2.imshow('hsv',hsv)

    # Define range of white color in HSV
    lower_white = np.array([0, 0, 168])
    upper_white = np.array([172, 111, 255])
    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Bigger white area
    kernel_erode = np.ones((6,6), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((4,4), np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    gray = np.float32(dilated_mask)

    #cv2.imshow('gray.png', gray)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1] #biggest contour
    result={}
    for cnt in contours:
        M = cv2.moments(contours[0])
        # Centroid
        if M['m00']==0:
            cx,cy=320,0
        else:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        #print("Centroid of the biggest area: ({}, {})".format(cx, cy))
        result["centroid"]=(cx,cy)

        beta = 0.01 # configure number poly
        approx = cv2.approxPolyDP(cnt, beta*cv2.arcLength(cnt, True), True)

        #cv2.imwrite('./corner/'+time.strftime("%H%M%S")+"_Tcorner.png", image)
        #print("number poly approx",len(approx))

        if len(approx) >= 6:
            cv2.drawContours(img, [approx], 0, (0,0,255), 5)
            result["intersection"] = True
        else:
            cv2.drawContours(img, [approx], 0, (0,255,0), 5)
            result["intersection"] = False

        cv2.imshow("aaa",img)
        cv2.waitKey(1)
    return result
