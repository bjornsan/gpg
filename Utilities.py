import cv2
import numpy as np

####
# Thresholding method explained:
#
# convert original image into a grayscale with the openCV method
# cvtColor wich takes the arguments of an image and the colorspace
# to wich it should convert the image to.
#
# The original image has 3 channels, Red, Green, Blue. When converted to grayscale
# there is only one channel. The range of this channel is 0-255.
#
# grayMin finds the lowest value in the image matrix,
# grayMax finds the highest value in the image matrix.
# This is done with the numpy methods 'amin' and 'amax'.
#
# ThreshFloat is then the mean value found in the image.
#
# Since we need an integer number between 0-255 we need to cast
# the float value of thresFloat to intger which we save in the
# variable thresh.
#
# The grayscale image is then converted to binary through the opencv
# method threshold wich take 4 arguments.
#
# 1. the image to convert
# 2. the threshold set for converting
# 3. the maximum value
# 4. the colorspace to convert to
#
# Finally the binary image is returned.
#
####
def thresholding(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	grayMin = np.amin(gray)
	grayMax = np.amax(gray)
	threshFloat = ( float(0.5)) * ( float(grayMax) + float(grayMin) )
	thresh = ( int (threshFloat) )
	binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
	
	return binary

####
#
# The warpImg method explained:
#
# Parameters: source image, points to warp, image width, image height, and a possibility to invert the image.
#
# pts1 is the original corners of the source image
# pts2 is where we want the warped corners to be, in other words
# the borders of the warped image.
#
# We save the image matrix in the variable matrix
# Then we warp the image into birdview with the opencv
# method warpPerspective
#
# Finally the warped image is returned
#
####
def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

####
#
# This method is doing exactly what it say, nothing.
# It is only used buy the trackbarmethods,
# when a trackbar is moved it is designed as an eventlistener
# and triggers a method which we define. But since we do
# not need to do anything in respons in our usage we just define
# this empty method.
#
####
def nothing(a):
    pass

####
#
# This method is only used to initialize values of trackbars, this ones are used in
# the warping method to tune the warping.
#
####
def initializeTrackbars(intialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0], wT // 2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2], wT // 2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)

####
#
# This method returns the points we need to warp the image in
# the warping method.
#
####
def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
                         (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
    return points

####
#
# This method draws circles in the points found in the
# valTrackbars method.
#
####
def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img

####
#
# parameters:
# 1 source image
# 2 minimum percentage of what is considered as the lane
# 3 display image on screen or not
# 4 what region should be evaluated
#
# region 1 is the full image
# any other region is a cutoff the image
#
# This is all about pixel summation
#
# maxValue finds the biggest sum
# minValue finds the smallest sum
#
# indexArray is an array that contains the indexes from
# our image that we consider to be the lane.
#
# Return the middlepoint of the image
#
####
def getHistogram(img, minPer=0.1, display=False, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0] // region:, :], axis=0)

    # print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    # print(basePoint)

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255 // region), (255, 0, 255), 1)
            cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist

    return basePoint

####
#
# cte = crosstrack error || in other words proportional error
# previous cte = previous crosstrack error
#
# diff_cte (differential error) = current cte - previous cte
#
# Kp, Kd are constants chosen by the user.
#
# correction is the total error calculated by the PD algorithm
#
####
def PD_control(Kp, Kd, cte, previous_cte):
    diff_cte = cte-previous_cte
    prop_term = -Kp * cte
    diff_term = -Kd * diff_cte
    correction = prop_term + diff_term
    return correction
