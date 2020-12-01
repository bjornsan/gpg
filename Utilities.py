import cv2
import numpy as np


def thresholding(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	grayMin = np.amin(gray)
	grayMax = np.amax(gray)
	threshFloat = ( float(0.5)) * ( float(grayMax) + float(grayMin) )
	thresh = ( int (threshFloat) )
	binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
	return binary

def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp


def nothing(a):
    pass

def initializeTrackbars(intialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0], wT // 2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2], wT // 2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)

def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
                         (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
    return points

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
def getHistogram(img, minPer=0.1, region=1):
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

    return basePoint

def find_intersection(img, minPer=0.1, region=4):
    hist_values = np.sum(img[img.shape[0] // region:, :], axis=0)
    len_hist_values = len(hist_values)

    roi_values = []

    for i in range(len_hist_values):
        roi_values.append(hist_values[i][0])
        roi_values.append(hist_values[i][1])
        roi_values.append(hist_values[i][2])
        roi_values.append(hist_values[i][3])
        roi_values.append(hist_values[i][-1])
        roi_values.append(hist_values[i][-2])
        roi_values.append(hist_values[i][-3])
        roi_values.append(hist_values[i][-4])

    max_value = np.max(roi_values)
    min_value = minPer * max_value
    white_values = np.where(roi_values >= min_value)

    white_two = []
    for i in range(len_roi_values):
        if roi_values[]
    len_roi_values = len(roi_values)
    len_white_values = len(white_values)
    percentage_white = len_white_values/len_roi_values

    if percentage_white > 0.7:
        return True

    return False

def find_intersection_corner_detection(img):
    corners = cv2.goodFeaturesToTrack(img, 4, 0.01, 10)
    corners = np.int0(corners)
    len_corners = len(corners)
    min_x = 3000
    min_y = 3000
    max_x = 0
    max_y = 0

    for i in corners:
        x, y = i.ravel()
        if x < min_x:
            min_x = x
        if x > max_x:
            max_x = x
        if y < min_y:
            min_y = y
        if y > max_y:
            max_y = y

    dist_x = int(max_x - min_x)
    dist_y = int(max_y - min_y)

    if len_corners >= 4 and dist_x < 200 and dist_y < 200:
        return True

    return False


def find_intersection_contours():
    pass

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
    angular_velocity = prop_term + diff_term
    return angular_velocity
