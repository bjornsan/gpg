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

####
#
# Needs more work, not currently working in all situations.
#
# The idea is to check the first few and last few pixels in
# each row and store them in a new array.
#
# After that to calculate the max value of the array and the
# min value of the array. With a user given min percentage.
#
# A new array is created containing the elements considered to be
# >= the minimum value.
#
# Lastly we find out if the percentage of white pixels found are
# more than 70%. If So then we have found an interection.
#
####
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

    len_roi_values = len(roi_values)
    len_white_values = len(white_values)
    percentage_white = len_white_values/len_roi_values

    if percentage_white > 0.7:
        return True

    return False

####
#
# Using Harris corner detection to find an intersection.
#
# Storing the x and y values in an array and finding out
# x_min, x_max, y_min, y_max
#
# calculating dx and dy.
#
# If we detect minimum 4 corners that are also
# separated by no more than 200pixels we consider
# that we have found an intersection.
#
def find_intersection_corner_detection(img):
    corners = cv2.goodFeaturesToTrack(img, 4, 0.01, 10)
    corners = np.int0(corners)
    len_corners = len(corners)

    rows, cols = 4,4
    xy_coordinates = [[0]*cols]*rows
    count = 0
    for i in corners:
        x, y = i.ravel()
        xy_coordinates[0][count] = x
        xy_coordinates[1][count] = y
        count = count + 1

    x_min = min(xy_coordinates[0])
    x_max = max(xy_coordinates[0])
    y_min = min(xy_coordinates[1])
    y_max = max(xy_coordinates[1])

    dx = int(x_max-x_min)
    dy = int(y_max-y_min)

    if len_corners >= 4 and dx < 200 and dy < 200:
        return True

    return False

####
#
# Detects straight lines using Hough Transform.
#
# declaring two variables first_border_x and last_border_x
# for storing the border x-values for the right most lane.
#
# Returning the x-values between where the right most track is.
#
####
def find_borders_of_right_lane(img):
    lines = cv2.HoughLines(img, 1, np.pi / 180, 200)

    first_border_x = 0
    last_border_x = 0
    for i in range(4):
        print(i)
        for rho, theta in lines[i]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

            if i == 2:
                last_border_x = x1
            if i == 3:
                first_border_x = x1

    return first_border_x, last_border_x

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
