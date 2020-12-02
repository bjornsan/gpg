import cv2
import numpy as np
import Utilities

curveList = []
avgVal = 10


def getLaneCurve(img, display =1):
    imgCopy = img.copy()
    imgResult = img.copy()
    #### STEP 1
    imgThres = Utilities.thresholding(img)

    #### STEP 2
    hT, wT, c = img.shape
    points = Utilities.valTrackbars()
    imgWarp = Utilities.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = Utilities.drawPoints(imgCopy, points)

    #### STEP 3

    ####
    # TODO
    # Add logic to limit the processing to the right most lane.
    #
    # multiple_lanes = Utilities.detect_multiple_lanesU(img)
    #
    # if multiple_lanes:
    #     first_border_x, last_border_x = Utilities.find_borders_of_right_lane(imgWarp)
    #     make histogram limit its pixelsummation to the interval between first_border_x
    #     and last_border_x
    #
    #     find middlePoint and curveAveragePoint like hits.
    # else:
    #     run the code as usual
    #     middlePoint, imgHist = Utilities.getHistogram(imgWarp, minPer=0.5, region=4)
    #     curveAveragePoint, imgHist = Utilities.getHistogram(imgWarp, minPer=0.9)
    #
    #
    # The rest of the code should run like normal.
    #
    ####

    middlePoint, imgHist = Utilities.getHistogram(imgWarp, minPer=0.5, region=4)
    curveAveragePoint, imgHist = Utilities.getHistogram(imgWarp, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    #### Find interdection
    imgIntersection = imgWarp.copy()
    intersection = Utilities.find_intersection(imgIntersection, minPer=0.5, region=4)

    #### Find intersection with corner detetection
    imgCornerDetection = imgWarp.copy()
    intersection = Utilities.find_intersection_corner_detection(imgCornerDetection)

    if intersection:
        return 0, intersection

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    #### STEP 5
    if display == 1:
        cv2.imshow('Resutlt', imgThres)

    #### NORMALIZATION
    curve = curve / 100
    if curve > 1: curve == 1
    if curve < -1: curve == -1

    return curve, intersection


