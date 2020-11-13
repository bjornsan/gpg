import cv2
import numpy as np
import Utilities

curveList = []
avgVal = 10


def getLaneCurve(img, display=2):
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
    middlePoint, imgHist = Utilities.getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = Utilities.getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    #### STEP 5
    if display == 1:
        cv2.imshow('Resutlt', imgResult)

    #### NORMALIZATION
    curve = curve / 100
    if curve > 1: curve == 1
    if curve < -1: curve == -1

    return curve


