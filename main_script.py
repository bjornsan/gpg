% matplotlib
inline
from easygopigo3 import EasyGoPiGo3
from picamera.array import PiRGBArray
from picamera import PiCamera
from matplotlib import pyplot as plt
import time
import cv2
import numpy as np

curveList = []
avgVal = 10
previous_cte = 0.0
Kp = 5
Kd = 2.5

robot = EasyGoPiGo3()
camera = PiCamera()
camera.resolution = (480, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480, 240))
time.sleep(2)

####
#
#  getLaneCurve is in part taken from Murtaza Workshop's
#  Youtube channel, and his tutorial about a raspberry pi lane follower
#  https://www.youtube.com/watch?v=aXqoPiMPhDw&list=PLMoSUbG1Q_r_wT0Ac7rOlhlwq9VsZDA0b
#
####
def getLaneCurve(img):
    #### STEP 1
    imgThres = thresholding(img)

    #### STEP 2
    hT, wT, c = img.shape
    points = valTrackbars()
    imgWarp = warpImg(imgThres, points, wT, hT)

    #### STEP 3
    middlePoint = getHistogram(imgWarp, minPer=0.5, region=4)
    curveAveragePoint = getHistogram(imgWarp, minPer=0.9, region=1)
    curveRaw = curveAveragePoint - middlePoint

    intersection = find_intersection(imgWarp)

    if intersection:
        return 0, intersection

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    #### NORMALIZATION
    curve = curve / 100
    if curve > 1: curve = 1
    if curve < -1: curve = -1

    return curve, intersection

####
#
#  The thresholding function is taken from the lectures
#  by Maben Rabi.
#
####
def thresholding(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayMin = np.amin(gray)
    grayMax = np.amax(gray)
    threshFloat = (float(0.5)) * (float(grayMax) + float(grayMin))
    thresh = (int(threshFloat))
    binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

    return binary

####
#
#  def nothing is taken from Murtaza Workshop's
#  Youtube channel, and his tutorial about a raspberry pi lane follower
#  https://www.youtube.com/watch?v=aXqoPiMPhDw&list=PLMoSUbG1Q_r_wT0Ac7rOlhlwq9VsZDA0b
#
####
def nothing(a):
    pass

####
#
#  valTrackbars is taken from Murtaza Workshop's
#  Youtube channel, and his tutorial about a raspberry pi lane follower
#  https://www.youtube.com/watch?v=aXqoPiMPhDw&list=PLMoSUbG1Q_r_wT0Ac7rOlhlwq9VsZDA0b
#
####
def valTrackbars(wT=480, hT=240):
    points = np.float32([(102, 80), (wT - 102, 80),
                         (20, 214), (wT - 20, 214)])
    return points

####
#
#  warpImg is taken from Murtaza Workshop's
#  Youtube channel, and his tutorial about a raspberry pi lane follower
#  https://www.youtube.com/watch?v=aXqoPiMPhDw&list=PLMoSUbG1Q_r_wT0Ac7rOlhlwq9VsZDA0b
#
####
def warpImg(img, points, w, h):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

####
#
#  getHistogram is in part taken from Murtaza Workshop's
#  Youtube channel, and his tutorial about a raspberry pi lane follower
#  https://www.youtube.com/watch?v=aXqoPiMPhDw&list=PLMoSUbG1Q_r_wT0Ac7rOlhlwq9VsZDA0b
#
####
def getHistogram(img, minPer=0.1, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0] // region:, :], axis=0)

    two_lanes, right_lane_index = find_two_lanes(histValues)

    if two_lanes:
        histValues = getRightLaneHistogram(histValues, right_lane_index)

    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    return basePoint

####
#
#  find_two_lanes is written by us,
#  group 4.
#
####
def find_two_lanes(histValues):
    intensity_flag = 0
    index = 0

    for pixel in range(len(histValues)):

        if intensity_flag == 0 and histValues[pixel] > 170:
            intensity_flag = 1
        if intensity_flag == 1 and histValues[pixel] < 175:
            intensity_flag = 2
        if intensity_flag == 2 and histValues[pixel] > 175:
            index = pixel
            print("two lanes found")
            return True, index

    print("one lane only")
    return False, index

####
#
#  getRightLaneHistogram is written by us,
#  group 4.
#
####
def getRightLaneHistogram(histValues, index):
    right_lane_values = np.zeros(len(histValues))

    second_lane_values = histValues[index:]
    for i in range(index):
        right_lane_values[i] = 0

    count = 0
    for pixel in range(index, len(histValues)):
        right_lane_values[pixel] = second_lane_values[count]
        count = count + 1

    return right_lane_values

####
#
#  find_intersection is written by us,
#  group 4
#
####
def find_intersection(img, region=4):
    hist_values = np.sum(img[img.shape[0] // region:, :], axis=0)
    min_value = 50

    for pixel in hist_values:
        if pixel < min_value:
            return False

    return True

####
#
#  PD_control is written by us,
#  group 4.
#
####
def PD_control(Kp, Kd, cte, previous_cte):
    diff_cte = cte - previous_cte
    prop_term = -Kp * cte
    diff_term = -Kd * diff_cte
    angular_velocity = prop_term + diff_term
    return angular_velocity

####
#
#  move is written by us,
#  group 4
#
####
def move(linear_speed_in_dps, angular_velocity):

    left_motor_speed = ((2 * linear_speed_in_dps) - (angular_velocity * 117)) / (2 * 33.25)
    right_motor_speed = ((2 * linear_speed_in_dps) + (angular_velocity * 117)) / (2 * 33.25)

    #### Drive forward
    if angular_velocity == 0:
        robot.set_motor_dps(robot.MOTOR_LEFT, 100)
        robot.set_motor_dps(robot.MOTOR_RIGHT, 100)

    #### Turn left
    elif angular_velocity < 0:
        robot.set_motor_dps(robot.MOTOR_LEFT, left_motor_speed)
        robot.set_motor_dps(robot.MOTOR_RIGHT, right_motor_speed)

    #### Turn right
    elif angular_velocity > 0:
        robot.set_motor_dps(robot.MOTOR_LEFT, left_motor_speed)
        robot.set_motor_dps(robot.MOTOR_RIGHT, right_motor_speed)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    curve, intersection = getLaneCurve(image)

    if intersection:
        robot.turn_degrees(2)
        angular_velocity = 0

    angular_velocity = PD_control(Kp, Kd, curve, previous_cte)
    linear_velocity = 150
    max_angular_speed = 300

    if angular_velocity > max_angular_speed:
        angular_velocity = max_angular_speed

    move(linear_velocity, angular_velocity)

    previous_cte = curve

    rawCapture.truncate(0)

    time.sleep(0.1)