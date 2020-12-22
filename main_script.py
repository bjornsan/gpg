% matplotlib
inline
from easygopigo3 import EasyGoPiGo3
from picamera.array import PiRGBArray
from picamera import PiCamera
from matplotlib import pyplot as plt
import time
import cv2
import numpy as np

####
#
#  Globals
#
####
curveList = []
avgVal = 10
previous_cte = 0.0
Kp = 5
Kd = 2.5
linear_velocity = 100
max_angular_speed = 300

####
#
#  Robot and camera initialization
#
####
robot = EasyGoPiGo3()
camera = PiCamera()
# (w, h)
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
#  Parameters: image - the raw image from camera feed to process
#
#  Return values: float curve, bool intersect
#  curve is the curvature of the track.
#  intersect depends on if there was found an intersection or not
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

    intersect = find_intersection(imgWarp)

    if intersect:
        return 0, intersect

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    #### NORMALIZATION
    curve = curve / 100
    if curve > 1: curve = 1
    if curve < -1: curve = -1

    # print("Curve_value: {}".format(curve))
    print(f"Curve_value: {curve}")

    return curve, intersect


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
    two_lanes = False

    if region == 1:
        hist_values = np.sum(img, axis=0)
    else:
        hist_values = np.sum(img[img.shape[0] // region:, :], axis=0)

    if region == 4:
        two_lanes, right_lane_index = find_two_lanes(hist_values)

        if two_lanes:
            hist_values = get_right_lane_histogram(hist_values, right_lane_index)
            # robot.turn_degrees(0)

    min_threshold = get_min_threshold(hist_values, minPer)
    index_array = np.where(hist_values >= min_threshold)
    base_point = int(np.average(index_array))

    if two_lanes:
        return -base_point

    return base_point


def get_min_threshold(hist_values, min_per):
    max_value = np.max(hist_values)
    min_threshold = min_per * max_value
    return min_threshold


####
#
#  find_two_lanes is written by us,
#  group 4.
#
####
def find_two_lanes(hist_values):
    min_per = 0.4
    min_threshold = get_min_threshold(hist_values, min_per)
    intensity_flag = 0

    for index, pixel_sum in enumerate(hist_values):
        if intensity_flag == 0 and pixel_sum > min_threshold:
            intensity_flag = 1
        if intensity_flag == 1 and pixel_sum < min_threshold:
            intensity_flag = 2
        if intensity_flag == 2 and pixel_sum > min_threshold:
            print("two lanes found")
            return True, index

    print("one lane only")
    return False, 0


####
#
#  getRightLaneHistogram is written by us,
#  group 4.
#
####
def get_right_lane_histogram(hist_values, index_of_right_lane):
    hist_length = len(hist_values)
    right_lane_hist = np.zeros(hist_length, dtype=int)
    right_lane_values = hist_values[index_of_right_lane:]

    count = 0
    for i in range(index_of_right_lane, hist_length):
        right_lane_hist[i] = right_lane_values[count]
        count = count + 1

    return right_lane_hist


####
#
#  find_intersection is written by us,
#  group 4
#
####
def find_intersection(img, region=4):
    hist_values = np.sum(img[img.shape[0] // region:, :], axis=0)
    # hist_values = np.sum(img, axis=0)
    min_value = 200

    for pixel in hist_values:
        if pixel < min_value:
            return False

    print("found intersection")
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
#  angular_velocity > 0 = turn right
#  angular_velocity < 0 = turn left
#
####
def move(linear_speed_in_dps, angular_velocity):
    wheel_base = 117
    wheel_radius = 33.25

    left_motor_speed = (((2 * linear_speed_in_dps) - (angular_velocity * wheel_base)) / (2 * wheel_radius)) * 2
    right_motor_speed = (((2 * linear_speed_in_dps) + (angular_velocity * wheel_base)) / (2 * wheel_radius)) * 2

    #### Drive forward
    if angular_velocity == 0:
        robot.set_motor_dps(robot.MOTOR_LEFT, linear_speed_in_dps)
        robot.set_motor_dps(robot.MOTOR_RIGHT, linear_speed_in_dps)
    #### Turn left/ right
    else:
        robot.set_motor_dps(robot.MOTOR_LEFT, left_motor_speed)
        robot.set_motor_dps(robot.MOTOR_RIGHT, right_motor_speed)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    cte, intersection = getLaneCurve(image)

    if intersection:
        robot.turn_degrees(2)
        # curve = 0.5
    if cte < 0.03 and cte > -0.03:
        cte = 0

    angular_velocity = PD_control(Kp, Kd, cte, previous_cte)

    if angular_velocity > max_angular_speed:
        angular_velocity = max_angular_speed

    move(linear_velocity, angular_velocity)

    previous_cte = cte

    rawCapture.truncate(0)
    time.sleep(0.1)