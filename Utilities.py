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

    if len_corners >= 4 and dx < 50 and dy < 50:
        return True

    return False


def find_intersection(img, region=4):
    hist_values = np.sum(img[img.shape[0] // region:, :], axis=0)
    # hist_values = np.sum(img, axis=0)
    min_value = 200

    for pixel in hist_values:
        if pixel < min_value:
            return False

    print("found intersection")
    return True


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


def get_right_lane_histogram(hist_values, index_of_right_lane):
    hist_length = len(hist_values)
    right_lane_hist = np.zeros(hist_length, dtype=int)
    right_lane_values = hist_values[index_of_right_lane:]

    count = 0
    for i in range(index_of_right_lane, hist_length):
        right_lane_hist[i] = right_lane_values[count]
        count = count + 1

    return right_lane_hist


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

def PD_control(Kp, Kd, cte, previous_cte):
    diff_cte = cte-previous_cte
    prop_term = -Kp * cte
    diff_term = -Kd * diff_cte
    angular_velocity = prop_term + diff_term
    return angular_velocity


def PID(Kp, Kd, Ki, N):
    int_crosstrack_error = 0.0

    crosstrack_error = my_robot.y

    for i in range(N):
        diff_crosstrackerror = my_robot.y - crosstrack_error
        crosstrack_error = my_robot.y
        int_crosstrack_error += crosstrack_error
        steer = -Kp * crosstrack_error \
                -Kd * diff_crosstrackerror \
                -Ki * int_crosstrack_error
        my_robot = myrobot.move(steer, speed)


def PD(Kp, Kd, N)
    robot_pos_y = 0
    crosstrack_error = robot_pos_y
    diff_crosstrackerror = robot_pos_y - crosstrack_error

    cte = present_curve
    dcte = present_curve - cte
    real_curve = -Kp * cte -Kd*dcte
