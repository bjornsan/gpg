from Motor import Motor
from LaneDetectionModule import getLaneCurve
import Utilities
import cameraModule

####
Kp = 5
Kd = 2.5
previous_cte = 0.0

motor = Motor()
####

def main():
    global previous_cte

    img = cameraModule.getImg()
    cte, intersection = getLaneCurve(img, 1)

    if intersection:
        # change region of interest to right side
        motor.gpg.turn_degrees(90)
        angular_velocity = 0
    else:
        # normal region of interest
        angular_velocity = Utilities.PD_control(Kp, Kd, cte, previous_cte)

    linear_velocity = 100
    max_angular_speed = 200
    if angular_velocity > max_angular_speed:
        angular_velocity = max_angular_speed
    if angular_velocity < -max_angular_speed:
        angular_velocity = -max_angular_speed

    if -0.05 < angular_velocity < 0.05:
        angular_velocity = 0

    motor.move(linear_velocity, angular_velocity)
    previous_cte = cte

    # only used if image is needed to be displayed
    # cv2.waitKey(1)


if __name__ == '__main__':
    while True:
        main()
