from Motor import Motor
from LaneDetectionModule import getLaneCurve
import Utilities
import cameraModule

####
Kp = 1
Kd = 1
previous_cte = 0.0

motor = Motor()
####

###
#
# angular_velocity > 0 robot turns right
# angular_velocity < 0: robot turns left
# angular_velocity == 0: robot goes straight
#
###
def main():
    global previous_cte

    img = cameraModule.getImg()
    cte = getLaneCurve(img, 1)

    angular_velocity = Utilities.PD_control(Kp, Kd, cte, previous_cte)
    linear_velocity = 200

    max_angular_speed = 300
    if angular_velocity > max_angular_speed:
        angular_velocity = max_angular_speed
    if angular_velocity < -max_angular_speed:
        angular_velocity = -max_angular_speed


    motor.move(linear_velocity, angular_velocity)


    previous_cte = cte

    # only used if image is needed to be displayed
    # cv2.waitKey(1)


if __name__ == '__main__':
    while True:
        main()
