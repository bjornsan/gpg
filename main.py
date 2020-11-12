from Motor import Motor
from LaneDetectionModule import getLaneCurve
import cameraModule

####
motor = Motor(Kp, Kd)
####

def main():
    img = cameraModule.getImg()
    curveVal = getLaneCurve(img, 1)

    sen = 1.3  # SENSITIVITY
    maxVAl = 0.3  # MAX SPEED
    if curveVal > maxVAl: curveVal = maxVAl
    if curveVal < -maxVAl: curveVal = -maxVAl
    # print(curveVal)
    if curveVal > 0:
        sen = 1.7
        if curveVal < 0.05: curveVal = 0
    else:
        if curveVal > -0.08: curveVal = 0
    motor.move(0.20, -curveVal * sen, 0.05)
    # cv2.waitKey(1)


if __name__ == '__main__':
    while True:
        main()
