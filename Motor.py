####
#
# This class controls in what direction the
# robot should be driving, depending on the
# curvature of the track
#
# PD_control should probably not be
# implemented here????
#
####


from easygopigo3 import EasyGoPiGo3

class Motor:
    def __init__(self):
        self.gpg = EasyGoPiGo3()
        self.left_motor_speed
        self.right_motor_speed

    ####
    #
    # Unicycle model - based on linear and angular velocity
    # Trying to keep the velocity low, for better performance.
    # Hardware wise the maximum speed can be set to 1000dps
    # We will keep the maximum speed to 300dps.
    # Linear speed is set in the main loop, we have chosen
    # 200dps to be the default linear speed.
    #
    # Curvature will define the total speed.
    #
    # if angular_speed == 0, then the robot is exactly on the lane and should
    # drive forward.
    #
    # if angular_speed is less than 0 than the robot is on the right side
    # of the lane and should turn left.
    #
    # if angular_speed is more than 0 then the robot is on the left side
    # of the lane and should turn right.
    #
    # The time signature is for how long the method should be operational,
    # have to investigate more on the functionallity of this. Maybe
    # it is not needed at all.
    #
    ####

    def move(self, linear_speed_in_dps, angular_velocity, time=2):

        linear_speed = linear_speed_in_dps*100
        angular_speed = angular_velocity*70
        left_motor_speed = linear_speed + angular_speed
        right_engine_speed = linear_speed - angular_speed

        #### Drive forward
        if angular_velocity == 0:
            gpg.forward()

        #### Turn left
        elif angular_velocity < 0:
            gpg.set_motor_dps(left_motor_speed, right_engine_speed)

        #### Turn right
        else:
            gpg.set_motor_dps(left_motor_speed, right_engine_speed)