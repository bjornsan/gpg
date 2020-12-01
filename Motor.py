from easygopigo3 import EasyGoPiGo3

class Motor:
    def __init__(self):
        self.gpg = EasyGoPiGo3()
        self.wheel_radius = 33.25
        self.wheel_base = 117
        # spesificed by gopigo docs:
        # wheel diameter = 66.5
        # wheel base = 117

    def move(self, linear_speed, angular_velocity):
        # Taking the wheel base and wheeel radius in consideration
        left_motor_speed = int((( 2 * linear_speed ) - (angular_velocity * self.wheel_base)) / ( 2 * self.wheel_radius))
        right_motor_speed = int((( 2 * linear_speed ) + (angular_velocity * self.wheel_base)) / ( 2 * self.wheel_radius))

        if left_motor_speed > 300:
            left_motor_speed = 300
        if right_motor_speed > 300:
            right_motor_speed = 300

        #### Drive forward
        if angular_velocity == 0:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, linear_speed)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, linear_speed)
        #### Turn right
        elif angular_velocity > 0:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, left_motor_speed)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, right_motor_speed)
        #### Turn left
        elif angular_velocity < 0:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, left_motor_speed)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, right_motor_speed)


