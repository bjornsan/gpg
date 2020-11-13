from easygopigo3 import EasyGoPiGo3
from time import sleep

class Motor:
    def__init__(self, Kp, Kd):
    self.gpg = EasyGoPiGo()
    self.Kp = Kp
    self.Kd = Kd

    ####
    #
    # Directional flags:
    # 
    # 0 = forward
    # 1 = left
    # 2 = right
    # 3 = backward
    #
    ####

    def move(directional_flag, speed_in_dps):
        if directional_flag == 0:
            gpg.forward(speed_in_dps)

        elif directional_flag == 1:
            left_motor_speed = speed_in_dps + PD_control_value
            right_motor_speed = speed_in_dps - PD_control_value
            gpg.set_motor_dps(gpg.MOTOR_LEFT, dps=leftMotorSpeed)
            gpg.set_motor_dps(gpg.MOTOR_RIGHT, dps=rightMotorSpeed)

        elif directional_flag == 2:
            left_engine_percentage = speed_in_dps
            right_engine_percentage = speed_in_dps * PD_control_value
            gpg.steer(left_engine_percentage, right_engine_percentage)

        elif directional_flag == 3:
            gpg.backward(speed_in_dps)
    
   # def PD_control():
   #     correction = Kp * Kp_error + Kd * (error - previous_error)
   #	 return correction
                
