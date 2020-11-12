from easygopigo3 import EasyGoPiGo3
import cv2
from time import sleep

class Motor:
    def__init__(self, Kp, Kd)
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
            left_engine_percentage = speed_in_dps * PD_control_value
            right_engine_percentage = speed_in_dps
            gpg.steer(left_engine_percentage, right_engine_percentage)

        elif directional_flag == 2:
            left_engine_percentage = speed_in_dps
            right_engine_percentage = speed_in_dps * PD_control_value
            gpg.steer(left_engine_percentage, right_engine_percentage)

        elif directional_flag == 3:
            gpg.backward(speed_in_dps)
    
    def PD_control():
        pass
                
