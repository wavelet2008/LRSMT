"""
single_leg controller.
This FILE is part of multi-legged robot field exploration model

This programm is explained by roboLAND in university of southern california.
Please notify the source if you use it

Copyright(c) 2021-2025 Ryoma Liu
Email: shipengl@usc.edu
"""
#from controller import Robot, Motor, DistanceSensor

from controller import Robot
from curve_helper.bs_pline_path import bspline_planning
import math
import time
import numpy as np

# create the Robot instance.
class Singleleg(Robot):
    def __init__(self):
        super(Singleleg, self).__init__()
        self.timeStep = 100 #time for controller simulation
        self.motor1 = self.getDevice("motor1")
        self.motor2 = self.getDevice("motor2")
        self.keyboard = self.getKeyboard()
        self.basictimestep = int(self.getBasicTimeStep())
        self.keyboard.enable(self.basictimestep)
        self.upperleg_length = 0.1
        self.lowerleg_length = 0.2
        self.toe_length = 0.025
        self.foot_step = 0.15 #reasonable value between 0.1 and 0.2
        self.step_height = 0.7 #reasonable value between 0.05 and 0.1
        self.curve_ratio = 0.6   #reasonable value between 0.5 and 1
    

    def inverse_kinematic(self, endpoint):
        '''This function converts foot end point to a pair of motor positions
        endpoint:     foot point[x,y], 
                      assume that the center point is the center of motor
        motorpoint:   motor position[x,y]

        '''
        x = endpoint[1]
        y = - endpoint[0]
        l = math.sqrt( x ** 2 + y ** 2) # virutal leg length
        psail = math.asin(x/l)
        fail = math.acos((l ** 2 + self.upperleg_length ** 2 - self.lowerleg_length ** 2) / (2 * l * self.upperleg_length))

        sita1 = fail - psail
        sita2 = fail + psail
   
        # transfer from the motor position
        sita1 = sita1 - 2.364711429998645
        sita2 = 2.364711429998645 - sita2
        return [sita1, sita2]

    def motor_control(self, sita):
        self.motor1.setPosition(sita[0])
        self.motor2.setPosition(sita[1])

    def set_trajetory_parameter(self, foot_step_, step_height, curve_ratio_):
        self.foot_step = foot_step_
        self.step_height = step_height
        self.curve_ratio = curve_ratio_

    def generate_b_spline_trajectory(self):
        # define the five points for B-spline curve
        point_a = [0.5*self.foot_step, -0.116]
        point_b = [0.25*self.foot_step, -0.116 + self.curve_ratio * self.step_height]
        point_c = [0, -0.116 + self.step_height]
        point_d = [-0.25*self.foot_step, -0.116 + self.curve_ratio * self.step_height]
        point_e = [-0.5*self.foot_step, -0.116]
        point_list = [point_a, point_b, point_c, point_d, point_e]
        # generate b-spline curve point
        x, y, b_trajectories = bspline_planning(point_list, 100)
        return b_trajectories


    def run(self):
        # Initial position
        x = 0.5*self.foot_step
        y = -0.116
        sita = self.inverse_kinematic([x,y])
        self.motor_control(sita)
        times = 0
        i = -0.2
        sign = 1
        # main loop control
        while(self.step(self.timeStep) != -1):
            times += 1
            '''key board control
            '''
            # #wait for keyboard reponse
            # print(x,y)
            # key = self.keyboard.getKey()
            # if(key == 315):
            #     print('press w')
            #     x -= 0.01
            # elif(key == 317):
            #     x += 0.01
            # elif(key == 316):
            #     y -= 0.01
            # elif(key == 314):
            #     y += 0.01
            # sita = self.inverse_kinematic([x,y])

            # generate the foot trajectories
            # trajectories = self.generate_b_spline_trajectory()
            # for waypoints in trajectories:
            #     sita = self.inverse_kinematic(waypoints)
            #     self.motor_control(sita)
            #     print("iterate", times)

            
            sita = self.inverse_kinematic([0, i])
            self.motor_control(sita)
            print("y position", i)

            if(sign == 1):
                if(i>-0.26):
                    i = i - 0.01
                else:
                    sign = -1
            else:
                if(i<-0.2):
                    i = i + 0.01
                else:
                    sign = 1


            
controller = Singleleg()
controller.run()
