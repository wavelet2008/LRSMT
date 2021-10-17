"""
single_leg controller.
This FILE is part of multi-legged robot field exploration model
dataset_making.py to make data collection process dataset from human interaction

This programm is explained by roboLAND in university of southern california.
Please notify the source if you use it

Copyright(c) 2021-2025 Ryoma Liu
Email: 1196075299@qq.com
"""
#from controller import Robot, Motor, DistanceSensor

from controller import Robot
import math
import time
# create the Robot instance.
class SinglelegController(Robot):
    def __init__(self):
        super(SinglelegController, self).__init__()
        self.timeStep = 64 #time for controller simulation
        self.motor1 = self.getDevice("motor1")
        self.motor2 = self.getDevice("motor2")
        self.upperleg_length = 0.1
        self.lowerleg_length = 0.2
        self.toe_length = 0.025
    

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
        fail = math.acos((l ** 2 + 0.1 ** 2 - 0.1 ** 2) / (2 * l * 0.2))
        sita1 = fail - psail
        sita2 = fail + psail
        # transfer from the motor position
        sita1 = sita1 - 1.276569489045914
        sita2 = 1.276569489045914 - sita2
        return [sita1, sita2]
        

    def run(self):
        # main loop control
        for x in range(400):
            x_axis = -0.116 - 0.001 * x
            sita = self.inverse_kinematic([x_axis, 0])
            self.motor1.setPosition(sita[0])
            self.motor2.setPosition(sita[1])

print("test_if_here")
controller = SinglelegController()
controller.run()
