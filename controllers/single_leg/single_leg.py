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
        self.keyboard = self.getKeyboard()
        self.basictimestep = int(self.getBasicTimeStep())
        self.keyboard.enable(self.basictimestep)
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


    def run(self):
        # Initial position
        x = -0.116
        y = 0
        sita = self.inverse_kinematic([x,y])
        self.motor_control(sita)
        
        # main loop control
        while(self.step(self.timeStep) != -1):
            #wait for keyboard reponse
            print(x,y)
            key = self.keyboard.getKey()
            if(key == 315):
                print('press w')
                x -= 0.01
            elif(key == 317):
                x += 0.01
            elif(key == 316):
                y -= 0.01
            elif(key == 314):
                y += 0.01
            sita = self.inverse_kinematic([x,y])
            self.motor_control(sita)

print("test_if_here")
controller = SinglelegController()
controller.run()
