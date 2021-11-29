import math

def inverse_kinematic(endpoint):
    '''This function converts foot end point to a pair of motor positions
    endpoint:     foot point[x,y], 
                    assume that the center point is the center of motor
    motorpoint:   motor position[x,y]

    '''
    x = endpoint[1]
    y = - endpoint[0]
    l = math.sqrt( x ** 2 + y ** 2) # virutal leg length
    psail = math.asin(x/l)
    fail = math.acos((l ** 2 + 0.1 ** 2 - 0.2 ** 2) / (2 * l * 0.1))
    sita1 = fail - psail
    sita2 = fail + psail
    return [sita1, sita2]


print(inverse_kinematic([-0.116, 0]))