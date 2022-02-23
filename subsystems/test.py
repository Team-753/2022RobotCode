import math

axleRadius = 2.55
strapThickness = 0.1
numberOfWraps = 5
scale = 360
length = 0
aList = []
for i in range(0,numberOfWraps*scale):
    i /= scale
    currentRadius = math.ceil(numberOfWraps - i)*strapThickness + axleRadius
    length += currentRadius*2*math.pi/scale
    aList.append((i, length))


def lookupValue(distance):
    for i in aList:
        if i[1] > distance:
            index = aList.index(i) - 1
            break
    return(aList[index])

def winchRotationLookup(distance):
        '''This finds the number of rotations you need the winch to do (approximately) based on a distance you want the winch to let out.'''
        for i in aList:
            if i[1] > distance:
                index = aList.index(i)
                break
        a = aList[index - 1]
        b = aList[index]
        deltaD1 = b[1] - a[1]
        deltaD2 = distance - a[1]
        ratioD = deltaD2/deltaD1
        deltaR = b[0] - a[0]
        rotationValue = a[0] + (deltaR*ratioD)
        return((rotationValue, distance))

def getArmInverseKinematics(x, y):
    '''This returns the angle of the shoulder and the length of strap let out of the winch based on the desired x and y position of the arm hook.'''
    L1 = 11
    L2 = 2.5
    L3 = 16.5
    L4 = math.hypot(L2, L3)
    L6 = 3
    theta = math.acos(x/(math.hypot(x,y))) - math.acos(((x**2)+(y**2)+(L1**2)-(L4**2))/(2*L1*math.hypot(x,y))) - (math.pi/4)
    L5 = math.hypot(x,(y-L6))
    return(L5, theta)