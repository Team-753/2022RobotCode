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

print(winchRotationLookup(42))

print(lookupValue(42))
