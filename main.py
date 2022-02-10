'''directionReversed = False
def optimize(moduleAngle: float, moduleTarget: float):
        normal = abs(moduleAngle - moduleTarget)
        oppositeAngle = moduleAngle - 180
        if oppositeAngle < -180:
            oppositeAngle += 360
        print(f"Opposite Angle: {oppositeAngle}")
        print(f"NormalDistance: {normal}")
        oppositeDistance = moduleTarget - oppositeAngle
        print(f"OppositeDistance: {oppositeDistance}")
        if oppositeDistance < normal:
            directionReversed = True
            return oppositeAngle, directionReversed
        else:
            directionReversed = False
            return moduleAngle, directionReversed
for i in range(1):
    print(optimize(89 - i, 180))
    print(directionReversed)
print(optimize(50, -90))'''
import math
checkPoint = (5, 10)
currentPoint = (0, 15)
x = checkPoint[0] - currentPoint[0]
y = checkPoint[1] - currentPoint[1]
print(x)
print(y)