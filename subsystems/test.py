import math

x = 0
y = 22
armLength = 11
forearmLength = math.hypot(16.5, 2.5)

theta1 = math.acos(x/(math.hypot(x,y)))
theta2 = math.acos(((x**2) + (y**2) + (armLength**2) - (forearmLength**2))/(2*armLength*math.hypot(x,y))) - (math.pi/4)

theta = (math.pi/2) + theta1 - theta2

print(theta1)
print(theta2)
print(theta)