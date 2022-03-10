import math

x = 10
y = 10
armLength = 11
forearmLength = math.hypot(16.5, 2.5)
pulleyHeight = 3

theta1 = math.acos(x/(math.hypot(x,y)))
theta2 = math.acos(((x**2)+(y**2)+(armLength**2)-(forearmLength**2))/(2*armLength*math.hypot(x,y)))

theta = (math.pi/2) + theta1 - theta2
length = math.hypot(x, y - pulleyHeight)

print(f"\ntesting... \nx: {x} \ntheta1: {theta1} \ntheta2: {theta2} \ny: {y} \nangle: {theta} \nlength: {length}\n")