for i in range(-180,180):
    angle = i
    if angle < -90:
        angle += 270
    else:
        angle -= 90
    angle = -angle
    print(str(i) + ", " + str(angle))