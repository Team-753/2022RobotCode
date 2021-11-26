import math

wheelBase = 48
trackWidth = 36

def manualControl(joystickX, joystickY, joystickRotation):
    translationVector = (joystickX, joystickY)

    fLRotationVectorAngle = (math.atan2(wheelBase, -1*trackWidth) - (math.pi/2))
    fRRotationVectorAngle = (math.atan2(wheelBase, trackWidth) - (math.pi/2))
    rLRotationVectorAngle = (math.atan2(-1*wheelBase, -1*trackWidth) - (math.pi/2))
    rRRotationVectorAngle = (math.atan2(-1*wheelBase, trackWidth) - (math.pi/2))

    fLRotationVector = (joystickRotation*math.cos(fLRotationVectorAngle), joystickRotation*math.sin(fLRotationVectorAngle))
    fRRotationVector = (joystickRotation*math.cos(fRRotationVectorAngle), joystickRotation*math.sin(fRRotationVectorAngle))
    rLRotationVector = (joystickRotation*math.cos(rLRotationVectorAngle), joystickRotation*math.sin(rLRotationVectorAngle))
    rRRotationVector = (joystickRotation*math.cos(rRRotationVectorAngle), joystickRotation*math.sin(rRRotationVectorAngle))
    
    fLTranslationVector = (fLRotationVector[0] + translationVector[0], fLRotationVector[1] + translationVector[1])
    fRTranslationVector = (fRRotationVector[0] + translationVector[0], fRRotationVector[1] + translationVector[1])
    rLTranslationVector = (rLRotationVector[0] + translationVector[0], rLRotationVector[1] + translationVector[1])
    rRTranslationVector = (rRRotationVector[0] + translationVector[0], rRRotationVector[1] + translationVector[1])

    fLAngle = math.atan2(fLTranslationVector[1], fLTranslationVector[0])*180/math.pi
    fRAngle = math.atan2(fRTranslationVector[1], fRTranslationVector[0])*180/math.pi
    rLAngle = math.atan2(rLTranslationVector[1], rLTranslationVector[0])*180/math.pi
    rRAngle = math.atan2(rRTranslationVector[1], rRTranslationVector[0])*180/math.pi

    fLSpeed = math.sqrt((fLTranslationVector[0]**2) + (fLTranslationVector[1]**2))
    fRSpeed = math.sqrt((fRTranslationVector[0]**2) + (fRTranslationVector[1]**2))
    rLSpeed = math.sqrt((rLTranslationVector[0]**2) + (rLTranslationVector[1]**2))
    rRSpeed = math.sqrt((rRTranslationVector[0]**2) + (rRTranslationVector[1]**2))

    maxSpeed = max(fLSpeed, fRSpeed, rLSpeed, rRSpeed)
    if maxSpeed > 1:
        fLSpeed /= maxSpeed
        fRSpeed /= maxSpeed
        rLSpeed /= maxSpeed
        rRSpeed /= maxSpeed

    print(f"{fLSpeed}, {fLAngle}")
    print(f"{fRSpeed}, {fRAngle}")
    print(f"{rLSpeed}, {rLAngle}")
    print(f"{rRSpeed}, {rRAngle}")
    #print(f"Converted frontLeftAngle: {angleConversion(fLAngle)}")
    
def otherThing(x, y, z):
  robotLength = 48
  robotWidth = 36
  diagonal = math.sqrt(robotLength**2 + robotWidth**2)
  a = y - z*robotLength/diagonal
  b = y + z*robotLength/diagonal
  c = x - z*robotWidth/diagonal
  d = x + z*robotWidth/diagonal

  frontLeftSpeed = math.hypot(b,d)
  frontRightSpeed = math.hypot(a,d) #used to be b and c
  rearLeftSpeed = math.hypot(b,c) #used to be a and d
  rearRightSpeed = math.hypot(a,c)

  frontLeftAngle = math.atan2(b,d)*180/math.pi #returns -180 to 180
  frontRightAngle = math.atan2(a,d)*180/math.pi #used to be b and c
  rearLeftAngle = math.atan2(b,c)*180/math.pi #used to be a and d
  rearRightAngle = math.atan2(a,c)*180/math.pi

  maxSpeed = max(frontLeftSpeed,frontRightSpeed,rearLeftSpeed,rearRightSpeed)
  if maxSpeed > 1:
    frontLeftSpeed /= maxSpeed
    frontRightSpeed /= maxSpeed
    rearLeftSpeed /= maxSpeed
    rearRightSpeed /= maxSpeed
  
  print(f"{frontLeftSpeed}, {frontLeftAngle}")
  print(f"{frontRightSpeed}, {frontRightAngle}")
  print(f"{rearLeftSpeed}, {rearLeftAngle}")
  print(f"{rearRightSpeed}, {rearRightAngle}")
  #print(f"Converted frontLeftAngle: {angleConversion(frontLeftAngle)}")

print("Ben Code:")
manualControl(-1, -1, 0)
print("-------------------------------")
print("Old Liam Magic Code:")
otherThing(-1, -1, 0)
