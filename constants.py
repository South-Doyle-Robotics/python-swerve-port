import math
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d

trackWidthIn = 21.5
wheelBaseIn =  21.5
wheelDiameterIn = 3

pinionTeeth = 13
driveGearRatio = (45 * 22) / (pinionTeeth * 15)

gyro_reversed = True
#safety limits; change later
maxSpeedMPS = 4.8
maxAngularSpeed = 2 *math.pi

gyro_reset_value = 0

trackWidthM = trackWidthIn * 2.54 / 100
wheelBaseM = wheelBaseIn * 2.54 / 100
wheelDiameterM = wheelDiameterIn * 2.54 / 100
wheelCircumferenceM = wheelDiameterM * math.pi

neoMotorFreeSpeedRPM = 5676
driveMotorFreeSpeedRps = neoMotorFreeSpeedRPM / 60
driveWheelFreeSpeedRps = (driveMotorFreeSpeedRps * wheelCircumferenceM) / driveGearRatio

driveEncPositionFactor = (wheelDiameterM * math.pi) / driveGearRatio
driveEncVelocityFactor = ((wheelDiameterM * math.pi) / driveGearRatio) / 60.0

driveMin = -1
driveMax = 1
turnMin = -1
turnMax = -1

driveP = .04
driveI = 0
driveD = 0
driveFF= 1 / driveWheelFreeSpeedRps

turnP = 1
turnI = 0
turnD = 0
turnFF = 0

# for field oriented steering? 
thetaP = 1
thetaI = 0
thetaD = 0
turnEncPositionFactor = (2 * math.pi)
turnEncVelocityFactor = (2 * math.pi) / 60.0

FL = {'driveID': 3, 'turnID': 2, 'offset': -math.pi/2.0}
FR = {'driveID': 5, 'turnID': 4, 'offset': 0}
BL = {'driveID': 7, 'turnID': 6, 'offset': math.pi}
BR = {'driveID': 9, 'turnID': 8, 'offset': math.pi/2.0}



kinematics = SwerveDrive4Kinematics(
    Translation2d(wheelBaseM / 2.0, trackWidthM / 2.0), 
    Translation2d(wheelBaseM / 2.0, -trackWidthM / 2.0), 
    Translation2d(-wheelBaseM / 2.0, trackWidthM / 2.0),
    Translation2d(-wheelBaseM / 2.0, -trackWidthM / 2.0))

