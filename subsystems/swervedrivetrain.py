
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState
import navx
from subsystems import swerve
import math
import constants

from wpimath.controller import PIDController

from commands2 import SubsystemBase

class drivetrain(SubsystemBase):
	def __init__(self):
		super().__init__()
		print("swerve drivetrain drivetrain init")

		self.FL = swerve.MAXSwerveModule(constants.FL['driveID'],
					  constants.FL['turnID'],
					  constants.FL['offset'])

		self.FR = swerve.MAXSwerveModule(constants.FR['driveID'],
					  constants.FR['turnID'],
					  constants.FR['offset'])

		self.BL = swerve.MAXSwerveModule(constants.BL['driveID'],
					  constants.BL['turnID'],
					  constants.BL['offset'])

		self.BR = swerve.MAXSwerveModule(constants.BR['driveID'],
					  constants.BR['turnID'],
					  constants.BR['offset'])


		self.gyro = navx.AHRS.create_spi()
		self.gyro_offset = 0
		self.gyro.reset() #setFusedHeading(constants.gyro_reset_value)

		self.targetHeading = math.radians(self.gyro.getAngle())
		self.kinematics = constants.kinematics

		# Can also pass states of all 4 modules, but they are zeroed at this time 
                # Make sure this is radians for fusedHeading
		pos = self.getAllPositions()
		self.odometry = SwerveDrive4Odometry(self.kinematics, 
							Rotation2d.fromDegrees(self.gyro.getAngle()),
							pos)

		self.thetaPID = PIDController(constants.thetaP, constants.thetaI, constants.thetaD)
		self.thetaPID.setTolerance(math.radians(2)) # why?

#		self.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	def getPose2d(self):
		return self.odometry.getPoseMeters()

	def getGyroHeading(self):
		return Rotation2d.fromDegrees((self.gyro.getAngle() - self.gyro_offset) % 360)

	def getGyroHeadingDegrees(self):
		return self.getGyroHeading().degrees()

	def setGyro(self, newHeading = 0):
		self.gyro_offset = self.gyro.getAngle() - newHeading

	def getOdometryRotation(self):
		return self.odometry.getPose().rotation()

	def drive(self, xSpeed, ySpeed, rot, field=False):
		self.ySpeed = ySpeed * constants.maxSpeedMPS
		self.xSpeed = xSpeed * constants.maxSpeedMPS
		self.rot = rot * constants.maxAngularSpeed
                
		states = []
		if field is True:
			self.cs = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.rot, Rotation2d.fromDegrees(self.gyro.getAngle()))
		else:
			self.cs = ChassisSpeeds(self.xSpeed, self.ySpeed, self.rot)
		print("Chassis speed: ", self.xSpeed, self.ySpeed, self.rot)
		states = constants.kinematics.toSwerveModuleStates(self.cs)
		#self.FL.setDesiredState(states[0])
		#self.FR.setDesiredState(states[1])
		#self.BL.setDesiredState(states[2])
		self.BR.setDesiredState(states[3])

	def getAllStates(self):
		fl = self.FL.getState()
		fr = self.FR.getState()
		bl = self.BL.getState()
		br = self.BR.getState()
		return(fl, fr, bl, br)

	def getAllPositions(self):
		fl = self.FL.getPosition()
		fr = self.FR.getPosition()
		bl = self.BL.getPosition()
		br = self.BR.getPosition()
		return(fl, fr, bl, br)

	
	def periodic(self):
#		print("Drive train periodic")

		pass
# This does get called .. periodically. So is it not getting joystick updates?
		self.odometry.update(self.gyro.getRotation2d(), 
			self.FL.getPosition(),
			self.FR.getPosition(),
			self.BL.getPosition(),
			self.BR.getPosition())

		
#		states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
#		
#		states = SwerveDrive4Kinematics.desaturateWheelSpeeds(states, constants.maxSpeedMpS)
#		self.FL.setDesiredState(states[0].speed, states[0].angle) # degrees
#		self.FR.setDesiredState(states[1].speed, states[1].angle)
#		self.BL.setDesiredState(states[2].speed, states[2].angle)
#		self.BR.setDesiredState(states[3].speed, states[3].angle)

	def stop(self):
		self.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

		

