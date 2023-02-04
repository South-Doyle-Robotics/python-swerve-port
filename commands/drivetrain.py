import math
#from numpy import sign # why is this not in math?

from wpimath.geometry import Pose2d, Translation2d, Rotation2d

from commands2 import Command, CommandBase

import wpimath.kinematics

# this isn't used yet. maybe someday we can get basic swerve working THEN fight the
# commands battle.

class Drive_Swerve(CommandBase):
	def __init__(self, drivetrain, getLeftRight, getForwardBack, getRotate):
		super().__init__()
		print("drive_swerve init")
		self.drivetrain = drivetrain

		#self.hasRequirement(self.drivetrain)
		self.addRequirements([self.drivetrain])

		self.getLeftRight = getLeftRight
		self.getForwardBack = getForwardBack
		self.getRotate = getRotate

	def getRequirements(self):
		return set([self.drivetrain])

	def execute(self):
		print("swerve drive command")
		assert False
		lr = self.getLeftRight
		fb = self.getForwardBack
		rot = self.getRotate


		chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
			vx = fb,
			vy = -lr,
			omega = rotate*1.1,
			robotAngle = self.drivetrain.getGyroHeading()
		)

		self.drivetrain.drive(chassisSpeeds)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.drivetrain.stop()

	def interrupted(self):
		self.end(True)

class Drive_Swerve_Field_Steering(Command):
	def __init__(self, getLeftRight, getForwardBack, getTurnX, getTurnY):
		super().__init__()
		

		self.hasRequirement(self.drivetrain)

		self.getLR = getLeftRight
		self.getFB = getForwardBack
		self.turnX = getTurnX
		self.turnY = getTurnY
		print("Drive swerve field steering init")

	def getRequirements(self):
		return set([self.drivetrain])

	def execute(self):
		print("Execute....")
		lr = self.getLR
		fb = self.getFB
		tx = self.turnX
		ty = self.turnY

		magnitude = math.sqrt(tx*tx+ty*ty)

		if math.abs(tx) > 0.05 or math.abs(ty) > 0.05:
			target_heading = (atan2(ty,tx)+math.pi/2)%(2*math.pi)
			self.drivetrain.targetHeading = target_heading
		else:
			target_heading = self.drivetrain.targetHeading

		currentHeading = self.drivetrain.getGyroHeading().radians()

		if abs(targetHeading - currentHeading) > math.pi:
			targetHeading += 2*math.pi * math.copysign(1, (currentHeading - targetHeading))

		omega = self.drivetrain.thetaPID.calculate(currentHeading, targetHeading)

		chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
			vx = fb,
			vy = -lr,
			omega = omega,
			robotAngle = self.drivetrain.getGyroHeading()
		)
		
		self.drivetrain.drive(chassisSpeeds)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.drivetrain.stop()

	def interrupted(self):
		self.end(True)

class setGyro(Command):
	def __init__(self, newHeading = 0):
		super().__init__()
	
		self.newHeading = newHeading

		self.hasRequirements(self.drivetrain)
	
	def getRequirements(self):
		return set([self.drivetrain])

	def execute(self):
		self.drivetrain.setGyro(self.newHeading)

	def isFinished(self):
		return True
	
	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end(True)

class resetOdometry(Command):
	def __init__(self):
		super().__init__()
	
		self.hasRequirement(self.drivetrain)

	def getRequirements(self):
		return set([self.drivetrain])

	def execute(self):
		self.drivetrain.odometry.resetPosition(Pose2d(Translation2d(0,0), Rotation2d(0)), Rotation2d(0))

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end(True)

		
