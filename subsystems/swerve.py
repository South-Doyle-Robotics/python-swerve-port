
import wpilib

import math
import constants

from rev import CANSparkMax
from rev import CANSparkMaxLowLevel
from rev import SparkMaxPIDController
from rev import SparkMaxAbsoluteEncoder
from rev import AbsoluteEncoder
from rev import RelativeEncoder

from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition

from wpimath.geometry import Rotation2d, Translation2d, Pose2d


class MAXSwerveModule:
	def __init__(self, drive_id, turn_id, angle_offset):
		self.drive_id = drive_id
		self.turn_id = turn_id
		self.angle_offset = angle_offset

		self.swerve_state = SwerveModuleState(0.0, Rotation2d(0.0)) # initialize to 0 speed, 0 angle
		self.swerve_position = SwerveModulePosition(0.0, Rotation2d(0.0)) # initial position/angle 
		
		self.drivemotor = CANSparkMax(self.drive_id, CANSparkMaxLowLevel.MotorType.kBrushless)
		self.turnmotor = CANSparkMax(self.turn_id, CANSparkMaxLowLevel.MotorType.kBrushless)
		
		self.drivemotor.restoreFactoryDefaults() 
		self.turnmotor.restoreFactoryDefaults()

		self.driveenc = self.drivemotor.getEncoder()
		self.turnenc = self.turnmotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
		self.drivepid = self.drivemotor.getPIDController()
		self.turnpid = self.turnmotor.getPIDController()

		self.drivepid.setFeedbackDevice(self.driveenc)
		self.turnpid.setFeedbackDevice(self.turnenc)
		
		self.driveenc.setPositionConversionFactor(constants.driveEncPositionFactor)
		self.driveenc.setVelocityConversionFactor(constants.driveEncVelocityFactor)

		self.turnenc.setPositionConversionFactor(constants.turnEncPositionFactor)
		self.turnenc.setVelocityConversionFactor(constants.turnEncVelocityFactor)

		self.turnenc.setInverted(True)

		self.turnpid.setPositionPIDWrappingEnabled(True)
		self.turnpid.setPositionPIDWrappingMinInput(0.0) # radians
		self.turnpid.setPositionPIDWrappingMaxInput(2 * math.pi) # radians, 360

		self.drivepid.setP(constants.driveP)
		self.drivepid.setI(constants.driveI)
		self.drivepid.setD(constants.driveD)
		self.drivepid.setFF(constants.driveFF)
		self.drivepid.setOutputRange(constants.driveMin, constants.driveMax)

		self.turnpid.setP(constants.turnP)
		self.turnpid.setI(constants.turnI)
		self.turnpid.setD(constants.turnD)
		self.turnpid.setFF(constants.turnFF)
		self.turnpid.setOutputRange(constants.turnMin, constants.turnMax)

		self.drivemotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
		self.turnmotor.setIdleMode(CANSparkMax.IdleMode.kBrake)

		self.drivemotor.setSmartCurrentLimit(50) # amps
		self.turnmotor.setSmartCurrentLimit(20)


		# Kinda scary 
	#	self.drivemotor.burnFlash()
	#	self.turnmotor.burnFlash()

                # These are in radians in constants.py
		self.chassisAngularOffset = angle_offset
		self.swerve_state.angle = Rotation2d(self.turnenc.getPosition())
		self.driveenc.setPosition(0)


	def setDrivePID(self, p, i, d, ff, min=constants.driveMin, max=constants.driveMax):
		self.drivepid.setP(p)
		self.drivepid.setI(i)
		self.drivepid.setD(d)
		self.drivepid.setFF(ff)
		self.drivepid.setOutputRange(min, max)

	def setTurnPID(self, p, i, d, ff, min=constants.turnMin, max=constants.turnMax):
		self.turnpid.setP(p)
		self.turnpid.setI(i)
		self.turnpid.setD(d)
		self.turnpid.setFF(ff)
		self.turnpid.setOutputRange(min, max)

	def getState(self): 
		swerve_state = SwerveModuleState(self.driveenc.getVelocity(), 
			       Rotation2d(self.turnenc.getPosition() - self.chassisAngularOffset))
		return swerve_state

	def getPosition(self):
		swerve_pos = SwerveModulePosition(self.driveenc.getPosition(),
				  Rotation2d(self.turnenc.getPosition() - self.chassisAngularOffset))
		return swerve_pos

        # The Java takes in a modulestate rather than speed/angle. verify where this is used passes speed/angle
	def setDesiredState(self, desiredState): # angle in degrees, speed in mps
		# angle comes in from a swervestate so should be a rotation2d already
		correctedDesiredState = SwerveModuleState(desiredState.speed, 
			Rotation2d(desiredState.angle.radians() + self.chassisAngularOffset))

		optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(self.turnenc.getPosition()))
		
		#self.swerve_state.optimize(correctedDesiredState, Rotation2d(self.turnenc.getPosition()))
		# test self.swerve_state.optimize(correctedDesiredState, Rotation2d(0.0))

		self.drivepid.setReference(optimizedDesiredState.speed, CANSparkMax.ControlType.kVelocity)
		self.turnpid.setReference(optimizedDesiredState.angle.radians(), CANSparkMax.ControlType.kPosition)
#		self.drivepid.setReference(self.swerve_state.speed, CANSparkMax.ControlType.kVelocity)
#		self.turnpid.setReference(self.swerve_state.angle.radians(), CANSparkMax.ControlType.kPosition)

		self.desiredState = desiredState # This looks like an error in the java. Shouldn't this be optimizedDesiredState or correctedDesiredState?
		# Either way, it's never used anywhere.
		
	def resetEncoders(self):
		self.driveenc.setPosition(0)
	
		




