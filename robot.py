#! python3

import wpilib
import commands2
import math
import constants

import robotcontainer
#import networktables
#networktables.NetworkTables.initialize()

import subsystems.swervedrivetrain
from commands import drivetrain

from wpilib import DriverStation

class Robot(commands2.TimedCommandRobot):
	def robotInit(self):
		DriverStation.silenceJoystickConnectionWarning(True)
            
		print("robot init before container")
		self.robot = robotcontainer.robotcontainer()
		print("robot init after container")
		
		#self.scheduler = commands2.CommandScheduler.getInstance()
		print("robot init scheduler")

	def robotPeriodic(self):
		
		commands2.CommandScheduler.getInstance().run()
		#self.scheduler.run()
		

	def teleopPeriodic(self):
		pass

	def teleopInit(self):
		pass 

	def disabledInit(self):
		commands2.CommandScheduler.getInstance().cancelAll()

		

	def disabledPeriodic(self):
		pass

	def autonomousInit(self):
		pass

	def autonomousPeriodic(self):
		pass


if __name__ == "__main__":
	wpilib.run(Robot)



