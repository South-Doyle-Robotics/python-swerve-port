import commands2
import wpilib
from commands.drivetrain import Drive_Swerve, Drive_Swerve_Field_Steering

from wpilib import XboxController

from subsystems import swervedrivetrain

class robotcontainer:
    def __init__(self):
        print("In robotcontainer init")
        self.drivetrain = swervedrivetrain.drivetrain()

        self.controller = XboxController(0)
        print("Setting default command")

        #drivetrain.setDefaultCommand = self.drivetrain.drive(self.controller.Axis.kLeftY, 
        #                                        self.controller.Axis.kLeftX, 
        #                                        self.controller.Axis.kRightX)
        self.drivetrain.setDefaultCommand = self.drivetrain.drive(0, 0, 0)
        print("Default command set")


