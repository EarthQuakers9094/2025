package frc.robot.commands.lights

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LightsSubsystem

class SetAllLedsCommand(private val lightsSubsystem: LightsSubsystem, color: Color) : SetSomeLedsCommand(lightsSubsystem, 0, lightsSubsystem.io.getLEDS(), color, true)
