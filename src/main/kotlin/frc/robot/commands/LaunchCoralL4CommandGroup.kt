package frc.robot.commands


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem

// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class LaunchCoralL4CommandGroup(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem)
    : ParallelCommandGroup(LaunchCoralCommand(intakeSubsystem),
    MoveArmCommand(armSubsystem,Constants.Poses.L4.angle.degrees - 15.0))
