package frc.robot.commands


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem

// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class LaunchCoralL4CommandGroup(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem)
    : ParallelCommandGroup( LaunchCoralCommand(intakeSubsystem, Constants.Intake.OUTPUT_L4),
    (WaitCommand(0.0).andThen(GotoAngle(armSubsystem,Constants.Poses.L4Back.angle))))
