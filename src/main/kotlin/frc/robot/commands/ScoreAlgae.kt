package frc.robot.commands


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ElevatorSubsystem


// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class ScoreAlgae(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem, elevatorSubsystem: ElevatorSubsystem)
    : ParallelCommandGroup( gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.FullExtend),
    (WaitCommand(0.6).andThen(ParallelCommandGroup( 
        WaitCommand(0.1).andThen(LaunchAlgaeCommand(intakeSubsystem)),
        InstantCommand({armSubsystem.fastSetSetpoint(Constants.Poses.Barge.angle.degrees)})
    ))))
