package frc.robot.commands

import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ElevatorSubsystem


// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class ScoreAlgae(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem, elevatorSubsystem: ElevatorSubsystem)
//    : ParallelCommandGroup( gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Barge),
//                            WaitCommand(0.8).andThen(LaunchAlgaeCommand(intakeSubsystem)))
    :SequentialCommandGroup(
        gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.FullExtend),
        ParallelCommandGroup(InstantCommand({armSubsystem.fastSetSetpoint(Constants.Poses.Barge.angle.degrees);
                                             armSubsystem.setPose("barge")}, armSubsystem),
                            Commands.waitUntil({armSubsystem.getAngle().degrees <= -190.0})
                                .andThen(LaunchAlgaeCommand(intakeSubsystem)))

        )

