package frc.robot.commands

import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ElevatorSubsystem

class FlickAlgae(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem): SequentialCommandGroup(
    ParallelCommandGroup(
        InstantCommand({armSubsystem.fastSetSetpoint(Constants.Poses.FullExtend.angle.degrees)}),
        WaitUntilCommand({armSubsystem.getAngle().degrees >= -265.0})
//                WaitUntilCommand({armSubsystem.getAngle().degrees >= -270.0})

            .andThen(LaunchAlgaeCommand(intakeSubsystem))))

// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class ScoreAlgae(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem, elevatorSubsystem: ElevatorSubsystem)
//    : ParallelCommandGroup( gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Barge2),
//                            WaitCommand(0.6).andThen(LaunchAlgaeCommand(intakeSubsystem)))
    :SequentialCommandGroup(
        gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Barge),
        WaitCommand(0.5),
        FlickAlgae(intakeSubsystem, armSubsystem))



//        gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.FullExtend),
//        ParallelCommandGroup(InstantCommand({armSubsystem.fastSetSetpoint(Constants.Poses.Barge.angle.degrees);
//                                             armSubsystem.setPose("barge")}, armSubsystem),
//                            Commands.waitUntil({armSubsystem.getAngle().degrees <= -190.0})
//                                .andThen(LaunchAlgaeCommand(intakeSubsystem)))



