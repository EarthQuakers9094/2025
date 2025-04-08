package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.SwerveSubsystem

class FlickAlgae(intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem, armSubsystem: ArmSubsystem, swerveSubsystem: SwerveSubsystem):
    ParallelRaceGroup(
        SequentialCommandGroup(
            ParallelCommandGroup(
                InstantCommand({// armSubsystem.fastSetSetpoint(Constants.Poses.FullExtend.angle.degrees)
                    elevatorSubsystem.setSetpoint(Constants.Poses.FullExtend.height)}),
                WaitUntilCommand({
                    elevatorSubsystem.getHeight() >= Constants.Elevator.MAX_HEIGHT - Meters.of(0.30)
                })
//        WaitUntilCommand({armSubsystem.getAngle().degrees >= -265.0})
//                WaitUntilCommand({armSubsystem.getAngle().degrees >= -270.0})
                    .andThen(LaunchAlgaeCommand(intakeSubsystem)))),
        RepeatCommand(InstantCommand({
            swerveSubsystem.drive(Translation2d(0.35, 0.0), 0.0, false)
        }, swerveSubsystem)
    ))


// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class ScoreAlgae(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem, elevatorSubsystem: ElevatorSubsystem, swerveSubsystem: SwerveSubsystem)
//    : ParallelCommandGroup( gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Barge2),
//                            WaitCommand(0.6).andThen(LaunchAlgaeCommand(intakeSubsystem)))
    :SequentialCommandGroup(
        gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Barge),
//        WaitCommand(0.5),
        FlickAlgae(intakeSubsystem, elevatorSubsystem, armSubsystem, swerveSubsystem)) // pure vertical
//    : SequentialCommandGroup(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.BargePrep))



//        gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.FullExtend),
//        ParallelCommandGroup(InstantCommand({armSubsystem.fastSetSetpoint(Constants.Poses.Barge.angle.degrees);
//                                             armSubsystem.setPose("barge")}, armSubsystem),
//                            Commands.waitUntil({armSubsystem.getAngle().degrees <= -190.0})
//                                .andThen(LaunchAlgaeCommand(intakeSubsystem)))



