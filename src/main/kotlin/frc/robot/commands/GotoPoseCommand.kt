package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Meter
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.Pose
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem

fun gotoPoseCommand(armSubsystem: ArmSubsystem, elevatorSubsystem: ElevatorSubsystem, pose: Pose): Command {
    if (elevatorSubsystem.getHeight() <= Constants.Elevator.COLLISION_HEIGHT_HIGH && pose.height >= Constants.Elevator.COLLISION_HEIGHT_LOW) {
        if (armSubsystem.getAngle().degrees < Constants.Arm.SAFE_ANGLE.degrees) {
            return SequentialCommandGroup(GotoSafeAngle(armSubsystem),GotoHeight(elevatorSubsystem, pose.height),GotoAngle(armSubsystem,pose.angle))
        }
        return ElevatorTrackingAngle(armSubsystem,elevatorSubsystem,pose)
    } else if (elevatorSubsystem.getHeight() >= Constants.Elevator.COLLISION_HEIGHT_LOW && pose.height <= Constants.Elevator.COLLISION_HEIGHT_HIGH) {
        if (armSubsystem.getAngle().degrees < Constants.Arm.SAFE_ANGLE.degrees) {
            return SequentialCommandGroup(GotoSafeAngle(armSubsystem),ArmTrackingHeight(armSubsystem,elevatorSubsystem,pose))
        }
        return ArmTrackingHeight(armSubsystem,elevatorSubsystem,pose)
    } else {
        return GotoPoseSimple(armSubsystem,elevatorSubsystem, pose);
    }
}

class GotoHeight(private val elevatorSubsystem: ElevatorSubsystem, private val height: Distance) : Command() {
    init {
        addRequirements(elevatorSubsystem)
    }
    override fun initialize() {
        elevatorSubsystem.setSetpoint(height)
    }

    override fun isFinished(): Boolean {
        return elevatorSubsystem.atLocation()
    }
}

open class GotoAngle(private val armSubsystem: ArmSubsystem, private val angle: Rotation2d) : Command() {
    init {
        addRequirements(armSubsystem)
    }
    override fun initialize() {
        armSubsystem.setSetpoint(angle.degrees)
    }

    override fun isFinished(): Boolean {
        return armSubsystem.atLocation()
    }
}

class GotoSafeAngle(private val armSubsystem: ArmSubsystem) : GotoAngle(armSubsystem, Constants.Arm.SAFE_ANGLE);

class ArmTrackingHeight(private val armSubsystem: ArmSubsystem, private val elevatorSubsystem: ElevatorSubsystem, private val pose: Pose) : Command() {
    private var startPosition: Rotation2d = Rotation2d.fromDegrees(0.0)
    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        elevatorSubsystem.setSetpoint(pose.height)
        startPosition = armSubsystem.getAngle()
    }

    override fun execute() {
        val percentTo = 1.0 - ((elevatorSubsystem.getHeight() - Constants.Elevator.COLLISION_HEIGHT_HIGH).`in`(Meter) /
                (Constants.Poses.L4.height - Constants.Elevator.COLLISION_HEIGHT_HIGH).`in`(Meter)).coerceIn(0.0,1.0);

        if (percentTo >= 1.0) {
            armSubsystem.setSetpoint(pose.angle.degrees)
        } else {
            armSubsystem.setSetpoint(startPosition.degrees + (pose.angle.degrees - startPosition.degrees) * percentTo)
        }
    }

    override fun isFinished(): Boolean {
        return armSubsystem.atLocation() && elevatorSubsystem.atLocation()
    }
}

class ElevatorTrackingAngle(private val armSubsystem: ArmSubsystem, private val elevatorSubsystem: ElevatorSubsystem, private val pose: Pose) : Command() {
    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        armSubsystem.setSetpoint(pose.angle.degrees)
    }

    override fun execute() {
        val percentTo = (armSubsystem.getAngle() - Constants.Poses.Pickup.angle).degrees /
                        (Constants.Arm.SAFE_ANGLE - Constants.Poses.Pickup.angle).degrees;

        if (percentTo >= 1.0) {
            elevatorSubsystem.setSetpoint(pose.height)
        } else {
            elevatorSubsystem.setSetpoint(Constants.Elevator.COLLISION_HEIGHT_LOW.times(percentTo))
        }
    }

    override fun isFinished(): Boolean {
        return armSubsystem.atLocation() && elevatorSubsystem.atLocation()
    }
}

class GotoPoseSimple(private val armSubsystem: ArmSubsystem, private val elevatorSubsystem: ElevatorSubsystem, private val pose: Pose):
    Command() {

    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        armSubsystem.setSetpoint(pose.angle.degrees)
        elevatorSubsystem.setSetpoint(pose.height)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return armSubsystem.atLocation() && elevatorSubsystem.atLocation()
    }

    override fun end(interrupted: Boolean) {}
}
