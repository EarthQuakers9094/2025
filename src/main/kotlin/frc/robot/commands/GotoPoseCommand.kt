package frc.robot.commands

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Pose
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem

class GotoPoseCommand(private val armSubsystem: ArmSubsystem, private val elevatorSubsystem: ElevatorSubsystem, private val pose: Pose):
    Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        armSubsystem.setSetpoint(pose.angle.degrees)
        elevatorSubsystem.setSetpoint(pose.height.`in`(Meters))
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return armSubsystem.atLocation() && elevatorSubsystem.atLocation()
    }

    override fun end(interrupted: Boolean) {}
}
