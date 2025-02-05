package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem

class MoveElevatorCommand(private val elevatorSubsystem: ElevatorSubsystem, private var output: Double) : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(elevatorSubsystem)
    }

    override fun initialize() {
        elevatorSubsystem.setOutput(output)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        elevatorSubsystem.setOutput(0.0)
    }
}
