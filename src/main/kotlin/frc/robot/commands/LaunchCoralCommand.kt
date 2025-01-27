package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class LaunchCoralCommand(private val intakeSubsystem: IntakeSubsystem) : Command() {
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.setVoltage(Constants.Intake.OUTPUT)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.setVoltage(0.0)
    }
}
