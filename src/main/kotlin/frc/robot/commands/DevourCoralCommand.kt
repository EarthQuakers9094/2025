package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class DevourCoralCommand(private val intakeSubsystem: IntakeSubsystem) : Command() {
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.setVoltage(Constants.Intake.INTAKE)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return intakeSubsystem.getOutputCurrent() > Constants.Intake.STOP_CURRENT
        // TODO: Make this return true when this Command no longer needs to run execute()
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.setVoltage(0.0)
    }
}
