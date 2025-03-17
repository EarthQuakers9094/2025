package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem
import edu.wpi.first.wpilibj.Timer


class DevourAlgaeCommand(private val intakeSubsystem: IntakeSubsystem, private val autoEnd: Boolean) : Command() {
    val timer = Timer();

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.setVoltageAlgae(Constants.Intake.INTAKE_ALGAE)
        timer.restart()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return autoEnd && timer.hasElapsed(0.5)
    }

    override fun end(interrupted: Boolean) {
        //intakeSubsystem.setVoltageAlgae(0.0)
    }
}
