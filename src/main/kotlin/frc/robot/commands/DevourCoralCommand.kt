package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class DevourCoralCommand(private val intakeSubsystem: IntakeSubsystem, private val autostop: Boolean) : Command() {
    var timer = Timer();


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        timer.restart()
        intakeSubsystem.setVoltage(Constants.Intake.INTAKE)
    }

    override fun execute() {
        if (intakeSubsystem.getOutputCurrent() < Constants.Intake.STOP_CURRENT) {
            timer.restart()
        }
        SmartDashboard.putNumber("grabber current", intakeSubsystem.getOutputCurrent())
    }

    override fun isFinished(): Boolean {
        // return false;

        return timer.hasElapsed(0.2) && autostop
        // TODO: Make this return true when this Command no longer needs to run execute()
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.setVoltage(0.0)
    }
}
