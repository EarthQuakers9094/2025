package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.ArmSubsystem
import frc.robot.commands.LaunchType
import frc.robot.commands.LaunchCoralCommand

enum class LaunchType {
    L4,
    L1,
    Other
}

fun launch_coral(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem): Command {
    return Commands.select(
        mapOf(LaunchType.L4 to LaunchCoralL4CommandGroup(intakeSubsystem, armSubsystem), 
              LaunchType.L1 to LaunchCoralCommand(intakeSubsystem, Constants.Intake.L1OUTPUT),
              LaunchType.Other to LaunchCoralCommand(intakeSubsystem, Constants.Intake.OUTPUT)),
    {
        if (armSubsystem.getPose() == Constants.Poses.L4.pose) {
            LaunchType.L4
        } else {
            LaunchType.Other
        }
    }
    )
}

class LaunchCoralCommand(private val intakeSubsystem: IntakeSubsystem, private val speed: Double) : Command() {
    private var timer = Timer();
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.setVoltage(speed)
        timer.restart()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
         return timer.hasElapsed(0.75)
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.setVoltage(0.0)
    }
}
