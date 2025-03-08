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
    L3,
    L2,
    L1,
    Other
}

fun launch_coral(intakeSubsystem: IntakeSubsystem, armSubsystem: ArmSubsystem): Command {
    return Commands.select(
        mapOf(LaunchType.L4 to LaunchCoralL4CommandGroup(intakeSubsystem, armSubsystem),
            LaunchType.L3 to LaunchCoralCommand(intakeSubsystem, Constants.Intake.OUTPUT_L3),
              LaunchType.L2 to LaunchCoralCommand(intakeSubsystem, Constants.Intake.OUTPUT_L2),
              LaunchType.L1 to LaunchCoralCommand(intakeSubsystem, Constants.Intake.OUTPUT_L1),
              LaunchType.Other to LaunchCoralCommand(intakeSubsystem, Constants.Intake.OUTPUT_L4)),
    {
        val pose = armSubsystem.getPose()

//            LaunchType.L4
//        } else {
//            LaunchType.Other
//        }
        if (pose == Constants.Poses.L1.pose) {
            LaunchType.L1
        } else if (pose == Constants.Poses.L2.pose) {
            LaunchType.L2
        } else if (pose == Constants.Poses.L3.pose) {
            LaunchType.L3
        } else if (pose == Constants.Poses.L4.pose) {
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
        //val pose = armSubsystem.getPose()
//        val speed = if (pose == Constants.Poses.L1.pose) {
//            Constants.Intake.OUTPUT_L1
//        } else if (pose == Constants.Poses.L2.pose) {
//            Constants.Intake.OUTPUT_L2
//        } else if (pose == Constants.Poses.L3.pose) {
//            Constants.Intake.OUTPUT_L3
//        } else if (pose == Constants.Poses.L4.pose) {
//            Constants.Intake.OUTPUT_L4
//        } else {
//            Constants.Intake.OUTPUT_L4
//        }
        intakeSubsystem.setVoltage(speed)
        timer.restart()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
         return timer.hasElapsed(0.4)
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.setVoltage(0.0)
    }
}
