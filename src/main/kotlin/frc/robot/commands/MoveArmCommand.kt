package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ArmSubsystem

class MoveArmCommand(private val armSubsystem: ArmSubsystem, private var output: Double) : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem)
    }

    override fun initialize() {
        armSubsystem.setSetpoint(armSubsystem.getSetpoint()+output )
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        //armSubsystem.setoutput(0.0)
    }
}
