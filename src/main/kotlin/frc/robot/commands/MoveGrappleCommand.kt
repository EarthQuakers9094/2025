package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.GrapplingSubsystem

class MoveGrappleCommand(private val grappleSubsystem: GrapplingSubsystem, private var power: Double) : Command() {

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(grappleSubsystem)
    }

    override fun initialize() {
        grappleSubsystem.setoutput(power)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        grappleSubsystem.setoutput(0.0)
    }
}
