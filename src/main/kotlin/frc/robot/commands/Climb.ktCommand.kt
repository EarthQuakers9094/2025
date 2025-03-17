package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.GrapplingSubsystem

class RetractClimber(private val grapplingSubsystem: GrapplingSubsystem, private val inPosition: Double) : Command() {
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(grapplingSubsystem)
    }

    override fun initialize() {
        grapplingSubsystem.setoutput(-1.0)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return grapplingSubsystem.getAngle() < inPosition // 10.0
    }

    override fun end(interrupted: Boolean) {
        grapplingSubsystem.setoutput(0.0)
    }
}
