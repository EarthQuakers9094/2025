package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

class BackupCommand(private val swerveSubsystem: SwerveSubsystem) : Command() {
    private var timer = Timer();

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
        timer.restart()
        swerveSubsystem.drive(Translation2d(-1.0,0.0), 0.0, false);
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.1)
    }

    override fun end(interrupted: Boolean) {}
}
