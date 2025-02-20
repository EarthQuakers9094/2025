

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.LightsSubsystem

class NotificationLeds(val ledSubsystem: LightsSubsystem, val visionSubsystem: VisionSubsystem): Command() {
    init {
        addRequirements(ledSubsystem)
    }
    override fun execute() {
        //if (visionSubsystem.io.hasTarget(filter, cameras))
        //ledSubsystem.setAllLeds(Color)
    }
}