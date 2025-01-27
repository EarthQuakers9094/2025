package frc.robot.commands.lights

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LightsSubsystem

class SetLedCommand(private val lightsSubsystem: LightsSubsystem, private val index: Int, private val color: Color, isExclusive: Boolean) : Command() {


    init {

        // each subsystem used by the command must be passed into the addRequirements() method
        if (isExclusive) {addRequirements(lightsSubsystem)}
    }

    override fun initialize() {
        if (index >= 0 && index < lightsSubsystem.io.getLEDS()) {
            lightsSubsystem.io.setLED(index, color)
        } else {
            DriverStation.reportError("Attempted to set index out of range. $index is not within size ${lightsSubsystem.io.getLEDS()}", false)
        }
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true
    }

    override fun end(interrupted: Boolean) {}
}
