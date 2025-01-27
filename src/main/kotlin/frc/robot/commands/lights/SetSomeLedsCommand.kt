package frc.robot.commands.lights

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LightsSubsystem
import kotlin.math.min

open class SetSomeLedsCommand(private val lightsSubsystem: LightsSubsystem, private val startInclusive: Int, private val endInclusive: Int, private val color: Color, isExclusive: Boolean) : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        if (isExclusive) {addRequirements(lightsSubsystem)}
    }

    override fun initialize() {
        if (startInclusive > endInclusive) {
            DriverStation.reportError("End of range is less than start :(", false)
            return
        }
        val numLeds = lightsSubsystem.io.getLEDS()
        for (i in clamp(startInclusive, 0, numLeds)..clamp(endInclusive, 0, numLeds)) {
            lightsSubsystem.io.setLED(i, color)
        }
    }

    private fun checkIndex(index: Int): Boolean {
        if (index >= 0 && index < lightsSubsystem.io.getLEDS()) {
            return true
        } else {
            DriverStation.reportError("Attempted to set index out of range. $index is not within size ${lightsSubsystem.io.getLEDS()}", false)
        }
        return false
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true
    }

    override fun end(interrupted: Boolean) {}
}
