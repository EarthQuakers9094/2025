package frc.robot.commands.lights.lighteffects

import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LightsSubsystem

class RippleCommand(private val lightsSubsystem: LightsSubsystem, private val width: Int, private val color: Color, isExclusive: Boolean) : Command() {
    private var offset: Int = 0
    private val numLeds = lightsSubsystem.io.getLEDS()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        if (isExclusive) {addRequirements(lightsSubsystem)}
    }

    override fun initialize() {
        for (i in 0..(width)) {
            val i = i % numLeds
            lightsSubsystem.io.setLED(i, color)
        }
    }

    override fun execute() {
        val start = offset
        val end = (offset + width) % numLeds

        lightsSubsystem.io.setLED(start, Color(0x00,0x00,0x00))
        lightsSubsystem.io.setLED(end, color)

        offset += 1
        offset %= numLeds
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
