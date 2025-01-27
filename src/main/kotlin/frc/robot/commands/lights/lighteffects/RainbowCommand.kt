package frc.robot.commands.lights.lighteffects

import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LightsSubsystem

class RainbowCommand(private val lightsSubsystem: LightsSubsystem, isExclusive: Boolean) : Command() {
    private var startingHue = 0
    private val numLeds = lightsSubsystem.io.getLEDS()
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        if (isExclusive) {addRequirements(lightsSubsystem)}
    }

    override fun initialize() {

    }

    override fun execute() {
        for (i in 0..numLeds) {
            lightsSubsystem.io.setLED(i, Color.fromHSV(startingHue + i, 255, 255))
        }
        startingHue += 1
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {

    }
}
