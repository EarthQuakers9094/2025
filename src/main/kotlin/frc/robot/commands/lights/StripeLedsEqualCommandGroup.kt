package frc.robot.commands.lights


import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.subsystems.LightsSubsystem
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.roundToInt

// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class StripeLedsEqualCommandGroup(lightsSubsystem: LightsSubsystem, stripes: Array<Color>, isExclusive: Boolean) : ParallelCommandGroup() {


    init {
        if (isExclusive) {addRequirements(lightsSubsystem)}
        val numLeds = lightsSubsystem.io.getLEDS()
        val size = ceil(numLeds.toDouble() / stripes.size.toDouble()).roundToInt()
        var currentLocation = 0
        for (stripe in stripes) {
            addCommands(SetSomeLedsCommand(lightsSubsystem, currentLocation, currentLocation + size, stripe,false))
            currentLocation += size + 1
        }
    }
}
