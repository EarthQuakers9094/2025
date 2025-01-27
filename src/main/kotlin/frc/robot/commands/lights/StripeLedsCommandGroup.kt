package frc.robot.commands.lights


import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.subsystems.LightsSubsystem

// TODO: Add your sequential commands in the super constructor call,
//       e.g. ParallelCommandGroup(OpenClawCommand(), MoveArmCommand())
class StripeLedsCommandGroup(lightsSubsystem: LightsSubsystem, stripes: Array<Stripe>, isExclusive: Boolean) : ParallelCommandGroup() {
    data class Stripe(val color: Color, val size: Int)

    init {
        if (isExclusive) {addRequirements(lightsSubsystem)}
        var currentLocation = 0
        for (stripe in stripes) {
            addCommands(SetSomeLedsCommand(lightsSubsystem, currentLocation, currentLocation + stripe.size, stripe.color,false))
            currentLocation += stripe.size + 1
        }
    }
}
