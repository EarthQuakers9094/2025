package frc.robot.commands.lights

import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.lights.lighteffects.RainbowCommand
import frc.robot.subsystems.LightsSubsystem

object LightEffects {
    fun rainbow(lightsSubsystem: LightsSubsystem): Command {
        return RainbowCommand(lightsSubsystem, true)
    }

    fun lesbian(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xd6, 0x28, 0x00),
                Color(0xff, 0x9b, 0x56),
                Color(0xff, 0xff, 0xff),
                Color(0xd4, 0x62, 0xa6),
                Color(0xa4, 0x00, 0x62)
            ),
            true
        )
    }

    fun solidColor(lightsSubsystem: LightsSubsystem, color: Color): Command {
        return SetAllLedsCommand(lightsSubsystem, color)
    }

    fun india(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
            Color(0xff, 0x9a, 0x2f), Color(0xff, 0xff, 0xff), Color(0xa, 0x89, 1)
            ),
            true
        )
    }

    fun gay(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xe4, 0x03, 0x03),
                Color(0xff, 0x8c, 0x00),
                Color(0xff, 0xed, 0x00),
                Color(0x00, 0x80, 0x26),
                Color(0x00, 0x4d, 0xff),
                Color(0x75, 0x07, 0x87)
            ),
            true
        )
    }



    fun trans(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0x55, 0xcd, 0xfd),
                Color(0xf6, 0xaa, 0xb7),
                Color(0xff, 0xff, 0xff),
                Color(0xf6, 0xaa, 0xb7),
                Color(0x55, 0xcd, 0xfd)
            ), true
        )
    }

    fun bisexual(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xd7, 0x00, 0x71),
                Color(0xd7, 0x00, 0x71),
                Color(0x9c, 0x4e, 0x97),
                Color(0x00, 0x35, 0xaa),
                Color(0x00, 0x35, 0xaa),
            ),
            true
        )
    }

    fun enby(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xFC,0xF4,0x31),
                Color(0xFC,0xFC,0xFC),
                Color(0x9D,0x59,0xD2),
                Color(0x28,0x28,0x28),
            ),
            true
        )
    }

    fun agen(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0x00,0x00,0x00),
                Color(0xBA,0xBA,0xBA),
                Color(0xFF,0xFF,0xFF),
                Color(0xBA,0xF4,0x84),
                Color(0xFF,0xFF,0xFF),
                Color(0xBA,0xBA,0xBA),
                Color(0x00,0x00,0x00),
            ),
            true
        )
    }

    fun ace(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0x00,0x00,0x00),
                Color(0xA4,0xA4,0xA4),
                Color(0xFF,0xFF,0xFF),
                Color(0x81,0x00,0x81),
                    ),
            true
        )
    }
    fun aro(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0x3B,0xA7,0x40),
                Color(0xA8,0xD4,0x7A),
                Color(0xFF,0xFF,0xFF),
                Color(0xAB,0xAB,0xAB),
                Color(0x00,0x00,0x00),
            ),
            true
        )
    }

    fun pan(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xFF,0x1C,0x8D),
                Color(0xFF,0xD7,0x00),
                Color(0x1A,0xB3,0xFF),
            ),
            true
        )
    }

    fun queer(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xFF,0x1C,0x8D),
                Color(0xFF,0xD7,0x00),
                Color(0x1A,0xB3,0xFF)
            ),
            true
        )
    }

    fun bulgaria(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xFF,0xff,0xff),
                Color(0x00,0x96,0x6e),
                Color(0xd6,0x26,0x12)
            ),
            true
        )
    }

    fun german(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0x00,0x00,0x00),
                Color(0xDD,0x00,0x00),
                Color(0xFF,0xCC,0x00)
            ),
            true
        )
    }

    fun poland(lightsSubsystem: LightsSubsystem): Command {
        return StripeLedsEqualCommandGroup(
            lightsSubsystem,
            arrayOf(
                Color(0xDC,0x14,0x3c),
                Color(0xff,0xff,0xff),
            ),
            true
        )
    }
}

