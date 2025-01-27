package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LightsSubsystem(val io: LightsIO) : SubsystemBase() {
    interface LightsIO {
        fun periodic();
        fun setLED(index: Int, color: Color);
        fun getLEDS(): Int;
    }

    class LightsReal(private val leds: Int, port: Int): LightsIO {
        private val ledStrip =
            AddressableLED(port).let {
                it.setLength(leds)
                it
            }

        private var ledbuffer = AddressableLEDBuffer(leds);

        override fun setLED(index: Int, color: Color) {
            ledbuffer.setLED(index, color)
        }

        override fun periodic() {
            ledStrip.setData(ledbuffer)
        }

        override fun getLEDS(): Int {
            return leds
        }

    }

}