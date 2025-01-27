package frc.robot.subsystems

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import kotlin.math.absoluteValue


class ElevatorSubsystem(private var elevator: ElevatorIO) : SubsystemBase() {
    private var setpoint = 0.0;

    fun getHeight(): Double {
        return elevator.getHeight()
    }

    fun setSetpoint(loc: Double) {
        setpoint = loc;
        elevator.setSetpoint(loc)
    }

    fun atLocation(): Boolean {
        return (elevator.getHeight() - setpoint).absoluteValue <= Constants.Elevator.TOLERANCE
    }

    override fun simulationPeriodic() {
        elevator.periodic()
    }
    override fun periodic() {
        elevator.periodic()
    }

    interface ElevatorIO {
        fun periodic()
        fun setSetpoint(loc: Double)
        fun getHeight(): Double;
    }

    class ElevatorNeoIO(motor_id: Int):ElevatorIO {
        var motor: SparkMax;
        var encoder: RelativeEncoder;

        init {
            motor = SparkMax(motor_id, SparkLowLevel.MotorType.kBrushless)
            encoder = motor.encoder;
            motor.configure(
                SparkMaxConfig()
                    .apply(
                        EncoderConfig().positionConversionFactor(1.0))
                    .apply(
                        ClosedLoopConfig().p(0.0).i(0.0).d(0.0)),
                ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)
        }

        override fun periodic() {}

        override fun setSetpoint(loc: Double) {
            motor.closedLoopController.setReference(loc, SparkBase.ControlType.kPosition)
        }

        override fun getHeight(): Double {
            return encoder.position
        }
    }

    class ElevatorSimIO:ElevatorIO {
        private var controller = PIDController(0.0,0.0,0.0);

        private var setpoint = 0.0;

        private val simulation = ElevatorSim(1.0,1.0, DCMotor.getNEO(1).withReduction(1.0),0.0,1.0,true,0.0);

        override fun periodic() {
            simulation.setInputVoltage(controller.calculate(simulation.positionMeters,setpoint))
            simulation.update(0.02)
        }


        override fun setSetpoint(loc: Double) {
            setpoint = loc
        }

        override fun getHeight(): Double {
            return simulation.positionMeters
        }
    }
}