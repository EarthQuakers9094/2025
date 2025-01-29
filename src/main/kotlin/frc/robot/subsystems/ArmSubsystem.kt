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
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants
import kotlin.math.absoluteValue

class ArmSubsystem(private val arm: ArmIO) : SubsystemBase() {
    private var setpoint = 0.0;

    fun getAngle(): Rotation2d {
        return Rotation2d.fromDegrees(arm.getAngle())
    }

    fun setSetpoint(loc: Double) {
        setpoint = loc;
        arm.setSetpoint(loc)
    }

    fun atLocation(): Boolean {
        return (arm.getAngle() - setpoint).absoluteValue <= Constants.Arm.TOLERANCE
    }

    override fun simulationPeriodic() {
        arm.periodic()
    }
    override fun periodic() {
        arm.periodic()
    }

    interface ArmIO {
        fun periodic()
        fun setSetpoint(loc: Double)
        fun getAngle(): Double;
    }

    class ArmNeoIO(motor_id: Int):ArmIO {
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

        override fun getAngle(): Double {
            return encoder.position
        }
    }

    class ArmSimIO:ArmIO {
        private var controller = PIDController(0.0,0.0,0.0);

        private var setpoint = 0.0;

        private val simulation = SingleJointedArmSim(DCMotor.getNEO(1), 1.0, 1.0,0.5, -5.0 * Math.PI/3.0, Math.PI/6.0,true, -Math.PI/2.0);

        override fun periodic() {
            simulation.setInputVoltage(controller.calculate(simulation.angleRads * 180.0 / Math.PI,setpoint))
            simulation.update(0.02)
        }


        override fun setSetpoint(loc: Double) {
            setpoint = loc
        }

        override fun getAngle(): Double {
            return simulation.angleRads * 180.0 / Math.PI
        }
    }
}