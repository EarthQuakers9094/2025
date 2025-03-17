package frc.robot.subsystems

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.config.AbsoluteEncoderConfig
import com.revrobotics.spark.SparkFlex
import com.revrobotics.AbsoluteEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants
import kotlin.math.PI
import kotlin.math.absoluteValue
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode

class GrapplingSubsystem(private val arm: GrapplingIO) : SubsystemBase() {
    private var setpoint = 0.0;

    fun getAngle(): Double {
        return arm.getAngle()
    }

    fun getSetpoint(): Double {
        return setpoint

    }

    override fun simulationPeriodic() {
        arm.periodic()
    }

    fun setoutput(output: Double) {
        arm.setOutput(output)
    }
    override fun periodic() {
        arm.periodic()
    }

    interface GrapplingIO {
        fun periodic();
        fun getAngle(): Double;
        fun setOutput(output: Double);
    }

    class GrapplingNeoIO(motor_id: Int):GrapplingIO {
        var motor: SparkMax;
        var encoder: RelativeEncoder;

        init {
            motor = SparkMax(motor_id, SparkLowLevel.MotorType.kBrushless)
            encoder = motor.encoder;
            motor.configure(
                SparkMaxConfig()
                    .idleMode(IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

            encoder.setPosition(0.0)

            // encoder.setPosition(Constants.Arm.START_POSITION)
        }

        override fun periodic() {
             SmartDashboard.putNumber("climber position",encoder.position)
        }


        override fun getAngle(): Double {
            return encoder.position
        }

        override fun setOutput(output: Double) {
            motor.set(output)
        }
    }

    class GrapplingSimIO:GrapplingIO {
        private var controller = PIDController(0.0,0.0,0.0);

        private var setpoint = 0.0;

        private val simulation = SingleJointedArmSim(DCMotor.getNEO(2), 16.0 * 40.0/12.0, 1.0,0.5, -5.0 * Math.PI/3.0, Math.PI/6.0,true, Constants.Arm.START_POSITION * PI/180.0);

        override fun periodic() {
            simulation.setInputVoltage(controller.calculate(simulation.angleRads * 180.0 / Math.PI,setpoint))
            simulation.update(0.02)
        }

        override fun getAngle(): Double {
            return simulation.angleRads * 180.0 / Math.PI
        }

        override fun setOutput(output: Double) {
            TODO("testing robot probaly never going to be implemented")
        }
    }
}