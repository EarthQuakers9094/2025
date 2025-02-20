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
import com.revrobotics.spark.config.SparkBaseConfig

class GrapplingSubsystem(private val grappling: GrapplingIO) : SubsystemBase() {
    private var setpoint = -90.0;

    private var lastTime = Timer.getFPGATimestamp();

    fun getAngle(): Rotation2d {
        return Rotation2d.fromDegrees(grappling.getAngle())
    }

    fun setSetpoint(loc: Double) {
        setpoint = loc;
        grappling.setSetpoint(loc)
    }

    fun atLocation(): Boolean {
        return (grappling.getAngle() - setpoint).absoluteValue <= Constants.Grappling.TOLERANCE
    }

    override fun simulationPeriodic() {
        grappling.periodic()
    }

    fun setoutput(output: Double) {
        grappling.setOutput(output)
    }
    override fun periodic() {
        SmartDashboard.putNumber("grappling angle",grappling.getAngle())

        grappling.periodic()
    }

    interface GrapplingIO {
        fun periodic()
        fun setSetpoint(loc: Double)
        fun getAngle(): Double;
        fun setOutput(output: Double);
    }

    class GrapplingNeoIO(motor_id: Int):GrapplingIO {
        var motor: SparkFlex;
        var encoder: RelativeEncoder;

        init {
            motor = SparkFlex(motor_id, SparkLowLevel.MotorType.kBrushless)
            encoder = motor.encoder;
            motor.configure(
                SparkMaxConfig()
                    .apply(
                        EncoderConfig().positionConversionFactor(Constants.Grappling.CONVERSION_FACTOR))
                    .apply(
                        ClosedLoopConfig().p(0.0).i(0.0).d(0.0))
                    .idleMode(SparkBaseConfig.IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

            encoder.setPosition(Constants.Grappling.START_POSITION)
        }

        override fun periodic() {
            // SmartDashboard.putNumber("abs grappling angle",absoluteEncoder.position)
        }

        override fun setSetpoint(loc: Double) {
            motor.closedLoopController.setReference(loc, SparkBase.ControlType.kPosition)
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

        // private val simulation = SingleJointedSim(DCMotor.getNEO(2), 16.0 * 40.0/12.0, 1.0,0.5, -5.0 * Math.PI/3.0, Math.PI/6.0,true, Constants.Grappling.START_POSITION * PI/180.0);

        override fun periodic() {
           // simulation.setInputVoltage(controller.calculate(simulation.angleRads * 180.0 / Math.PI,setpoint))
           // simulation.update(0.02)
        }


        override fun setSetpoint(loc: Double) {
            setpoint = loc
        }

        override fun getAngle(): Double {
            return 0.00
        }

        override fun setOutput(output: Double) {
            TODO("testing robot probaly never going to be implemented")
        }
    }
}