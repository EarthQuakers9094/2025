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

class ArmSubsystem(private val arm: ArmIO) : SubsystemBase() {
    private var setpoint = -90.0;

    private var profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(
            Constants.Arm.MAX_VEL,
            Constants.Arm.MAX_ACELERATION))

    private var lastTime = Timer.getFPGATimestamp();

    private var current_setpoint = TrapezoidProfile.State(Constants.Arm.START_POSITION,0.0);

    fun getAngle(): Rotation2d {
        return Rotation2d.fromDegrees(arm.getAngle())
    }

    fun setSetpoint(loc: Double) {
        setpoint = loc;
        // arm.setSetpoint(loc)
    }

    fun atLocation(): Boolean {
        return (arm.getAngle() - setpoint).absoluteValue <= Constants.Arm.TOLERANCE
    }

    override fun simulationPeriodic() {
        arm.periodic()
    }

    fun setoutput(output: Double) {
        arm.setOutput(output)
    }
    override fun periodic() {
        val ntime = Timer.getFPGATimestamp();

        current_setpoint = profile.calculate(ntime - lastTime, current_setpoint, TrapezoidProfile.State(setpoint, 0.0))
        lastTime = ntime;


        arm.setSetpoint(current_setpoint.position)
        SmartDashboard.putNumber("arm angle",arm.getAngle())
        SmartDashboard.putNumber("arm setpoint",current_setpoint.position)


        arm.periodic()
    }

    interface ArmIO {
        fun periodic()
        fun setSetpoint(loc: Double)
        fun getAngle(): Double;
        fun setOutput(output: Double);
    }

    class ArmNeoIO(motor_id: Int):ArmIO {
        var motor: SparkFlex;
        var encoder: RelativeEncoder;
        var absoluteEncoder: AbsoluteEncoder;

        init {
            motor = SparkFlex(motor_id, SparkLowLevel.MotorType.kBrushless)
            encoder = motor.encoder;
            motor.configure(
                SparkMaxConfig()
                    .apply(
                        EncoderConfig().positionConversionFactor(Constants.Arm.CONVERSION_FACTOR))
                    .apply(
                        ClosedLoopConfig().p(0.018).i(0.0).d(0.0))
                    .apply(
                        AbsoluteEncoderConfig()
                            .positionConversionFactor(Constants.Arm.ABSOLUTE_ENCODER_CONVERSION_FACTOR)
                            .zeroOffset(Constants.Arm.ABSOLUTE_ENCODER_OFFSET)
                            .zeroCentered(true)
                    ),
                ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

            absoluteEncoder = motor.absoluteEncoder;

            SmartDashboard.putNumber("position of arm set at start", absoluteEncoder.position - 90.0)

            encoder.setPosition(absoluteEncoder.position - 90.0) // abs encoder zeroed around thing starting position

            // encoder.setPosition(Constants.Arm.START_POSITION)
        }

        override fun periodic() {
            // SmartDashboard.putNumber("abs arm angle",absoluteEncoder.position)
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

    class ArmSimIO:ArmIO {
        private var controller = PIDController(0.0,0.0,0.0);

        private var setpoint = 0.0;

        private val simulation = SingleJointedArmSim(DCMotor.getNEO(2), 16.0 * 40.0/12.0, 1.0,0.5, -5.0 * Math.PI/3.0, Math.PI/6.0,true, Constants.Arm.START_POSITION * PI/180.0);

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

        override fun setOutput(output: Double) {
            TODO("testing robot probaly never going to be implemented")
        }
    }
}