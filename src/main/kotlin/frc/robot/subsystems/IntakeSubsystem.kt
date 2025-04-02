package frc.robot.subsystems

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkFlexConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.config.ClosedLoopConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class IntakeSubsystem(private val intake: IntakeIO) : SubsystemBase() {
 fun setVoltage(power: Double) {
  intake.setVoltage(power)
 }
 fun setVoltageAlgae(power: Double) {
    intake.setVoltageAlgae(power)
 }

 override fun simulationPeriodic() {
  intake.periodic()
 }

fun getOutputCurrent(): Double {
   return intake.getCurrent()
 }

 override fun periodic() {
  intake.periodic()
 }

 fun hasPiece(): Boolean {
  return intake.hasPiece()
 }

 interface IntakeIO {
  fun periodic()
  fun setVoltage(power: Double)
  fun setVoltageAlgae(power: Double)
  fun hasPiece(): Boolean;
  fun getCurrent(): Double;
 }

class IntakeNeoIO(motor_id: Int, motor2_id: Int):IntakeIO {
 var motor: SparkMax;
 var motor2: SparkFlex;
 var encoder: RelativeEncoder;

 init {
  motor = SparkMax(motor_id, SparkLowLevel.MotorType.kBrushless)
  motor2 = SparkFlex(motor2_id, SparkLowLevel.MotorType.kBrushless)
  encoder = motor.encoder;
  motor2.configure(SparkFlexConfig()
   .idleMode(SparkBaseConfig.IdleMode.kCoast)
   .apply(
    ClosedLoopConfig()
     .p(0.01)
     .i(0.0)
     .d(0.0))
   .inverted(true),
   SparkBase.ResetMode.kNoResetSafeParameters,
   SparkBase.PersistMode.kPersistParameters)
 }

 override fun periodic() {
  SmartDashboard.putNumber("algae voltage output", motor2.appliedOutput)

 }
 override fun setVoltage(power: Double) {
  motor.set(power)
 }

 override fun setVoltageAlgae(power: Double) {
  motor2.setVoltage(power)
 }

 private val button = DigitalInput(9)

 override fun hasPiece():Boolean {
  return button.get()
 }

 override fun getCurrent(): Double { 
   return motor.outputCurrent
 }
}

 class IntakeSym() : IntakeIO {
  var set = 0.0;
  var setAlgae = 0.0;

  init {
  }

  override fun periodic() {}
  override fun setVoltage(power: Double) {
   set = power
  }

  override fun setVoltageAlgae(power: Double) {
   setAlgae = power
  }

  override fun hasPiece(): Boolean {
   return set >= 0.0
  }

  override fun getCurrent(): Double { 
   return 1000000.0
  }
 }
}