package frc.robot.subsystems

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.Spark

class IntakeSubsystem(private val intake: IntakeIO) : SubsystemBase() {
 fun setVoltage(power: Double) {
  intake.setVoltage(power)
 }

 override fun simulationPeriodic() {
  intake.periodic()
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
 }

class IntakeNeoIO(motor_id: Int, motor2_id: Int):IntakeIO {
 var motor: SparkMax;
 var motor2: SparkFlex;
 var encoder: RelativeEncoder;

 init {
  motor = SparkMax(motor_id, SparkLowLevel.MotorType.kBrushless)
  motor2 = SparkFlex(motor2_id, SparkLowLevel.MotorType.kBrushless)
  encoder = motor.encoder;
 }

 override fun periodic() {}
 override fun setVoltage(power: Double) {
  motor.set(power)
 }

 override fun setVoltageAlgae(power: Double) {
  motor2.set(power)
 }

 private val button = DigitalInput(9)

 override fun hasPiece():Boolean {
  return button.get()
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
 }
}