package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.utils.PIDController
import kotlin.math.cos
import kotlin.math.sin

//public fun moveRobotRelativeDistance(swerveSubsystem: SwerveSubsystem, distanceX: Distance, distanceY: Distance): Command {
//    val currentPose = swerveSubsystem.pose.translation
//    val currentRotation = swerveSubsystem.pose.rotation.radians
//    val offsetX = cos(currentRotation) * distanceX.`in`(Units.Meters)
//    val offsetY = sin(currentRotation) * distanceX.`in`(Units.Meters)
//    val targetPose = Translation2d(currentPose.x + offsetX, currentPose.y + offsetY)
//}
//
//class MoveRobotRelativeDistanceCommand(private val swerveSubsystem: SwerveSubsystem, private val distanceX: Distance, private val distanceY: Distance) : Command() {
//
//    private var targetPose: Translation2d? = null;
//    private var horizontalPID = PIDController()
//
//    init {
//        // each subsystem used by the command must be passed into the addRequirements() method
//        addRequirements(swerveSubsystem)
//    }
//
//    override fun initialize() {
//        val currentPose = swerveSubsystem.pose.translation
//        val currentRotation = swerveSubsystem.pose.rotation.radians
//        val offsetX = cos(currentRotation) * distanceX.`in`(Units.Meters)
//        val offsetY = sin(currentRotation) * distanceX.`in`(Units.Meters)
//        targetPose = Translation2d(currentPose.x + offsetX, currentPose.y + offsetY)
//
//    }
//
//    override fun execute() {
//
//    }
//
//    override fun isFinished(): Boolean {
//        // TODO: Make this return true when this Command no longer needs to run execute()
//        return false
//    }
//
//    override fun end(interrupted: Boolean) {}
//}
