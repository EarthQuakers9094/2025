import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants
import frc.robot.subsystems.SwerveSubsystem
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.sin
import kotlin.math.cos
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)

fun pathPlannerToTag(offset: Distance, tag: () -> Int, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean, back: Boolean): Command{
    return DeferredCommand({
        val constraints = PathConstraints(
            Constants.Drivebase.MAX_SPEED, 1.0,
            Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)
        );

        val location = fieldLayout.getTagPose(tag()).getOrNull()

        when (location) {
            null -> {
                Commands.none()}
            else -> {
                val rotation = location.rotation;

                val yaw = rotation.z;

                val fakeWidth = Constants.ROBOT_WIDTH - Inches.of(2.0) - Inches.of(8.0)

                val xoffset: Distance = offset.times(-sin(yaw)) + fakeWidth.times(cos(yaw)/2.0)
                val yoffset: Distance = offset.times(cos(yaw)) + fakeWidth.times(sin(yaw)/2.0)

                val position = Translation2d(
                    xoffset + location.measureX,
                    yoffset + location.measureY);


                //SmartDashboard.putNumber("yoffset (inches)", yoffset.`in`(Inches))
                val end = AutoBuilder.pathfindToPose(
                    Pose2d(
                        position,
                        Rotation2d(yaw + if (back) {0.0} else {Math.PI})),
                    constraints);

                if (autoEnd) {
                    end
                } else {
                    end.andThen(WaitCommand(360.0))
                }
            }
        }
    }, setOf(swerveSubsystem))
}

fun pathPlannerToReef(offset: Distance, tag: () -> Int, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean): Command {
    return pathPlannerToTag(offset, tag, swerveSubsystem, autoEnd, false)
}

fun pathPlannerToPickup(offset: Distance, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean): Command {
    return pathPlannerToTag(offset, 
        {
            val upperSide = swerveSubsystem.pose.measureY.`in`(Meters) >= 4.0;
            if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)) {
                if (upperSide) {
                    2
                } else {
                    1
                }
            } else {
                if (upperSide) {
                    13
                } else {
                    12
                }
            }
        }, swerveSubsystem, autoEnd, true)
}

//class PathPlannerToReef(private val offset: Distance, private val tag: () -> Int) : Command() {
//
//    init {
//        // each subsystem used by the command must be passed into the addRequirements() method
//    }
//
//    override fun initialize() {
//
//        // Create the constraints to use while pathfinding
//        val constraints = PathConstraints(
//            3.0, 4.0,
//            Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)
//        );
//
//        val location = fieldLayout.getTagPose(tag()).getOrNull()
//
//        when (location) {
//            null -> {}
//            else -> {
//                val rotation = location.rotation;
//
//                val yaw = rotation.z;
//
//                val xoffset: Distance = offset.times(-sin(yaw)) + Constants.ROBOT_WIDTH.times(cos(yaw)/2.0)
//                val yoffset: Distance = offset.times(cos(yaw)) + Constants.ROBOT_WIDTH.times(sin(yaw)/2.0)
//
//                AutoBuilder.pathfindToPose(
//                    Pose2d(
//                        Translation2d(
//                            xoffset + location.measureX,
//                            yoffset + location.measureY),
//                        Rotation2d(yaw + Math.PI)),
//                    constraints).andThen(WaitCommand(360.0)).schedule()
//            }
//        }
//    }
//
//    override fun execute() {}
//
//    override fun isFinished(): Boolean {
//        // TODO: Make this return true when this Command no longer needs to run execute()
//        return true
//    }
//
//    override fun end(interrupted: Boolean) {
//    }
//}
