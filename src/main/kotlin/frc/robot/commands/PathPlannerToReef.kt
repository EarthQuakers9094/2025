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
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.utils.PIDController
import kotlin.math.pow
import kotlin.math.sqrt


val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)

class PidToPosition(private val pose: Pose2d, private val swerveSubsystem: SwerveSubsystem): Command() {
    val xpid = PIDController(Constants.Drivebase.TRANSLATION_PID_TELEOP)
    val ypid = PIDController(Constants.Drivebase.TRANSLATION_PID_TELEOP)
    val rpid = PIDController(Constants.Drivebase.ROTATION_PID_TELEOP)

    init {
        addRequirements(swerveSubsystem)

        rpid.enableContinuousInput(-180.0, 180.0)
    }

    override fun execute() {
        val loc = swerveSubsystem.pose;
        val xi = xpid.calculate(loc.x, pose.x)

        SmartDashboard.putNumber("drivebase current x", loc.x);
        SmartDashboard.putNumber("drivebase desired x", pose.x);


        val yi = ypid.calculate(loc.y, pose.y)

        SmartDashboard.putNumber("drivebase current y", loc.y)
        SmartDashboard.putNumber("drivebase desired y", pose.y)

        val cr = if (loc.rotation.degrees < 0.0) {loc.rotation.degrees + 360.0} else {loc.rotation.degrees}
        val dr = if (pose.rotation.degrees < 0.0) {pose.rotation.degrees + 360.0} else {pose.rotation.degrees}

//        val ri = rpid.calculate(cr, dr)

        SmartDashboard.putNumber("drivebase current r", cr)
        SmartDashboard.putNumber("drivebase desired r", dr)

        swerveSubsystem.drive(Translation2d(xi,yi), 0.0, true)
    }
}

fun pathPlannerToTag(offset: Distance, tag: () -> Int, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean, back: Boolean): Command{
    return DeferredCommand({
        val constraints = PathConstraints(
            Constants.Drivebase.MAX_SPEED, 1.0,
            Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)
        );
        //val sideTags = DriverStation.getAlliance() == Alliance.

        val location = fieldLayout.getTagPose(tag()).getOrNull()

        when (location) {
            null -> {
                Commands.none()}
            else -> {
                val rotation = location.rotation;

                val yaw = rotation.z;

                val fakeWidth = Constants.ROBOT_WIDTH - Inches.of(3.0/* + 0.5*/) - Inches.of(8.0)

                val xoffset: Distance = offset.times(-sin(yaw)) + fakeWidth.times(cos(yaw)/2.0)
                val yoffset: Distance = offset.times(cos(yaw)) + fakeWidth.times(sin(yaw)/2.0)

                val position = Translation2d(
                    xoffset + location.measureX,
                    yoffset + location.measureY);

                val endPose = Pose2d(
                    position,
                    Rotation2d(yaw + if (back) {0.0} else {Math.PI}))


                //SmartDashboard.putNumber("yoffset (inches)", yoffset.`in`(Inches))
                val end = AutoBuilder.pathfindToPose(endPose,
                    constraints);

                if (autoEnd) {
                    end
                } else {
                    end.andThen(PidToPosition(endPose, swerveSubsystem))
                }
            }
        }
    }, setOf(swerveSubsystem))
}

fun pathPlannerToReef(offset: Distance, tag: () -> Int, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean): Command {
    return pathPlannerToTag(offset, tag, swerveSubsystem, autoEnd, false)
}

fun closestTag(swerveSubsystem: SwerveSubsystem): Int {
    val sideTags = if (DriverStation.getAlliance().getOrNull() == Alliance.Red) {
        kotlin.collections.listOf(6,7,8,9,10,11)
    } else {
        kotlin.collections.listOf(17,18,19,20,21,22)
    }

    return sideTags.map {
        val translation = (fieldLayout.getTagPose(it).getOrNull()!!.toPose2d() - swerveSubsystem.pose).translation
        return@map it to sqrt(translation.x.pow(2) + translation.y.pow(2))
    }.minByOrNull { it.second }!!.first
}

fun pathPlannerToReef(offset: Distance, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean): Command {
    return DeferredCommand({
        val tag = closestTag(swerveSubsystem)
        pathPlannerToTag(offset, {tag}, swerveSubsystem, autoEnd, false)
    },setOf(swerveSubsystem))

}

fun getClosestPickupTag(swerveSubsystem: SwerveSubsystem): Int {
    val upperSide = swerveSubsystem.pose.measureY.`in`(Meters) >= 4.0;
    return if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)) {
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
}

fun pathPlannerToPickup(offset: Distance, swerveSubsystem: SwerveSubsystem, autoEnd: Boolean): Command {
    return pathPlannerToTag(offset, 
        {
            getClosestPickupTag(swerveSubsystem)
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
