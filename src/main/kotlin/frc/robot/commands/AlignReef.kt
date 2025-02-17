package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.Units
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.Constants
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils
import CameraAlignInfo
import kotlin.collections.firstOrNull
import kotlin.math.abs
import kotlin.math.pow
import java.lang.Math

class AlignReef(private val swerveSubsystem: SwerveSubsystem, private val rightCamera: CameraAlignInfo, private val leftCamera: CameraAlignInfo, private val lateralOffset: Distance) : Command() {

    private val skewPID = PIDController(0.05, 0.0, 0.0)
    private val lateralPID = PIDController(2.5, 0.0, 0.0)
    private val forwardPID = PIDController(2.0, 0.0, 0.0)
    private val skewTolerance = 0.1
    private val lateralTolerance = 0.1
    private val distanceTolerance = 0.1
    private var skew = 0.0
    private var distance = 0.0
    private var yaw = 0.0
    private var targetId = 0
    private val reefTags = listOf(6,7,8,9,10,11)


    init {
        skewPID.enableContinuousInput(0.0, 360.0)
        
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
        targetId = 0 
    }

    override fun execute() {
        var offsetX = 0.0
        var offsetY = 0.0
        var offsetZ = 0.0
        var yaw = 0.0
        
        var numTargets = 0
        for (camera in listOf(rightCamera, leftCamera).withIndex()) {
            val index = camera.index
            val camera = camera.value
            
             val results = camera.camera.allUnreadResults
             val validResults = results.map { it.targets.filter { reefTags.contains(it.fiducialId) && (targetId == 0 || targetId == it.fiducialId) }.sortedBy { it.poseAmbiguity } }.filter {it.isNotEmpty()}
            val result = validResults.firstOrNull()?.firstOrNull()
            if (result != null){
                if (targetId == 0) { targetId = result.fiducialId}
                val rawTransform = result.getBestCameraToTarget()
                if (index == 0) {
                    SmartDashboard.putNumber("align target raw transform x", rawTransform.getX())
                    SmartDashboard.putNumber("align target transform y", rawTransform.getY())
                    SmartDashboard.putNumber("align target raw transform z", rawTransform.getZ())
                }
                numTargets += 1
                offsetX += rawTransform.getX()
                offsetY += rawTransform.getY() - camera.lateralOffset.`in`(edu.wpi.first.units.Units.Meters)
                offsetZ += rawTransform.getZ()
                yaw += ((rawTransform.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0
            }
        }
        
        if (numTargets == 0) {
            return
        }
        offsetX /= numTargets
        offsetY /= numTargets
        offsetZ /= numTargets
        yaw /= numTargets

        val rotation = if (abs(yaw) > 3.0) { -skewPID.calculate(yaw,180.0)} else {0.0}
        val lateral = -lateralPID.calculate(offsetY,lateralOffset.`in`(Units.Meters))
        val forward = if /*(yaw < skewTolerance && offsetY < lateralTolerance)*/(true) {
            -forwardPID.calculate(offsetX,0.0)
        } else {
            0.01
        }

        SmartDashboard.putNumber("align target offsetX", offsetX)
        SmartDashboard.putNumber("align target offsetY", offsetY)
        SmartDashboard.putNumber("align target offsetZ", offsetZ)
        SmartDashboard.putNumber("align target yaw", yaw)
        SmartDashboard.putNumber("align rotation", rotation)
        SmartDashboard.putNumber("align lateral", lateral)
        SmartDashboard.putNumber("align forward", forward)
        swerveSubsystem.drive(if ((swerveSubsystem.robotVelocity.vxMetersPerSecond.pow(2) + swerveSubsystem.robotVelocity.vyMetersPerSecond.pow(2)) < 1.0) { Translation2d(forward, lateral) } else {Translation2d(0.01, 0.01)},rotation,false)
        
        // SmartDashboard.putNumber("hello folks this is line 33", 1.0)
        // val results = camera.allUnreadResults
        // val validResults = results.map { it.targets.filter { reefTags.contains(it.fiducialId) }.sortedBy { it.poseAmbiguity } }
        // for (result in validResults) {
        //     //PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters(p0, p1, p2, p3), p1)
        //     SmartDashboard.putNumber("hello folks this is line 37", 1.0)
        //     //result.bestTarget.skew
        //     val target = result.get(0).bestCameraToTarget
        //     // skew = target.skew
        //     // yaw = target.yaw + lateralOffset
            
        //     distance = target.getX()
        //     val rotation = skewPID.calculate(skew,0.0)
        //     val lateral = lateralPID.calculate(yaw,0.0)
        //     val forward = forwardPID.calculate(distance,0.0)
        //     SmartDashboard.putNumber("align target skew", skew)
        //     SmartDashboard.putNumber("align target yaw", yaw)
        //     SmartDashboard.putNumber("align target distance", distance)
        //     SmartDashboard.putNumber("align rotation", rotation)
        //     SmartDashboard.putNumber("align lateral", lateral)
        //     SmartDashboard.putNumber("align forward", forward)
        //     swerveSubsystem.drive(Translation2d(forward, lateral),rotation,false)
        // }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false//((yaw) < lateralTolerance) && (skew < skewTolerance) && (distance < distanceTolerance)
    }

    override fun end(interrupted: Boolean) {}
}
