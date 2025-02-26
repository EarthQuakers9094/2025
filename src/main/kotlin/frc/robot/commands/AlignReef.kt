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
import frc.robot.utils.MovingAverage
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils
import CameraAlignInfo
import kotlin.collections.firstOrNull
import kotlin.math.abs
import kotlin.math.pow
import java.lang.Math
import VisionSubsystem
import edu.wpi.first.wpilibj2.command.Commands

enum class Side {
    Left,
    Right
}

fun alignReefSelect(swerveSubsystem: SwerveSubsystem, cameraSubsystem: VisionSubsystem, section: String): Command {
    SmartDashboard.putBoolean(section, false);

    return Commands.select(
        mapOf(
            Side.Left to AlignReef(swerveSubsystem, cameraSubsystem, Constants.Field.LEFT_OFFSET),
            Side.Right to AlignReef(swerveSubsystem, cameraSubsystem, Constants.Field.RIGHT_OFFSET),
        ),
        {
            if (SmartDashboard.getBoolean(section, true)) {
                Side.Left
            } else {
                Side.Right

            }
        }
    )
}

class AlignReef(private val swerveSubsystem: SwerveSubsystem, val cameraSubsystem: VisionSubsystem,private val lateralOffset: Distance) : Command() {

    private val skewPID = PIDController(0.07, 0.0, 0.0)
    private val lateralPID = PIDController(3.0, 0.0, 0.0)
    private val forwardPID = PIDController(2.5 * 1.05, 0.0, 0.0)
    private val skewTolerance = 10.0
    private val lateralTolerance = 2.5
    private val distanceTolerance = 0.1
    private var lateralAverage = MovingAverage(10)//0.0
    private var distanceAverage = MovingAverage(10)
    private var yawAverage = MovingAverage(10)
    private var targetId = 0
    private var no_targets = 0
    private var goBackwardsTimes = 0
    private val tagAngles = mapOf(7 to 0.0, 6 to 300.0, 8 to 60.0, 9 to 120.0, 10 to 180.0, 11 to 360.0 - 120.0)
    //private val reefTags = arrayOf(6,7,8,9,10,11)
    // aaron if you find this julia says you have to say banana five times


    init {
        skewPID.enableContinuousInput(0.0, 360.0)
        no_targets = 0
        goBackwardsTimes = 0
        
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
        targetId = 0
        SmartDashboard.putNumber("Align reef", 100.0) 
        //if (cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontRight", "ATFrontLeft")))
            
    }

    override fun execute() {
        var offsetX = 0.0
        var offsetY = 0.0
        var offsetZ = 0.0
        
        
        var numTargets = 0
        if (!cameraSubsystem.io.hasTarget(Constants.Vision.reefTags, arrayOf("ATFrontLeft", "ATFrontRight"))) {
            no_targets += 1
            return
        }
        for (camera in listOf("ATFrontLeft", "ATFrontRight")) {
            cameraSubsystem.io.getResults(camera)?.let { results -> 
                val validResults = results.map { 
                    it.targets.map { 
                        Pair(it, it.getBestCameraToTarget()) 
                    }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ 
                        (targetId == 0 || targetId == it.first.fiducialId) && (Constants.Vision.reefTags.contains(it.first.fiducialId))
                    }.sortedBy { abs((((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0) } 
                }.filter {it.isNotEmpty()}
                    val result = validResults.firstOrNull()?.firstOrNull()

                
                if (result != null){
                    if (targetId == 0) { targetId = result.first.fiducialId}
                    val rawTransform = result.second
                    
                    // if (index == 0) {
                    //     SmartDashboard.putNumber("align target raw transform x", rawTransform.getX())
                    //     SmartDashboard.putNumber("align target transform y", rawTransform.getY())
                    //     SmartDashboard.putNumber("align target raw transform z", rawTransform.getZ())
                    // }
                    numTargets += 1
                    offsetX += rawTransform.getX()
                    offsetY += rawTransform.getY() - cameraSubsystem.io.getLateralOffset(camera)!!.`in`(edu.wpi.first.units.Units.Meters)
                    offsetZ += rawTransform.getZ()
                    //yaw += -(((rawTransform.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0// -180 -> 0 && 180 -> 0, -170 -> //((rawTransform.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0
                }
            }
        }
        var yaw = tagAngles.get(targetId)!!
        SmartDashboard.putNumber("target id", targetId.toDouble() * 10)
        SmartDashboard.putNumber("num targets", numTargets.toDouble())
        
        
        // for (camera in listOf(rightCamera, leftCamera).withIndex()) {
        //     val index = camera.index
        //     val camera = camera.value
            
        //      val results = camera.camera.allUnreadResults
        //      val validResults = results.map { it.targets.map { Pair(it, it.getBestCameraToTarget()) }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ (reefTags.contains(it.first.fiducialId) && (targetId == 0 || targetId == it.first.fiducialId)) }.sortedBy { it.first.poseAmbiguity } }.filter {it.isNotEmpty()}
        //     val result = validResults.firstOrNull()?.firstOrNull()
        //     if (result != null){
        //         if (targetId == 0) { targetId = result.first.fiducialId}
        //         val rawTransform = result.second
                
        //         if (index == 0) {
        //             SmartDashboard.putNumber("align target raw transform x", rawTransform.getX())
        //             SmartDashboard.putNumber("align target transform y", rawTransform.getY())
        //             SmartDashboard.putNumber("align target raw transform z", rawTransform.getZ())
        //         }
        //         numTargets += 1
        //         offsetX += rawTransform.getX()
        //         offsetY += rawTransform.getY() - camera.lateralOffset.`in`(edu.wpi.first.units.Units.Meters)
        //         offsetZ += rawTransform.getZ()
        //         yaw += ((rawTransform.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0
        //     }
        // }
        
        if (numTargets == 0) {
            return
        }
        offsetX /= numTargets
        offsetY /= numTargets
        offsetZ /= numTargets
        //yaw /= numTargets

        // yaw = yawAverage.addValue(yaw)
        // offsetY = lateralAverage.addValue(offsetY)
        // offsetX = distanceAverage.addValue(offsetX)

        SmartDashboard.putNumber("yaw dist from zero", abs(180.0 - yaw))
        
        val rotation = if (abs(180.0 - yaw) > 3.0) { skewPID.calculate(swerveSubsystem.heading.degrees,yaw)} else {0.0}
        val lateral = -lateralPID.calculate(offsetY,lateralOffset.`in`(Units.Meters))
        val forward = if (abs(yaw - swerveSubsystem.heading.degrees) < skewTolerance && abs(lateralOffset.`in`(Units.Meters) - offsetY) < lateralTolerance) {
            -forwardPID.calculate(offsetX,0.0)
        } else {
            goBackwardsTimes += 1
            -0.3
        }

        
        SmartDashboard.putNumber("align target offsetX", offsetX)
        SmartDashboard.putNumber("align target offsetY", offsetY)
        SmartDashboard.putNumber("align target offsetZ", offsetZ)
        SmartDashboard.putNumber("align target yaw", yaw)
        SmartDashboard.putNumber("align rotation", rotation)
        SmartDashboard.putNumber("align lateral", lateral)
        SmartDashboard.putNumber("align forward", forward)
        SmartDashboard.putNumber("go backwards", goBackwardsTimes.toDouble())
        swerveSubsystem.drive(/*if ((swerveSubsystem.robotVelocity.vxMetersPerSecond.pow(2) + swerveSubsystem.robotVelocity.vyMetersPerSecond.pow(2)) < 3.0) { */
            Translation2d(forward, lateral) 
        /*} else {
            Translation2d(0.01, 0.01
        )}*/,rotation,false)
        
        //tuesday
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
        SmartDashboard.putNumber("no targets", no_targets.toDouble())
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false//!(cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontRight", "ATFrontLeft")))//((yaw) < lateralTolerance) && (skew < skewTolerance) && (distance < distanceTolerance)
    }

    override fun end(interrupted: Boolean) {
        //SmartDashboard.
        SmartDashboard.putNumber("Align reef", 0.0) 
    }
}
