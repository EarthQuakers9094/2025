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



class AlignPickup(private val swerveSubsystem: SwerveSubsystem, val cameraSubsystem: VisionSubsystem,private val lateralOffset: Distance) : Command() {

    private val skewPID = PIDController(0.07, 0.0, 0.0)
    private val lateralPID = PIDController(2.5, 0.0, 0.0)
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
    private val tagAngles = mapOf(2 to 234.0)
    private val tags = arrayOf(2)

    init {
        skewPID.enableContinuousInput(0.0, 360.0)
        no_targets = 0
        goBackwardsTimes = 0
        
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
        targetId = 0

        //val angle = swerveSubsystem.heading.degrees + 360;
        // val dx = Math.cos(angle.radians);
        // val dy = Math.sin(angle.radians);

        for (camera in listOf("ATBack")) {
            cameraSubsystem.io.getResults(camera)?.let { results -> 
                val validResults = results.map { 
                    it.targets.map { 
                        Pair(it, it.getBestCameraToTarget())
                    }.filter {
                        (targetId == 0 || targetId == it.first.fiducialId) && (tags.contains(it.first.fiducialId))
                    }
                    //.sortedBy { abs((tagAngles.get(it.first.fiducialId)!!) + 360 - angle) } // Math.abs(it.second.getX() * dx + it.second.getY() * dy) }
                    .sortedBy { (it.second.getX().pow(2) + (it.second.getY() - cameraSubsystem.io.getLateralOffset(camera)!!.`in`(edu.wpi.first.units.Units.Meters)).pow(2))/*abs((((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0)*/ } 
                }.filter {it.isNotEmpty()}

                val result = validResults.firstOrNull()?.firstOrNull()
                
                if (result != null){
                    targetId = result.first.fiducialId
                }
            }
        }
        SmartDashboard.putNumber("Align reef", 100.0) 
        //if (cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontRight", "ATFrontLeft")))
            
    }

    override fun execute() {
        var offsetX = 0.0
        var offsetY = 0.0
        var offsetZ = 0.0
        
        
        var numTargets = 0
        if (!cameraSubsystem.io.hasTarget(tags, arrayOf("ATBack"))) {
            no_targets += 1
            return
        }

        for (camera in listOf("ATBack")) {
            cameraSubsystem.io.getResults(camera)?.let { results -> 
                val validResults = results.map { 
                    it.targets.map {
                        Pair(it, it.getBestCameraToTarget())
                    }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ 
                        (targetId == 0 || targetId == it.first.fiducialId) && (tags.contains(it.first.fiducialId))
                    }
                    // removed sorting because only two results should exist right?
                    // .sortedBy { (it.second.getX().pow(2) + (it.second.getY() - cameraSubsystem.io.getLateralOffset(camera)!!.`in`(edu.wpi.first.units.Units.Meters)).pow(2)).pow(0.5)/*abs((((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0)*/ } 
                }.filter {it.isNotEmpty()}
                    val result = validResults.firstOrNull()?.firstOrNull()

                
                if (result != null){
                    // if (targetId == 0) { targetId = result.first.fiducialId}
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
        
        
        
        
        if (numTargets == 0) {
            SmartDashboard.putNumber("no targets", no_targets.toDouble())
            return
        }
        offsetX /= numTargets
        offsetY /= numTargets
        offsetZ /= numTargets


        SmartDashboard.putNumber("yaw dist from zero", abs(180.0 - yaw))
        
        

        val rotation = skewPID.calculate(swerveSubsystem.heading.degrees,yaw)

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
        
        
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return targetId == 0 // don't do anything if you don't have a targetId
        // return false//!(cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontRight", "ATFrontLeft")))//((yaw) < lateralTolerance) && (skew < skewTolerance) && (distance < distanceTolerance)
    }

    override fun end(interrupted: Boolean) {
        //SmartDashboard.
        SmartDashboard.putNumber("Align reef", 0.0) 
        targetId = 0
    }
}
