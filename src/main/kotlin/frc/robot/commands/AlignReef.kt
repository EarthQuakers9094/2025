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
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.utils.PIDController

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

class AlignReef(private val swerveSubsystem: SwerveSubsystem, val cameraSubsystem: VisionSubsystem,private val lateralOffset: Distance, /*private val pov: () -> Int*/) : Command() {

    private val skewPID = PIDController(0.07, 0.0, 0.0)
    private val lateralPID = PIDController(Constants.Drivebase.TRANSLATION_PID_TELEOP)
    private val forwardPID = PIDController(Constants.Drivebase.TRANSLATION_PID_TELEOP)
    private val skewTolerance = 10.0
    private val lateralTolerance = 2.5
    private val distanceTolerance = 0.1
    private var lateralAverage = MovingAverage(10)//0.0
    private var distanceAverage = MovingAverage(10)
    private var yawAverage = MovingAverage(10)
    private var targetId = -1
    private var no_targets = 0
    private var goBackwardsTimes = 0
    private val tagAngles = mapOf(7 to 0.0, 6 to 300.0, 8 to 60.0, 9 to 120.0, 10 to 180.0, 11 to 360.0 - 120.0)
    private val reefTags = arrayOf(6,7,8,9,10,11)

    init {
        skewPID.enableContinuousInput(0.0, 360.0)
        no_targets = 0
        goBackwardsTimes = 0
        
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem, cameraSubsystem)
    }
    
    // private fun getSelected(): Int {
    //     return when (pov()) {
    //         0 -> 10     
    //         45 -> 9
    //         135 -> 8
    //         180 -> 7
    //         225 -> 6
    //         315 -> 11
    //         else -> -1
    //     }
    // }

    override fun initialize() {
        //this.targetId = 0
        var id = 0

        //val angle = swerveSubsystem.heading.degrees + 360;
        // val dx = Math.cos(angle.radians);
        // val dy = Math.sin(angle.radians);

        for (camera in listOf("ATFrontLeft", "ATFrontRight")) {
            val results = cameraSubsystem.io.getResults(camera)!!
            //if (results)
            
            val validResults = results.map { 
                it.targets.map { 
                    Pair(it, it.getBestCameraToTarget())
                }.filter {
                    //it.first.fiducialId == getSelected()
                    (reefTags.contains(it.first.fiducialId))
                }
                //.sortedBy { abs((tagAngles.get(it.first.fiducialId)!!) + 360 - angle) } // Math.abs(it.second.getX() * dx + it.second.getY() * dy) }
                //.sortedBy { (it.second.getX().pow(2) + (it.second.getY() - cameraSubsystem.io.getLateralOffset(camera)!!.`in`(edu.wpi.first.units.Units.Meters)).pow(2))/*abs((((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0)*/ } 
            }.filter {it.isNotEmpty()}.flatten().sortedBy { (it.second.getX().pow(2) + (it.second.getY() - cameraSubsystem.io.getLateralOffset(camera)!!.`in`(edu.wpi.first.units.Units.Meters)).pow(2))/*abs((((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0)*/ }

            val result = validResults.firstOrNull()
            
            if (result != null){
                id = result.first.fiducialId
                DriverStation.reportWarning("did find a target, ${id}", false);
                
                this.targetId = id
                if (id == 0) {
                    TODO("robot killing itself :)")
                }
                if (this.targetId != id) {
                    TODO("im going to shoot dean kamen out of a human cannon unconsentually")
                }
                DriverStation.reportWarning("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ${this.targetId}", false);
            } /*else {
                TODO("robot killing itself wacky fun haha silly isnt that hilarious omg this is so funny im having a blast blast haha get it because human cannon:)")
            }*/
        }
        if (this.targetId == -1) {
            DriverStation.reportError("GOT NO TARGETS", false)
            //TODO("robot killing itself :) 2ngcgjcg cjgfhcnjkhgczslfkdg lrashflkasdfkulh dasljkfhalskdjhf klasdhlfkjasdh kljfhlasdkjfhlkadsh fkdshflkjdshflkjasdhkfjl hdskljfhsafhlkjsdhlfsadfhadsljkfhldjshf")
        }
        DriverStation.reportWarning("????????????????????????????? ${this.targetId}", false);
        SmartDashboard.putNumber("Align reef", 100.0) 
        //if (cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontRight", "ATFrontLeft")))
            
    }

    override fun execute() {
        var offsetX = 0.0
        var offsetY = 0.0
        var offsetZ = 0.0
        
        DriverStation.reportWarning("targetid &&&&&&&&&&&& ${this.targetId}", false);
        var numTargets = 0
        if (!cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontLeft", "ATFrontRight"))) {
            no_targets += 1
            
            return
        }
        DriverStation.reportWarning("targetid, ${this.targetId}", false);

        for (camera in listOf("ATFrontLeft", "ATFrontRight")) {
            cameraSubsystem.io.getResults(camera)?.let { results -> 
                val validResults = results.map { 
                    it.targets.map {
                        Pair(it, it.getBestCameraToTarget())
                    }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ 
                        (this.targetId == 0 || this.targetId == it.first.fiducialId) && (reefTags.contains(it.first.fiducialId))
                    }
                    // removed sorting because only two results should exist right?
                    // .sortedBy { (it.second.getX().pow(2) + (it.second.getY() - cameraSubsystem.io.getLateralOffset(camera)!!.`in`(edu.wpi.first.units.Units.Meters)).pow(2)).pow(0.5)/*abs((((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) - 180.0)*/ } 
                }.filter {it.isNotEmpty()}
                    val result = validResults.firstOrNull()?.firstOrNull()

                
                if (result != null){
                    DriverStation.reportWarning("Has a target to align to, ${this.targetId}", false);
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
        DriverStation.reportWarning("Am i going to get yaw successfully? ${this.targetId}", false);
        var yaw = tagAngles.get(this.targetId) ?: return
        DriverStation.reportWarning("got yaw successfully", false);
        
        
        SmartDashboard.putNumber("target id", this.targetId.toDouble() * 10)
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
        val forward = /*if (abs(yaw - swerveSubsystem.heading.degrees) < skewTolerance && abs(lateralOffset.`in`(Units.Meters) - offsetY) < lateralTolerance) {*/
            -forwardPID.calculate(offsetX,0.0)
        /*} else {
            goBackwardsTimes += 1
            -0.3
        }*/

        
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
        return false//targetId == 0 // don't do anything if you don't have a targetId
        // return false//!(cameraSubsystem.io.hasTarget(reefTags, arrayOf("ATFrontRight", "ATFrontLeft")))//((yaw) < lateralTolerance) && (skew < skewTolerance) && (distance < distanceTolerance)
    }

    override fun end(interrupted: Boolean) {
        //SmartDashboard.
        SmartDashboard.putNumber("Align reef", 0.0) 
        //targetId = 0
    }
}
