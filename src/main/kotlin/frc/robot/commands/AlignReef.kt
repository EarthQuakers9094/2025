package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import org.photonvision.PhotonCamera

class AlignReef(private val swerveSubsystem: SwerveSubsystem, private val camera: PhotonCamera, private val lateralOffset: Double) : Command() {

    private val skewPID = PIDController(0.0, 0.0, 0.0)
    private val lateralPID = PIDController(0.0, 0.0, 0.0)
    private val forwardPID = PIDController(0.0, 0.0, 0.0)
    private val skewTolerance = 0.1
    private val lateralTolerance = 0.1
    private val distanceTolerance = 0.1
    private var skew = 0.0
    private var distance = 0.0
    private var yaw = 0.0


    init {

        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        val results = camera.allUnreadResults
        for (result in results) {
            //result.bestTarget.skew
            val target = result.bestTarget
            skew = target.skew
            yaw = target.yaw + lateralOffset
            distance = target.pitch
            val rotation = skewPID.calculate(skew,0.0)
            val lateral = lateralPID.calculate(yaw,0.0)
            val forward = forwardPID.calculate(distance,0.0)
            swerveSubsystem.drive(Translation2d(forward, lateral),rotation,false)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return ((yaw) < lateralTolerance) && (skew < skewTolerance) && (distance < distanceTolerance)
    }

    override fun end(interrupted: Boolean) {}
}
