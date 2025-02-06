package frc.robot.commands


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Pose
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.SwerveSubsystem
import org.photonvision.PhotonCamera

// TODO: Add your sequential commands in the super constructor call,
//       e.g. SequentialCommandGroup(OpenClawCommand(), MoveArmCommand())
class ReefScoreCommandGroup(private val swerveSubsystem: SwerveSubsystem, private val camera: PhotonCamera, private val armSubsystem: ArmSubsystem, private val elevatorSubsystem: ElevatorSubsystem, private val pose: Pose, private val lateralOffset: Double) : SequentialCommandGroup() {
    init {
        addCommands(
                ParallelCommandGroup(AlignReef(swerveSubsystem, camera, lateralOffset), gotoPoseCommand(armSubsystem, elevatorSubsystem, pose)),

        )
    }
}
