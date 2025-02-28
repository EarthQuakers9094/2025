// class ReefScoreCommandGroup(private val swerveSubsystem: SwerveSubsystem, private val camera: PhotonCamera, private val armSubsystem: ArmSubsystem, private val elevatorSubsystem: ElevatorSubsystem, private val pose: Pose, private val lateralOffset: Double) : SequentialCommandGroup() {
//     init {
//         addCommands(
//                // ParallelCommandGroup(AlignReef(swerveSubsystem, camera, lateralOffset), gotoPoseCommand(armSubsystem, elevatorSubsystem, pose)),

//         )
//     }
// }