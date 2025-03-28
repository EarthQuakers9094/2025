package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTablesJNI
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.robot.Robot
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.PhotonUtils
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import swervelib.SwerveDrive
import swervelib.telemetry.SwerveDriveTelemetry
import java.awt.Desktop
import java.util.*
import java.util.function.Supplier
import kotlin.math.max

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
class Vision
    (
    /**
     * Current pose from the pose estimator using wheel odometry.
     */
    private val currentPose: Supplier<Pose2d>,
    /**
     * Field from [swervelib.SwerveDrive.field]
     */
    private val field2d: Field2d
) {
    /**
     * Ambiguity defined as a value between (0,1). Used in [Vision.filterPose].
     */
    private val maximumAmbiguity = 0.25
    /**
     * Vision simulation.
     *
     * @return Vision Simulation
     */
    /**
     * Photon Vision Simulation
     */
    var visionSim: VisionSystemSim? = null

    /**
     * Count of times that the odom thinks we're more than 10meters away from the april tag.
     */
    private var longDistangePoseEstimationCount = 0.0


    /**
     * Constructor for the Vision class.
     *
     * @param currentPose Current pose supplier, should reference [SwerveDrive.getPose]
     * @param field       Current field, should be [SwerveDrive.field]
     */
    init {
        if (RobotBase.isSimulation()) {
            visionSim = VisionSystemSim("Vision")
            visionSim!!.addAprilTags(fieldLayout)

            for (c in Cameras.entries) {
                c.addToVisionSim(visionSim!!)
            }

            openSimCameraViews()
        }
    }

    /**
     * Update the pose estimation inside of [SwerveDrive] with all of the given poses.
     *
     * @param swerveDrive [SwerveDrive] instance.
     */
    fun updatePoseEstimation(swerveDrive: SwerveDrive) {
        if (SwerveDriveTelemetry.isSimulation && swerveDrive.simulationDriveTrainPose.isPresent) {
            /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
       */
            visionSim!!.update(swerveDrive.simulationDriveTrainPose.get())
        }
        for (camera in Cameras.entries) {
            val poseEst = getEstimatedGlobalPose(camera)
            if (poseEst!!.isPresent) {
                val pose = poseEst.get()
                swerveDrive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds,
                    camera.curStdDevs
                )
            }
        }
    }

    /**
     * Generates the estimated robot pose. Returns empty if:
     *
     *  *  No Pose Estimates could be generated
     *  *  The generated pose estimate was considered not accurate
     *
     *
     * @return an [EstimatedRobotPose] with an estimated pose, timestamp, and targets used to create the estimate
     */
    fun getEstimatedGlobalPose(camera: Cameras): Optional<EstimatedRobotPose>? {
        val poseEst = camera.estimatedGlobalPose
        if (RobotBase.isSimulation()) {
            val debugField = visionSim!!.debugField
            // Uncomment to enable outputting of vision targets in sim.
            poseEst!!.ifPresentOrElse(
                { est: EstimatedRobotPose ->
                    debugField
                        .getObject("VisionEstimation").pose = est.estimatedPose.toPose2d()
                },
                {
                    debugField.getObject("VisionEstimation").setPoses()
                })
        }
        return poseEst
    }


    /**
     * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
     * 10m for a short amount of time.
     *
     * @param pose Estimated robot pose.
     * @return Could be empty if there isn't a good reading.
     */
    @Deprecated("")
    private fun filterPose(pose: Optional<EstimatedRobotPose>): Optional<EstimatedRobotPose> {
        if (pose.isPresent) {
            var bestTargetAmbiguity = 1.0 // 1 is max ambiguity
            for (target in pose.get().targetsUsed) {
                val ambiguity = target.getPoseAmbiguity()
                if (ambiguity != -1.0 && ambiguity < bestTargetAmbiguity) {
                    bestTargetAmbiguity = ambiguity
                }
            }
            //ambiguity to high dont use estimate
            if (bestTargetAmbiguity > maximumAmbiguity) {
                return Optional.empty()
            }

            //est pose is very far from recorded robot pose
            if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1) {
                longDistangePoseEstimationCount++

                //if it calculates that were 10 meter away for more than 10 times in a row its probably right
                if (longDistangePoseEstimationCount < 10) {
                    return Optional.empty()
                }
            } else {
                longDistangePoseEstimationCount = 0.0
            }
            return pose
        }
        return Optional.empty()
    }


    /**
     * Get distance of the robot from the AprilTag pose.
     *
     * @param id AprilTag ID
     * @return Distance
     */
    fun getDistanceFromAprilTag(id: Int): Double {
        val tag = fieldLayout.getTagPose(id)
        return tag.map { pose3d: Pose3d ->
            PhotonUtils.getDistanceToPose(
                currentPose.get(),
                pose3d.toPose2d()
            )
        }.orElse(-1.0)
    }

    /**
     * Get tracked target from a camera of AprilTagID
     *
     * @param id     AprilTag ID
     * @param camera Camera to check.
     * @return Tracked target.
     */
    fun getTargetFromId(id: Int, camera: Cameras): PhotonTrackedTarget? {
        val target: PhotonTrackedTarget? = null
        for (result in camera.resultsList) {
            if (result.hasTargets()) {
                for (i in result.getTargets()) {
                    if (i.getFiducialId() == id) {
                        return i
                    }
                }
            }
        }
        return target
    }

    /**
     * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
     */
    private fun openSimCameraViews() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
//      try
//      {
//        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
//        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
//        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
//      } catch (IOException | URISyntaxException e)
//      {
//        e.printStackTrace();
//      }
        }
    }

    /**
     * Update the [Field2d] to include tracked targets/
     */
    fun updateVisionField() {
        val targets: MutableList<PhotonTrackedTarget> = ArrayList()
        for (c in Cameras.entries) {
            if (!c.resultsList.isEmpty()) {
                val latest = c.resultsList[0]
                if (latest.hasTargets()) {
                    targets.addAll(latest.targets)
                }
            }
        }

        val poses: MutableList<Pose2d> = ArrayList()
        for (target in targets) {
            if (fieldLayout.getTagPose(target.getFiducialId()).isPresent) {
                val targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d()
                poses.add(targetPose)
            }
        }

        field2d.getObject("tracked targets").poses = poses
    }

    /**
     * Camera Enum to select each camera
     */
    enum class Cameras
        (
        name: String, robotToCamRotation: Rotation3d, robotToCamTranslation: Translation3d,
        singleTagStdDevs: Matrix<N3, N1>, multiTagStdDevsMatrix: Matrix<N3, N1>
    ) {
        /**
         * Left Camera
         */
        LEFT_CAM(
            "left",
            Rotation3d(0.0, Math.toRadians(-24.094), Math.toRadians(30.0)),
            Translation3d(
                Units.inchesToMeters(10.108), //12.056), // forwards?
                Units.inchesToMeters(8.296),//10.981), // left-right
                Units.inchesToMeters(/* new up 6.58 */ 8.44) // up
            ),
            VecBuilder.fill(4.0, 4.0, 8.0), VecBuilder.fill(0.5, 0.5, 1.0)
        ),

        /**
         * Right Camera
         */
        RIGHT_CAM(
            "right",
            Rotation3d(0.0, Math.toRadians(-24.094), Math.toRadians(-30.0)),
            Translation3d(
                Units.inchesToMeters(10.108), //12.056), // forward
                Units.inchesToMeters(-8.296), // -10.981), // left-right
                Units.inchesToMeters(/* new up 6.58) */ 8.44) // up
            ),
            VecBuilder.fill(4.0, 4.0, 8.0), VecBuilder.fill(0.5, 0.5, 1.0)
        ),

        /**
         * Center Camera
         */
        CENTER_CAM(
            "center",
            Rotation3d(0.0, Units.degreesToRadians(18.0), 0.0),
            Translation3d(
                Units.inchesToMeters(-4.628),
                Units.inchesToMeters(-10.687),
                Units.inchesToMeters(39.0)
            ),
            VecBuilder.fill(4.0, 4.0, 8.0), VecBuilder.fill(0.5, 0.5, 1.0)
        );

        /**
         * Latency alert to use when high latency is detected.
         */
        // val latencyAlert: Alert =
        //     Alert("'$name' Camera is experiencing high latency.", AlertType.kWarning)

        /**
         * Camera instance for comms.
         */
        val camera: PhotonCamera = PhotonCamera(name)

        /**
         * Pose estimator for camera.
         */
        val poseEstimator: PhotonPoseEstimator

        /**
         * Standard Deviation for single tag readings for pose estimation.
         */
        private val singleTagStdDevs: Matrix<N3, N1>

        /**
         * Standard deviation for multi-tag readings for pose estimation.
         */
        private val multiTagStdDevs: Matrix<N3, N1>

        /**
         * Transform of the camera rotation and translation relative to the center of the robot
         */

        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        private val robotToCamTransform = Transform3d(robotToCamTranslation, robotToCamRotation)

        /**
         * Current standard deviations used.
         */
        var curStdDevs: Matrix<N3, N1>? = null

        /**
         * Estimated robot pose.
         */
        var estimatedRobotPose: Optional<EstimatedRobotPose>? = null

        /**
         * Simulated camera instance which only exists during simulations.
         */
        var cameraSim: PhotonCameraSim? = null

        /**
         * Results list to be updated periodically and cached to avoid unnecessary queries.
         */
        var resultsList: MutableList<PhotonPipelineResult> = ArrayList()

        /**
         * Last read from the camera timestamp to prevent lag due to slow data fetches.
         */
        private var lastReadTimestamp = edu.wpi.first.units.Units.Microseconds.of(NetworkTablesJNI.now().toDouble())
            .`in`(edu.wpi.first.units.Units.Seconds)

        /**
         * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
         * estimation noise on an actual robot.
         *
         * @param name                  Name of the PhotonVision camera found in the PV UI.
         * @param robotToCamRotation    [Rotation3d] of the camera.
         * @param robotToCamTranslation [Translation3d] relative to the center of the robot.
         * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
         * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
         */
        init {
            poseEstimator = PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamTransform
            )
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)

            this.singleTagStdDevs = singleTagStdDevs
            this.multiTagStdDevs = multiTagStdDevsMatrix

            if (RobotBase.isSimulation()) {
                val cameraProp = SimCameraProperties()
                // A 640 x 480 camera with a 100 degree diagonal FOV.
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100.0))
                // Approximate detection noise with average and standard deviation error in pixels.
                cameraProp.setCalibError(0.25, 0.08)
                // Set the camera image capture framerate (Note: this is limited by robot loop rate).
                cameraProp.fps = 30.0
                // The average and standard deviation in milliseconds of image data latency.
                cameraProp.avgLatencyMs = 35.0
                cameraProp.latencyStdDevMs = 5.0

                cameraSim = PhotonCameraSim(camera, cameraProp)
                cameraSim!!.enableDrawWireframe(true)
            }
        }

        /**
         * Add camera to [VisionSystemSim] for simulated photon vision.
         *
         * @param systemSim [VisionSystemSim] to use.
         */
        fun addToVisionSim(systemSim: VisionSystemSim) {
            if (RobotBase.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform)
            }
        }

        val bestResult: Optional<PhotonPipelineResult>
            /**
             * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
             * recent result!
             *
             * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
             */
            get() {
                if (resultsList.isEmpty()) {
                    return Optional.empty()
                }

                var bestResult = resultsList[0]
                var amiguity = bestResult.bestTarget.getPoseAmbiguity()
                var currentAmbiguity = 0.0
                for (result in resultsList) {
                    currentAmbiguity = result.bestTarget.getPoseAmbiguity()
                    if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
                        bestResult = result
                        amiguity = currentAmbiguity
                    }
                }
                return Optional.of(bestResult)
            }

        val latestResult: Optional<PhotonPipelineResult>
            /**
             * Get the latest result from the current cache.
             *
             * @return Empty optional if nothing is found. Latest result if something is there.
             */
            get() = if (resultsList.isEmpty()) Optional.empty() else Optional.of(
                resultsList[0]
            )

        val estimatedGlobalPose: Optional<EstimatedRobotPose>?
            /**
             * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
             * cache of results.
             *
             * @return Estimated pose.
             */
            get() {
                updateUnreadResults()
                return estimatedRobotPose
            }

        /**
         * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
         */
        private fun updateUnreadResults() {
            var mostRecentTimestamp = if (resultsList.isEmpty()) 0.0 else resultsList[0].timestampSeconds
            val currentTimestamp = edu.wpi.first.units.Units.Microseconds.of(NetworkTablesJNI.now().toDouble())
                .`in`(edu.wpi.first.units.Units.Seconds)
            val debounceTime = edu.wpi.first.units.Units.Milliseconds.of(15.0).`in`(edu.wpi.first.units.Units.Seconds)
            for (result in resultsList) {
                mostRecentTimestamp = max(mostRecentTimestamp, result.timestampSeconds)
            }
            if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
                (currentTimestamp - lastReadTimestamp) >= debounceTime
            ) {
                resultsList = if (RobotBase.isReal()) camera.allUnreadResults else cameraSim!!.camera.allUnreadResults
                lastReadTimestamp = currentTimestamp
                resultsList.sortWith(Comparator { a: PhotonPipelineResult, b: PhotonPipelineResult -> if (a.timestampSeconds >= b.timestampSeconds) 1 else -1 })
                if (!resultsList.isEmpty()) {
                    updateEstimatedGlobalPose()
                }
            }
        }

        /**
         * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
         * per loop.
         *
         *
         * Also includes updates for the standard deviations, which can (optionally) be retrieved with
         * [Cameras.updateEstimationStdDevs]
         *
         * @return An [EstimatedRobotPose] with an estimated pose, estimate timestamp, and targets used for
         * estimation.
         */
        private fun updateEstimatedGlobalPose() {
            var visionEst = Optional.empty<EstimatedRobotPose>()
            for (change in resultsList) {
                visionEst = poseEstimator.update(change)
                updateEstimationStdDevs(visionEst, change.getTargets())
            }
            estimatedRobotPose = visionEst
        }

        /**
         * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
         * on number of tags, estimation strategy, and distance from the tags.
         *
         * @param estimatedPose The estimated pose to guess standard deviations for.
         * @param targets       All targets in this camera frame
         */
        private fun updateEstimationStdDevs(
            estimatedPose: Optional<EstimatedRobotPose>, targets: List<PhotonTrackedTarget>
        ) {
            if (estimatedPose.isEmpty) {
                // No pose input. Default to single-tag std devs
                curStdDevs = singleTagStdDevs
            } else {
                // Pose present. Start running Heuristic
                var estStdDevs = singleTagStdDevs
                var numTags = 0
                var avgDist = 0.0

                // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (tgt in targets) {
                    val tagPose = poseEstimator.fieldTags.getTagPose(tgt.getFiducialId())
                    if (tagPose.isEmpty) {
                        continue
                    }
                    numTags++
                    avgDist +=
                        tagPose
                            .get()
                            .toPose2d()
                            .translation
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().translation)
                }

                if (numTags == 0) {
                    // No tags visible. Default to single-tag std devs
                    curStdDevs = singleTagStdDevs
                } else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags.toDouble()
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) {
                        estStdDevs = multiTagStdDevs
                    }
                    // Increase std devs based on (average) distance
                    estStdDevs = if (numTags == 1 && avgDist > 4) {
                        VecBuilder.fill(
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE
                        )
                    } else {
                        estStdDevs.times(1 + (avgDist * avgDist / 30))
                    }
                    curStdDevs = estStdDevs
                }
            }
        }
    }

    companion object {
        /**
         * April Tag Field Layout of the year.
         */
        val fieldLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2024Crescendo
        )

        /**
         * Calculates a target pose relative to an AprilTag on the field.
         *
         * @param aprilTag    The ID of the AprilTag.
         * @param robotOffset The offset [Transform2d] of the robot to apply to the pose for the robot to position
         * itself correctly.
         * @return The target pose of the AprilTag.
         */
        fun getAprilTagPose(aprilTag: Int, robotOffset: Transform2d?): Pose2d {
            val aprilTagPose3d = fieldLayout.getTagPose(aprilTag)
            if (aprilTagPose3d.isPresent) {
                return aprilTagPose3d.get().toPose2d().transformBy(robotOffset)
            } else {
                throw RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString())
            }
        }
    }
}
