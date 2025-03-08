

import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.targeting.PhotonPipelineResult
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import CameraAlignInfo
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.SwerveSubsystem
import org.photonvision.PhotonPoseEstimator
import kotlin.jvm.optionals.getOrNull


class VisionSubsystem(val io: VisionIO): SubsystemBase() {
    override fun periodic() {
        io.periodic()
    }
    interface VisionIO {
        fun periodic()
//        public fun getFrontCameras(): HashMap<String, CameraAlignInfo>
//        public fun getBackCameras(): HashMap<String, CameraAlignInfo>
//        public fun hasTarget(filter: Array<Int>, cameras: Array<String>): Boolean
//        public fun getResults(camera: String): List<PhotonPipelineResult>?
        //fun getLateralOffset(camera: String): Distance?

    }

    class VisionRealIO(val swerveSubsystem: SwerveSubsystem, val frontLeftCamera: PhotonCamera, val frontLeftOffset: Transform3d, val frontRightCamera: PhotonCamera, val frontRightOffset: Transform3d, val backCenterCamera: PhotonCamera, val backCenterOffset: Transform3d):VisionIO {

        private var cameraResults = HashMap<String, MutableList<PhotonPipelineResult>>()
        private var readResults = HashMap<String,Boolean>()
        private var frontLeftPoseEstimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            frontLeftOffset
        )
        private var frontRightPoseEstimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            frontRightOffset
        )
        private var backCenterPoseEstimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            backCenterOffset
        )

        override fun periodic() {
            //cameraResults = HashMap()
//            for (camera in this.getFrontCameras() + this.getBackCameras()) {
//                val name = camera.key
//                val cam = camera.value
//                if (readResults.get(name) == true) {
//                    cameraResults.remove(name)
//                }
//                //TODO(name)
//                if (cameraResults.containsKey(name)) {
//                    cameraResults.get(name)!! += cam.camera.allUnreadResults
//                } else {
//                    cameraResults.put(name, cam.camera.allUnreadResults)
//                }
//
//                //cam.camera.allUnreadResults
//                // camera.camera.allUnreadResults
//                // camera.camera.name
//            }
            frontLeftCamera.allUnreadResults.lastOrNull()?.let {
                val pose = frontLeftPoseEstimator.update(it).getOrNull()
                pose?.let { swerveSubsystem.swerveDrive.addVisionMeasurement(it.estimatedPose.toPose2d(), Timer.getFPGATimestamp()) }

            }
            frontRightCamera.allUnreadResults.lastOrNull()?.let {
                val pose = frontRightPoseEstimator.update(it).getOrNull()
                pose?.let { swerveSubsystem.swerveDrive.addVisionMeasurement(it.estimatedPose.toPose2d(), Timer.getFPGATimestamp()) }
            }
            backCenterCamera.allUnreadResults.lastOrNull()?.let {
                val pose = backCenterPoseEstimator.update(it).getOrNull()
                pose?.let { swerveSubsystem.swerveDrive.addVisionMeasurement(it.estimatedPose.toPose2d(), Timer.getFPGATimestamp()) }
            }

            //this.getFrontCameras().map { it.camera.allUnreadResults }
        }

//        override fun getFrontCameras(): HashMap<String, CameraAlignInfo> {
//            return hashMapOf(frontLeftCamera.name to CameraAlignInfo(frontLeftCamera, frontLeftLateralOffset), frontRightCamera.name to CameraAlignInfo(frontRightCamera, frontRightLateralOffset))
//        }



//        override fun hasTarget(filter: Array<Int>, cameras: Array<String>): Boolean {
//            for ((name, results) in cameraResults) {
//                if (!cameras.contains(name)) {
//                    continue
//                }
//                if (results.filter {it.targets.any{filter.contains(it.fiducialId)}}.isEmpty()) {
//                    return false
//                } else {
//                    return true
//                }
//            }
//            return false
//
//
//            //results.map { it.targets.map { Pair(it, it.getBestCameraToTarget()) }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ (reefTags.contains(it.first.fiducialId) && (targetId == 0 || targetId == it.first.fiducialId)) }.sortedBy { it.first.poseAmbiguity } }.filter {it.isNotEmpty()}
//        }
//        override fun getResults(camera: String): List<PhotonPipelineResult>? {
//            val results = cameraResults.get(camera)
//            readResults[camera] = true
//            //cameraResults.remove(camera)
//            return results
//
//
//            //results.map { it.targets.map { Pair(it, it.getBestCameraToTarget()) }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ (reefTags.contains(it.first.fiducialId) && (targetId == 0 || targetId == it.first.fiducialId)) }.sortedBy { it.first.poseAmbiguity } }.filter {it.isNotEmpty()}
//        }
//        override fun getLateralOffset(camera: String): Distance? {
//            return this.getFrontCameras().get(camera)?.lateralOffset
//        }
//
//        override fun getBackCameras(): HashMap<String, CameraAlignInfo> {
//            //TODO("askdhfjkasdhfksdkjafjkadsjkfds")
//            return hashMapOf(backCenterCamera.name to CameraAlignInfo(backCenterCamera, backCenterLateralOffset))
//        }

        

        
    }
    class VisionSimIO(): VisionIO {
        //private val frontCameras = arrayOf(CameraAlignInfo(PhotonCameraSim(), lateralOffset))

        override fun periodic() { }

//        override fun getFrontCameras(): HashMap<String, CameraAlignInfo> {
//            TODO("got lazy")
//        }
//
//
//
//        override fun hasTarget(filter: Array<Int>, cameras: Array<String>): Boolean {
//            TODO("got lazy")
//         }
//
//        override fun getResults(camera: String): List<PhotonPipelineResult>? {
//            TODO("got lazy")
//        }
//
//
//        override fun getLateralOffset(camera: String): Distance? {
//            TODO("got lazy")
//        }
//
//        override fun getBackCameras(): HashMap<String, CameraAlignInfo> { TODO("stayed lazy")}

        
    }
}