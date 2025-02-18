

import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.targeting.PhotonPipelineResult
import edu.wpi.first.units.measure.Distance


class VisionSubsystem {
    interface VisionIO {
        fun periodic()
        fun getFrontCameras(): HashMap<String, CameraAlignInfo> 
        fun hasTarget(filter: Array<Int>, cameras: Array<String>): Boolean
        fun getResults(camera: String): List<PhotonPipelineResult>?

    }

    class VisionRealIO(val frontLeftCamera: PhotonCamera, val frontLeftLateralOffset: Distance, val frontRightCamera: PhotonCamera, val frontRightLateralOffset: Distance):VisionIO {

        var cameraResults = HashMap<String, MutableList<PhotonPipelineResult>>()

        override fun periodic() {
            //cameraResults = HashMap()
            for (camera in this.getFrontCameras()) {
                val name = camera.key
                val cam = camera.value
                if (cameraResults.containsKey(name)) {
                    cameraResults.get(name)!! += cam.camera.allUnreadResults
                } else {
                    cameraResults.put(name, cam.camera.allUnreadResults)
                }
                
                //cam.camera.allUnreadResults
                // camera.camera.allUnreadResults
                // camera.camera.name
            }
            //this.getFrontCameras().map { it.camera.allUnreadResults }
        }

        override fun getFrontCameras(): HashMap<String, CameraAlignInfo> {
            return hashMapOf(frontLeftCamera.name to CameraAlignInfo(frontLeftCamera, frontLeftLateralOffset), frontRightCamera.name to CameraAlignInfo(frontRightCamera, frontRightLateralOffset))
        }



        override fun hasTarget(filter: Array<Int>, cameras: Array<String>): Boolean {
            for ((name, results) in cameraResults) {
                if (!cameras.contains(name)) {
                    continue
                }
                if (results.filter {it.targets.any{filter.contains(it.fiducialId)}}.isEmpty()) {
                    return false
                } else {
                    return true
                }
            }
            return false
            
            
            //results.map { it.targets.map { Pair(it, it.getBestCameraToTarget()) }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ (reefTags.contains(it.first.fiducialId) && (targetId == 0 || targetId == it.first.fiducialId)) }.sortedBy { it.first.poseAmbiguity } }.filter {it.isNotEmpty()}
        }
        override fun getResults(camera: String): List<PhotonPipelineResult>? {
            val results = cameraResults.get(camera)
            cameraResults.remove(camera)
            return results
            
            
            //results.map { it.targets.map { Pair(it, it.getBestCameraToTarget()) }.filter { /*(((it.second.rotation.getZ() * (180/Math.PI)) + 360.0) % 360.0) < (100.0)  &&*/ (reefTags.contains(it.first.fiducialId) && (targetId == 0 || targetId == it.first.fiducialId)) }.sortedBy { it.first.poseAmbiguity } }.filter {it.isNotEmpty()}
        }

        
    }
    class VisionSimIO(): VisionIO {
        //private val frontCameras = arrayOf(CameraAlignInfo(PhotonCameraSim(), lateralOffset))

        override fun periodic() { }

        override fun getFrontCameras(): HashMap<String, CameraAlignInfo> { 
            TODO("got lazy")
        }

        

        override fun hasTarget(filter: Array<Int>, cameras: Array<String>): Boolean {
            TODO("got lazy")
         }

        override fun getResults(camera: String): List<PhotonPipelineResult>? {
            TODO("got lazy")
        }
    }
}