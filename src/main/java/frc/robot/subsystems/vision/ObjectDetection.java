package frc.robot.subsystems.vision;

import org.ejml.equation.Variable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase{

    private PhotonTrackedTarget target;
    private PhotonCamera obDetectCam;
    private Transform3d robotToCamera;

    public ObjectDetection() {

        obDetectCam = new PhotonCamera(VisionConstants.camera3Name);
        robotToCamera = VisionConstants.robotToCamera3;

    }

    public double aimAtCluster() {
        var result = obDetectCam.getLatestResult();

        if (result.hasTargets()) {
            double targetRotation = result.getBestTarget().getArea();
            return targetRotation;
        } else {
            return 0;
        }
        
    }

    public double calculateDistance() {
        var result = obDetectCam.getLatestResult();
        if (result.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(), 
                0, 
                0, 
                Units.degreesToRadians(result.getBestTarget().getArea()));
            return distance;
        } else {
            return 0;
        }

    }




    
}
