package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final AprilTagCamera[] cameras;

    public Vision() {
        cameras = new AprilTagCamera[] {
            new AprilTagCamera(VisionConstants.cameraNames[0], new AprilTagCameraIOPhotonVision(VisionConstants.cameraNames[0], VisionConstants.robotToCameras[0])),
            new AprilTagCamera(VisionConstants.cameraNames[1], new AprilTagCameraIOPhotonVision(VisionConstants.cameraNames[1], VisionConstants.robotToCameras[1])),
        };
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            camera.periodic();
        }
    }
}