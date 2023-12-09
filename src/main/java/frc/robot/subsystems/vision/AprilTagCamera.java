package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.AprilTagCameraIO.AprilTagCameraIOInputs;

public class AprilTagCamera {

    private final String name;
    private final AprilTagCameraIO cameraIO;
    private final AprilTagCameraIOInputs inputs = new AprilTagCameraIOInputs();

    public AprilTagCamera(String name, AprilTagCameraIO cameraIO) {
        this.name = name;
        this.cameraIO = cameraIO;
    }

    public void periodic() {
        // System.out.println("[DEBUG AprilTagCamera] Pre-Update Inputs");
        cameraIO.updateInputs(inputs);
        // System.out.println("[DEBUG AprilTagCamera] Pre-Process Inputs");
        Logger.processInputs("Vision/Camera/" + name, inputs);
        
        // update RobotState
        inputs.visionPose.ifPresent((pose) -> {
            // System.out.println("[DEBUG AprilTagCamera] Pre-Add Vision");
            RobotState.getInstance().addVisionMeasurement(
                pose.toPose2d(),
                computeStdDevs(0),  // TODO: figure out vision stdDevs 
                inputs.timestamp
            );
        });
        // System.out.println("[DEBUG AprilTagCamera] End Cam Loop");
    }

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double stdDev = Math.max(
            VisionConstants.minimumStdDev, 
            VisionConstants.stdDevEulerMultiplier * Math.exp(distance * VisionConstants.stdDevDistanceMultiplier)
        );
        return VecBuilder.fill(stdDev, stdDev, (DriverStation.isDisabled() ? VisionConstants.minimumStdDev : 1000));
    }
    
}