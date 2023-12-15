package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.AprilTagCameraIO.AprilTagCameraIOInputs;
import frc.robot.util.LoggedTunableNumber;

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
                computeStdDevs(inputs.cameraToTargetDist),  // TODO: figure out vision stdDevs 
                inputs.timestamp
            );
        });
        // System.out.println("[DEBUG AprilTagCamera] End Cam Loop");
    }

    private static final LoggedTunableNumber kTransA = new LoggedTunableNumber("Vision/StdDevs/Translational/aCoef", 1);
    private static final LoggedTunableNumber kTransC = new LoggedTunableNumber("Vision/StdDevs/Translational/cCoef", 0.75);
    private static final LoggedTunableNumber kRotA = new LoggedTunableNumber("Vision/StdDevs/Rotational/aCoef", 5);
    private static final LoggedTunableNumber kRotC = new LoggedTunableNumber("Vision/StdDevs/Rotational/cCoef", 1000);
    private static final LoggedTunableNumber kRotCDisabled = new LoggedTunableNumber("Vision/StdDevs/Rotational/disabledcCoef", 5);

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double transStdDev = kTransA.get() * distance * distance + kTransC.get();
        double rotStdDev = kRotA.get() * distance * distance + (DriverStation.isEnabled() ? kRotC.get() : kRotCDisabled.get());
        return VecBuilder.fill(transStdDev, transStdDev, rotStdDev);
    }
}