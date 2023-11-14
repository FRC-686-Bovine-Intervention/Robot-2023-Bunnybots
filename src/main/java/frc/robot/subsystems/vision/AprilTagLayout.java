package frc.robot.subsystems.vision;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class AprilTagLayout {
    AprilTagLayout() {
        try {
            new AprilTagFieldLayout("src/main/deploy/Bunnybot_2023.json");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
