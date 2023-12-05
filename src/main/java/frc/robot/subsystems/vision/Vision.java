package frc.robot.subsystems.vision;

import frc.robot.util.VirtualSubsystem;

public class Vision extends VirtualSubsystem {

    private final AprilTagCamera[] cameras;

    public Vision(AprilTagCamera... cameras) {
        System.out.println("[Init Vision] Instantiating Vision");
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            camera.periodic();
        }
    }
}