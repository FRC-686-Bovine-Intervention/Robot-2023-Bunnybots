package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ElbowIO.ElbowIOInputs;
import frc.robot.subsystems.arm.ManipulatorIO.ManipulatorIOInputs;

public class Arm extends SubsystemBase {
    private final ElbowIO elbowIO;
    private final ElbowIOInputs elbowIOInputs = new ElbowIOInputsAutoLogged();
    private final ManipulatorIO manipIO;
    private final ManipulatorIOInputs manipIOInputs = new ManipulatorIOInputsAutoLogged();

    public Arm(ElbowIO elbowIO, ManipulatorIO manipIO) {
        this.elbowIO = elbowIO;
        this.manipIO = manipIO;
    }

    @Override
    public void periodic() {
        elbowIO.updateInputs(elbowIOInputs);
        manipIO.updateInputs(manipIOInputs);
    }
}
