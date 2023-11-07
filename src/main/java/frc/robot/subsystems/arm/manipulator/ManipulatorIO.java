package frc.robot.subsystems.arm.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        public double manipAppliedVolts = 0.0;
        public double manipCurrentAmps = 0.0;
    }

    public static ManipulatorIO blank() {
        return new ManipulatorIO() {
            @Override
            public void updateInputs(ManipulatorIOInputs inputs) {}
            @Override
            public void setVoltage(double volts) {}
        };
    }

    public void updateInputs(ManipulatorIOInputs inputs);

    public void setVoltage(double volts);
}
