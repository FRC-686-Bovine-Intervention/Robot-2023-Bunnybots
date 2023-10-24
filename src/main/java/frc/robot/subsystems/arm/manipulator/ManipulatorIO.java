package frc.robot.subsystems.arm.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    public static ManipulatorIO blank() {
        return new ManipulatorIO() {
            @Override
            public void updateInputs(ManipulatorIOInputs inputs) {}
            @Override
            public void setLeftVoltage(double volts) {}
            @Override
            public void setRightVoltage(double volts) {}
        };
    }

    public void updateInputs(ManipulatorIOInputs inputs);

    public void setLeftVoltage(double volts);

    public void setRightVoltage(double volts);
}
