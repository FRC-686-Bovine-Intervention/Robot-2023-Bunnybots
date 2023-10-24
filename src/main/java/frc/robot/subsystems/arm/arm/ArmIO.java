package frc.robot.subsystems.arm.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    
    @AutoLog
    public static class ArmIOInputs {
        public double armPositionRad = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
        public double armTempCelcius = 0.0;
    }
    public static ArmIO blank() {
        return new ArmIO() {
            @Override
            public void updateInputs(ArmIOInputs inputs) {}
            @Override
            public void setArmVoltage(double volts) {}
            @Override
            public void zeroEncoders() {}
        };
    }

    public void updateInputs(ArmIOInputs inputs);
    
    public void setArmVoltage(double volts);

    public void zeroEncoders();
}
