package frc.robot.subsystems.arm.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double armUncalRawPositionRad = 0.0;
        public double armPositionRad = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
        public double armTempCelcius = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmVoltage(double volts) {}

    public default void zeroEncoders() {}

    public default void setBrakeMode(Boolean enabled) {}
}
