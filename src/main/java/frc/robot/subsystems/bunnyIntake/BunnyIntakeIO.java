package frc.robot.subsystems.bunnyIntake;

import org.littletonrobotics.junction.AutoLog;

public interface BunnyIntakeIO {
    @AutoLog
    public static class BunnyIntakeIOInputs {
        public double bunnyIntakePositionRad = 0.0;
        public double bunnyIntakeAppliedVolts = 0.0;
        public double bunnyIntakeCurrentAmps = 0.0;
    }

    public default void updateInputs(BunnyIntakeIOInputs inputs) {}

    public default void setVoltage(double volts) {}
    
    public default void zeroEncoders() {}
    
}