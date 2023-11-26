package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;

        public double driveSetpointRadPerSec = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double volts) {}

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(double volts) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(Boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(Boolean enable) {}

    /** Zero drive encoders */
    public default void zeroEncoders() {}
}
