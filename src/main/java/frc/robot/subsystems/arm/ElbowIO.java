package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ElbowIO {
    
    @AutoLog
    public static class ElbowIOInputs {
        public double elbowPositionRad = 0.0;
        public double elbowVelocityRadPerSec = 0.0;
        public double elbowAppliedVolts = 0.0;
        public double elbowCurrentAmps = 0.0;
        public double elbowTempCelcius = 0.0;
    }
    public static ElbowIO blank() {
        return new ElbowIO() {
            @Override
            public void updateInputs(ElbowIOInputs inputs) {}
            @Override
            public void setElbowVoltage(double volts) {}
            @Override
            public void zeroEncoders() {}
        };
    }

    public void updateInputs(ElbowIOInputs inputs);
    
    public void setElbowVoltage(double volts);

    public void zeroEncoders();
}
