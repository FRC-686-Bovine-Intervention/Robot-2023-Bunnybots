package frc.robot.subsystems.arm.manipulator;

import frc.robot.util.LoggedTunableNumber;

public class ManipulatorIOSim implements ManipulatorIO {
    private double appliedVolts;
    private final LoggedTunableNumber current = new LoggedTunableNumber("Simulation/Manipulator/Current", 50);
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.manipAppliedVolts = appliedVolts;
        inputs.manipCurrentAmps = current.get();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts * 12;
    }
}
