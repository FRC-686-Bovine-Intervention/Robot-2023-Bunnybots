package frc.robot.subsystems.bunnyIntake;

import frc.robot.util.LoggedTunableNumber;

public class BunnyIntakeIOSim implements BunnyIntakeIO {
    private double appliedVolts;
    private final LoggedTunableNumber current = new LoggedTunableNumber("Simulation/BunnyIntake/Current", 50);
    @Override
    public void updateInputs(BunnyIntakeIOInputs inputs) {
        inputs.bunnyIntakeAppliedVolts = appliedVolts;
        inputs.bunnyIntakeCurrentAmps = current.get();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts * 12;
    }
}
