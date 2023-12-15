package frc.robot.subsystems.bunnyIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class BunnyIntakeIOSim implements BunnyIntakeIO {
    private double appliedVolts;
    private final SingleJointedArmSim simulation = new SingleJointedArmSim(
        DCMotor.getNeo550(1).withReduction(125),
        1,
        1,
        Units.inchesToMeters(12),
        Units.degreesToRadians(-110),
        Units.degreesToRadians(95),
        true,
        0
    );

    @Override
    public void updateInputs(BunnyIntakeIOInputs inputs) {
        simulation.update(Constants.dtSeconds);

        inputs.bunnyIntakePositionRad = simulation.getAngleRads() - Units.degreesToRadians(95);
        inputs.bunnyIntakeAppliedVolts = appliedVolts;
        inputs.bunnyIntakeCurrentAmps = simulation.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = -MathUtil.clamp(volts, -12.0, 12.0);
        simulation.setInputVoltage(appliedVolts);
    }
}
