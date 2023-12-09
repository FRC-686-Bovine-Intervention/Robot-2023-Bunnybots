package frc.robot.subsystems.bunnyIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedTunableNumber;

public class BunnyIntakeIOSim implements BunnyIntakeIO {
    private double appliedVolts;
    private final LoggedTunableNumber current = new LoggedTunableNumber("Simulation/BunnyIntake/Current", 50);

    private final SingleJointedArmSim bunnyIntakeSim = new SingleJointedArmSim(DCMotor.getNeo550(1).withReduction(125), 1, 1, 0.876300, BunnyIntake.BunnyIntakePos.Defense.getRads(), BunnyIntake.BunnyIntakePos.Score.getRads(), true, 0);
    private double armAppliedVolts;
    @Override
    public void updateInputs(BunnyIntakeIOInputs inputs) {        
        inputs.bunnyIntakePositionRad = bunnyIntakeSim.getAngleRads();
        inputs.bunnyIntakeVelocityRadPerSec = bunnyIntakeSim.getVelocityRadPerSec();
        inputs.bunnyIntakeAppliedVolts = armAppliedVolts;
        inputs.bunnyIntakeCurrentAmps = Math.abs(bunnyIntakeSim.getCurrentDrawAmps());
        inputs.bunnyIntakeAppliedVolts = appliedVolts;
        inputs.bunnyIntakeCurrentAmps = current.get();
        inputs.bunnyIntakeTempCelcius = 0.0;
    }

    @Override
    public void setVoltage(double volts) {
        armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        bunnyIntakeSim.setInputVoltage(armAppliedVolts);
    }
}
