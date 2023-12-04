package frc.robot.subsystems.arm.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1).withReduction(25), 4, 1, 0.876300, Arm.ArmPos.LowBack.getRads() + Units.degreesToRadians(90), Arm.ArmPos.Ground.getRads() + Units.degreesToRadians(90), true, 0);

    private double armAppliedVolts;
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(Constants.dtSeconds);

        inputs.armPositionRad = armSim.getAngleRads() - Units.degreesToRadians(90);
        inputs.armVelocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.armAppliedVolts = armAppliedVolts;
        inputs.armCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());
        inputs.armTempCelcius = 0.0;
    }
    @Override
    public void setArmVoltage(double volts) {
        armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        armSim.setInputVoltage(armAppliedVolts);
    }
    @Override
    public void zeroEncoders() {}
    @Override
    public void setBrakeMode(Boolean enabled) {}
}
