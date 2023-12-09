package frc.robot.subsystems.bunnyIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CANDevices;

public class BunnyIntakeIONeo implements BunnyIntakeIO {
    private final CANSparkMax bunnyIntakeMotor = new CANSparkMax(CANDevices.bunnyIntakeMotorID, MotorType.kBrushless);
    
    private static final boolean kMotorInverted = false;
    private static final double kOpenLoopRamp = 0.5;
    private static final int kStallCurrentLimit = 20;
    private static final int kFreeCurrentLimit = 20;

    public BunnyIntakeIONeo() {
        bunnyIntakeMotor.setInverted(kMotorInverted);
        bunnyIntakeMotor.setOpenLoopRampRate(kOpenLoopRamp);
        bunnyIntakeMotor.setSmartCurrentLimit(kStallCurrentLimit, kFreeCurrentLimit);
    }
    @Override
    public void updateInputs(BunnyIntakeIOInputs inputs) {
        inputs.bunnyIntakeAppliedVolts = bunnyIntakeMotor.getAppliedOutput();
        inputs.bunnyIntakeCurrentAmps = bunnyIntakeMotor.getOutputCurrent();
    }
    @Override
    public void setVoltage(double volts) {
        bunnyIntakeMotor.setVoltage(volts);
    }
}
