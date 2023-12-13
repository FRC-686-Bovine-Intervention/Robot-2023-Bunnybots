package frc.robot.subsystems.bunnyIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;

public class BunnyIntakeIONeo implements BunnyIntakeIO {
    private final CANSparkMax bunnyIntakeMotor = new CANSparkMax(CANDevices.bunnyIntakeMotorID, MotorType.kBrushless);
    
    private static final boolean kMotorInverted = true;
    private static final double kOpenLoopRamp = 0.75;
    private static final int kStallCurrentLimit = 10;
    private static final int kFreeCurrentLimit = 20;

    public BunnyIntakeIONeo() {
        bunnyIntakeMotor.setInverted(kMotorInverted);
        // bunnyIntakeMotor.setOpenLoopRampRate(kOpenLoopRamp);
        bunnyIntakeMotor.setSmartCurrentLimit(kStallCurrentLimit, kFreeCurrentLimit);
        
        zeroEncoders();
    }
    @Override
    public void updateInputs(BunnyIntakeIOInputs inputs) {
        inputs.bunnyIntakePositionRad = Units.rotationsToRadians(bunnyIntakeMotor.getEncoder().getPosition() / 125);
        inputs.bunnyIntakeAppliedVolts = bunnyIntakeMotor.getAppliedOutput();
        inputs.bunnyIntakeCurrentAmps = bunnyIntakeMotor.getOutputCurrent();
    }
    @Override
    public void setVoltage(double volts) {
        bunnyIntakeMotor.setVoltage(volts);
    }
    @Override
    public void zeroEncoders() {
        bunnyIntakeMotor.getEncoder().setPosition(0);
    }
}
