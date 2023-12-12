package frc.robot.subsystems.bunnyIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
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
        
        zeroEncoders();
    }
    @Override
    public void updateInputs(BunnyIntakeIOInputs inputs) {
        inputs.bunnyIntakePositionRad = bunnyIntakeMotor.getEncoder().getPosition();
        inputs.bunnyIntakeAppliedVolts = bunnyIntakeMotor.getAppliedOutput();
        inputs.bunnyIntakeCurrentAmps = bunnyIntakeMotor.getOutputCurrent();
    }
    @Override
    public void setVoltage(double volts) {
        bunnyIntakeMotor.setVoltage(volts);
    }
    @Override
    public void zeroEncoders() {
        bunnyIntakeMotor.getEncoder().setPosition(MathUtil.inputModulus(bunnyIntakeMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(), 0, 2 * Math.PI));
    }
}
