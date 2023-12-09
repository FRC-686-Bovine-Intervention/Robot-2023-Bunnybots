package frc.robot.subsystems.bunnyIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.util.LoggedTunableNumber;

public class BunnyIntakeIONeo implements BunnyIntakeIO {
    private final CANSparkMax bunnyIntakeMotor = new CANSparkMax(CANDevices.bunnyIntakeMotorID, MotorType.kBrushless);
    
    private final LoggedTunableNumber positionCalOffset = new LoggedTunableNumber("Arm/Arm Calibration Offset", 0);

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
        inputs.bunnyIntakeUncalRawPositionRad = Units.rotationsToRadians(bunnyIntakeMotor.getEncoder().getPosition());
        inputs.bunnyIntakePositionRad =         Units.rotationsToRadians(bunnyIntakeMotor.getEncoder().getPosition()) - positionCalOffset.get();
        inputs.bunnyIntakeVelocityRadPerSec =   Units.rotationsToRadians(bunnyIntakeMotor.getEncoder().getVelocity());
        inputs.bunnyIntakeAppliedVolts = bunnyIntakeMotor.getAppliedOutput();
        inputs.bunnyIntakeCurrentAmps = bunnyIntakeMotor.getOutputCurrent();
        inputs.bunnyIntakeTempCelcius = bunnyIntakeMotor.getMotorTemperature();
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
