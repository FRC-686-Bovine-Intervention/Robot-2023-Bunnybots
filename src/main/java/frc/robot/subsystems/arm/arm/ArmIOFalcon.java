package frc.robot.subsystems.arm.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.util.LoggedTunableNumber;

public class ArmIOFalcon implements ArmIO {
    private final TalonFX armMotor = new TalonFX(CANDevices.armMotorID, CANDevices.armCanBusName);
    private final CANcoder armEncoder = new CANcoder(CANDevices.armEncoderID, CANDevices.armCanBusName);

    private final LoggedTunableNumber positionCalOffset = new LoggedTunableNumber("Arm Calibration Offset", 1.7380002326744315);

    public ArmIOFalcon() {
        var armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotor.getConfigurator().apply(armMotorConfig);

        var armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armEncoder.getConfigurator().apply(armEncoderConfig);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armUncalRawPositionRad = Units.rotationsToRadians(armEncoder.getPosition().getValue());
        inputs.armPositionRad =         Units.rotationsToRadians(armEncoder.getPosition().getValue()) - positionCalOffset.get();
        inputs.armVelocityRadPerSec =   Units.rotationsToRadians(armEncoder.getVelocity().getValue());
        inputs.armAppliedVolts =        armMotor.getSupplyVoltage().getValue();
        inputs.armCurrentAmps =         armMotor.getSupplyCurrent().getValue();
        inputs.armTempCelcius =         armMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public void zeroEncoders() {
        armEncoder.setPosition(armEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public void setBrakeMode(Boolean enabled) {
        armMotor.setControl(enabled == null ? new NeutralOut() : (enabled.booleanValue() ? new StaticBrake() : new CoastOut()));
    }
}
