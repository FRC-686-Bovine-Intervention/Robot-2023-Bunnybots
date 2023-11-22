package frc.robot.subsystems.arm.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;

public class ArmIOFalcon implements ArmIO {
    private final TalonFX armMotor = new TalonFX(CANDevices.armMotorID, CANDevices.armCanBusName);
    private final CANcoder armEncoder = new CANcoder(CANDevices.armEncoderID, CANDevices.armCanBusName);

    public ArmIOFalcon() {
        var armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armMotor.getConfigurator().apply(armMotorConfig);

        var armEncoderConfig = new CANcoderConfiguration();

        armEncoder.getConfigurator().apply(armEncoderConfig);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPositionRad =       Units.rotationsToRadians(armEncoder.getPosition().getValue());
        inputs.armVelocityRadPerSec = Units.rotationsToRadians(armEncoder.getVelocity().getValue());
        inputs.armAppliedVolts =      armMotor.getSupplyVoltage().getValue();
        inputs.armCurrentAmps =       armMotor.getSupplyCurrent().getValue();
        inputs.armTempCelcius =       armMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public void zeroEncoders() {
        armEncoder.setPosition(armEncoder.getAbsolutePosition().getValue());
    }
}
