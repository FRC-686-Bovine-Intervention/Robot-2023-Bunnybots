package frc.robot.subsystems.arm.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class ArmIOFalcon implements ArmIO {
    private final TalonFX armMotor = new TalonFX(CANDevices.armMotorID, CANDevices.armCanBusName);
    private final CANcoder armEncoder = new CANcoder(CANDevices.armEncoderID, CANDevices.armCanBusName);

    public ArmIOFalcon() {
        var armMotorConfig = new TalonFXConfiguration();

        armMotor.getConfigurator().apply(armMotorConfig);

        var armEncoderConfig = new CANcoderConfiguration();

        armEncoder.getConfigurator().apply(armEncoderConfig);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPositionRad =       Units.rotationsToRadians(armMotor.getPosition().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.armVelocityRadPerSec = Units.rotationsToRadians(armMotor.getVelocity().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.armAppliedVolts =      armMotor.getSupplyVoltage().getValue();
        inputs.armCurrentAmps =       armMotor.getSupplyCurrent().getValue();
        inputs.armTempCelcius =       armMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setArmVoltage(double volts) {
        armMotor.setControl(new DutyCycleOut(volts / 12));
    }

    @Override
    public void zeroEncoders() {
        armEncoder.setPosition(armEncoder.getAbsolutePosition().getValue());
    }
}
