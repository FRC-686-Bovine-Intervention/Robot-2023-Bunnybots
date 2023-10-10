package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class ElbowIOFalcon implements ElbowIO {
    private final TalonFX elbowMotor = new TalonFX(CANDevices.elbowMotorID, CANDevices.armCanBusName);
    private final CANcoder elbowEncoder = new CANcoder(CANDevices.elbowEncoderID, CANDevices.armCanBusName);

    public ElbowIOFalcon() {
        var elbowMotorConfig = new TalonFXConfiguration();

        elbowMotor.getConfigurator().apply(elbowMotorConfig);

        var elbowEncoderConfig = new CANcoderConfiguration();

        elbowEncoder.getConfigurator().apply(elbowEncoderConfig);
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs) {
        inputs.elbowPositionRad =       Units.rotationsToRadians(elbowMotor.getPosition().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.elbowVelocityRadPerSec = Units.rotationsToRadians(elbowMotor.getVelocity().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.elbowAppliedVolts =      elbowMotor.getSupplyVoltage().getValue();
        inputs.elbowCurrentAmps =       elbowMotor.getSupplyCurrent().getValue();
        inputs.elbowTempCelcius =       elbowMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setElbowVoltage(double volts) {
        elbowMotor.setControl(new DutyCycleOut(volts / 12));
    }

    @Override
    public void zeroEncoders() {
        elbowEncoder.setPosition(elbowEncoder.getAbsolutePosition().getValue());
    }
}
