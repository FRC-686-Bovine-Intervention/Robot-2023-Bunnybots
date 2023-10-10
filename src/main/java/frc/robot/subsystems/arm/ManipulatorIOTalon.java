package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.CANDevices;

public class ManipulatorIOTalon implements ManipulatorIO {
    private final TalonSRX leftMotor = new TalonSRX(CANDevices.leftManipMotorID);
    private final TalonSRX rightMotor = new TalonSRX(CANDevices.rightManipMotorID);
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.leftAppliedVolts = leftMotor.getMotorOutputVoltage();
        inputs.leftCurrentAmps = leftMotor.getStatorCurrent();

        inputs.rightAppliedVolts = rightMotor.getMotorOutputVoltage();
        inputs.rightCurrentAmps = rightMotor.getStatorCurrent();
    }
    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.set(ControlMode.PercentOutput, volts / 12);
    }
    @Override
    public void setRightVoltage(double volts) {
        rightMotor.set(ControlMode.PercentOutput, volts / 12);
    }
}
