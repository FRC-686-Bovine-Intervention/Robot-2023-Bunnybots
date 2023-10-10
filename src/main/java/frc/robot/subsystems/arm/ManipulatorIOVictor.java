package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.CANDevices;

public class ManipulatorIOVictor implements ManipulatorIO {
    private final VictorSPX leftMotor = new VictorSPX(CANDevices.leftManipMotorID);
    private final VictorSPX rightMotor = new VictorSPX(CANDevices.rightManipMotorID);
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.leftAppliedVolts = leftMotor.getMotorOutputVoltage();
        // inputs.leftCurrentAmps = leftMotor

        inputs.rightAppliedVolts = rightMotor.getMotorOutputVoltage();
        // inputs.rightCurrentAmps = rightMotor
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
