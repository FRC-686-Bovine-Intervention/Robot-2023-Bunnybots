package frc.robot.subsystems.arm.manipulator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.CANDevices;

public class ManipulatorIOTalon implements ManipulatorIO {
    private final TalonSRX manipMotor = new TalonSRX(CANDevices.leftManipMotorID);
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.manipAppliedVolts = manipMotor.getMotorOutputVoltage();
        inputs.manipCurrentAmps = manipMotor.getStatorCurrent();
    }
    @Override
    public void setVoltage(double volts) {
        manipMotor.set(ControlMode.PercentOutput, volts / 12);
    }
}
