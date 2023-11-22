package frc.robot.subsystems.arm.manipulator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.CANDevices;

public class ManipulatorIOVictor implements ManipulatorIO {
    private final VictorSPX manipMotor = new VictorSPX(CANDevices.manipMotorID);

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.manipAppliedVolts = manipMotor.getMotorOutputVoltage();
        // inputs.manipCurrentAmps = manipMotor
    }
    @Override
    public void setVoltage(double volts) {
        manipMotor.set(ControlMode.PercentOutput, volts / 12);
    }
}
