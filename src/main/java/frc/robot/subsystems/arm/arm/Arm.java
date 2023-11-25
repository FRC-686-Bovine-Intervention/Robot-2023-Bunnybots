package frc.robot.subsystems.arm.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

    public enum ArmPos {
        Defense(0),
        Ground(0),
        Hedge(0),
        Low(0),
        High(0),
        ;
        private final LoggedTunableNumber val;
        ArmPos(double defaultRads) {
            val = new LoggedTunableNumber("Arm/Positions/" + this.name(), defaultRads);
        }
        public double getRads() {return val.get();}
    }
    public ArmPos targetPos = ArmPos.Defense;

    private final LoggedTunableNumber armPIDkP =  new LoggedTunableNumber("Arm/PID/kP",  0);
    private final LoggedTunableNumber armPIDkI =  new LoggedTunableNumber("Arm/PID/kI",  0);
    private final LoggedTunableNumber armPIDkD =  new LoggedTunableNumber("Arm/PID/kD",  0);
    private final LoggedTunableNumber armPIDkV =  new LoggedTunableNumber("Arm/PID/kV",  0);
    private final LoggedTunableNumber armPIDkA =  new LoggedTunableNumber("Arm/PID/kA",  0);
    private final ProfiledPIDController armPID =  new ProfiledPIDController(
        armPIDkP.get(),
        armPIDkI.get(),
        armPIDkD.get(),
        new Constraints(
            armPIDkV.get(),
            armPIDkA.get()
        )
    );

    private final LoggedTunableNumber armFFkS =   new LoggedTunableNumber("Arm/FF/kS",   0);
    private final LoggedTunableNumber armFFkV =   new LoggedTunableNumber("Arm/FF/kV",   0);
    private SimpleMotorFeedforward armFF =        new SimpleMotorFeedforward(
        armFFkS.get(),
        armFFkV.get()
    );

    private void updateTunables() {
        if(armPIDkP.hasChanged(hashCode()) || armPIDkI.hasChanged(hashCode()) || armPIDkD.hasChanged(hashCode())) {
            armPID.setPID(armPIDkP.get(), armPIDkI.get(), armPIDkD.get());
        }
        if(armPIDkV.hasChanged(hashCode()) || armPIDkA.hasChanged(hashCode())) {
            armPID.setConstraints(new Constraints(armPIDkV.get(), armPIDkA.get()));
        }
        if(armFFkS.hasChanged(hashCode()) || armFFkV.hasChanged(hashCode())) {
            armFF = new SimpleMotorFeedforward(armFFkS.get(), armFFkV.get());
        }
    }

    public Arm(ArmIO armIO) {
        this.armIO = armIO;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armIOInputs);
        Logger.getInstance().processInputs("Arm", armIOInputs);
        updateTunables();

        // armIO.setArmVoltage(
        //     armFF.calculate(armIOInputs.armVelocityRadPerSec) +
        //     armPID.calculate(armIOInputs.armPositionRad, targetPos.getRads())
        // );
    }

    public Command setTargetPos(ArmPos pos) {
        return new InstantCommand(() -> targetPos = pos, this);
    }
    public Command setTargetPosAndWait(ArmPos pos) {
        return setTargetPos(pos).andThen(new WaitUntilCommand(() -> isAtPos(pos)));
    }
    private final LoggedTunableNumber armvolts = new LoggedTunableNumber("Arm Volts", 0.25);
    public Command setArmVolts(double dir) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                armIO.setArmVoltage(armvolts.get() * dir);
            },
            (i) -> {
                armIO.setArmVoltage(0);
            },
            () -> false,
            this
        );
    }

    public static final double kArmPosTolerance = Math.PI / 16;
    public boolean isAtPos(ArmPos pos) {
        return Math.abs(armIOInputs.armPositionRad - pos.getRads()) <= kArmPosTolerance;
    }

    public void setBrakeMode(boolean enabled) {
        armIO.setBrakeMode(enabled);
    }
}
