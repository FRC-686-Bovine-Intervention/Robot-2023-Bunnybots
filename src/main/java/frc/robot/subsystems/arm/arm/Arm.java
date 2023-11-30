package frc.robot.subsystems.arm.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

    private final Mechanism2d armMech = new Mechanism2d(1, 1, new Color8Bit(Color.kBlack));
    public final MechanismLigament2d measuredArmLig = new MechanismLigament2d("Arm Measured", 0.485775, 0, 5, new Color8Bit(Color.kLightBlue));
    public final MechanismLigament2d setpointArmLig = new MechanismLigament2d("Arm Setpoint", 0.485775, 0, 5, new Color8Bit(Color.kLightGreen));

    public static enum ArmPos {
        Defense     (+0),
        Ground      (+2.371534),
        Hedge       (+2.133767),
        LowFront    (+1.93588),
        HighFront   (+0.89124),
        LowBack     (-1.852874),
        HighBack    (-0.89124),
        ;
        private final LoggedTunableNumber val;
        ArmPos(double defaultRads) {
            val = new LoggedTunableNumber("Arm/Positions/" + this.name(), defaultRads);
        }
        public double getRads() {return val.get();}
        public CommandBase goTo(Arm arm) {return arm.setArmPos(this);}
    }
    public ArmPos targetPos = ArmPos.Defense;

    private final LoggedTunableNumber armPIDkP =  new LoggedTunableNumber("Arm/PID/kP",  10);
    private final LoggedTunableNumber armPIDkI =  new LoggedTunableNumber("Arm/PID/kI",  0);
    private final LoggedTunableNumber armPIDkD =  new LoggedTunableNumber("Arm/PID/kD",  0);
    private final LoggedTunableNumber armPIDkV =  new LoggedTunableNumber("Arm/PID/kV",  3);
    private final LoggedTunableNumber armPIDkA =  new LoggedTunableNumber("Arm/PID/kA",  15);
    private final LoggedTunableNumber armPIDToleranceDeg = new LoggedTunableNumber("Arm/PID/Tolerance (Deg)",  5);
    private final ProfiledPIDController armPID =  new ProfiledPIDController(
        armPIDkP.get(),
        armPIDkI.get(),
        armPIDkD.get(),
        new Constraints(
            armPIDkV.get(),
            armPIDkA.get()
        )
    );

    private void updateTunables() {
        if(armPIDkP.hasChanged(hashCode()) || armPIDkI.hasChanged(hashCode()) || armPIDkD.hasChanged(hashCode())) {
            armPID.setPID(armPIDkP.get(), armPIDkI.get(), armPIDkD.get());
        }
        if(armPIDkV.hasChanged(hashCode()) || armPIDkA.hasChanged(hashCode())) {
            armPID.setConstraints(new Constraints(armPIDkV.get(), armPIDkA.get()));
        }
        if(armPIDToleranceDeg.hasChanged(hashCode())) {
            armPID.setTolerance(Units.degreesToRadians(armPIDToleranceDeg.get()));
        }
    }

    private static final ShuffleboardTab SBTab = Shuffleboard.getTab("Arm");
    public Arm(ArmIO armIO) {
        this.armIO = armIO;
        SBTab.add("Arm Subsystem", this);
        for(ArmPos pos : ArmPos.values()) {
            SBTab.add(pos.name(), setArmPos(pos));
        }
        var mechRoot = armMech.getRoot("Pivot", 0.5, 0.5).append(new MechanismLigament2d("Fixed", 0, 90, 0, new Color8Bit(Color.kBlack)));
        mechRoot.append(measuredArmLig);
        mechRoot.append(setpointArmLig);
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armIOInputs);
        Logger.getInstance().processInputs("Arm", armIOInputs);
        measuredArmLig.setAngle(-Units.radiansToDegrees(armIOInputs.armPositionRad));
        Logger.getInstance().recordOutput("Mechanism2d/Arm Side Profile", armMech);
        Logger.getInstance().recordOutput("ArmPos", getTargetPos().name());
        updateTunables();
    }

    private final LoggedTunableNumber manualArmVolts = new LoggedTunableNumber("Arm Volts", 2);
    public CommandBase setArmVolts(double dir) {
        return new StartEndCommand(
            () -> armIO.setArmVoltage(manualArmVolts.get() * dir),
            () -> armIO.setArmVoltage(0),
            this
        ).withName("Manual Volts: " + (manualArmVolts.get() * dir));
    }

    public CommandBase waitUntilAtGoal() {
        return new WaitUntilCommand(armPID::atGoal);
    }

    public boolean isAtPos(ArmPos pos) {
        return Math.abs(armIOInputs.armPositionRad - pos.getRads()) <= Units.degreesToRadians(armPIDToleranceDeg.get());
    }

    public ArmPos getTargetPos() {
        for(ArmPos pos : ArmPos.values()) {
            if(Math.abs(armPID.getGoal().position - pos.getRads()) < 0.01) {
                return pos;
            }
        }
        return null;
    }

    public void setBrakeMode(Boolean enabled) {
        armIO.setBrakeMode(enabled);
    }

    public CommandBase setArmPos(ArmPos pos) {
        return new ProfiledPIDCommand(
            armPID,
            () -> armIOInputs.armPositionRad, pos.getRads(),
            (output, setpoint) -> {
                armIO.setArmVoltage(output);
                setpointArmLig.setAngle(-Units.radiansToDegrees(setpoint.position));
            },
            this
        ).withName(pos.name());
    }
}
