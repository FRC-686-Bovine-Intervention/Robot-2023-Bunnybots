package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ElbowIO.ElbowIOInputs;
import frc.robot.subsystems.arm.ManipulatorIO.ManipulatorIOInputs;
import frc.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private final ElbowIO elbowIO;
    private final ElbowIOInputs elbowIOInputs = new ElbowIOInputsAutoLogged();
    private final ManipulatorIO manipIO;
    private final ManipulatorIOInputs manipIOInputs = new ManipulatorIOInputsAutoLogged();

    // Elbow
    // | States
    public enum ElbowPos {
        Ground(0),
        Hedge(0),
        Low(0),
        High(0),
        ;
        private final LoggedTunableNumber val;
        ElbowPos(double defaultValue) {
            val = new LoggedTunableNumber("Arm/Elbow/Positions/" + this.name(), defaultValue);
        }
        public double get() {return val.get();}
    }
    public ElbowPos targetPos = ElbowPos.Ground;
    // | PID
    private final LoggedTunableNumber elbowPIDkP =  new LoggedTunableNumber("Arm/Elbow/PID/kP",  0);
    private final LoggedTunableNumber elbowPIDkI =  new LoggedTunableNumber("Arm/Elbow/PID/kI",  0);
    private final LoggedTunableNumber elbowPIDkD =  new LoggedTunableNumber("Arm/Elbow/PID/kD",  0);
    private final LoggedTunableNumber elbowPIDkV =  new LoggedTunableNumber("Arm/Elbow/PID/kV",  0);
    private final LoggedTunableNumber elbowPIDkA =  new LoggedTunableNumber("Arm/Elbow/PID/kA",  0);
    private final ProfiledPIDController elbowPID =  new ProfiledPIDController(
        elbowPIDkP.get(), 
        elbowPIDkI.get(), 
        elbowPIDkD.get(), 
        new Constraints(
            elbowPIDkV.get(), 
            elbowPIDkA.get()
        )
    );

    // | Feedforward
    private final LoggedTunableNumber elbowFFkS =   new LoggedTunableNumber("Arm/Elbow/FF/kS",   0);
    private final LoggedTunableNumber elbowFFkV =   new LoggedTunableNumber("Arm/Elbow/FF/kV",   0);
    private SimpleMotorFeedforward elbowFF =        new SimpleMotorFeedforward(
        elbowFFkS.get(), 
        elbowFFkV.get()
    );

    // Manipulator
    private final LoggedTunableNumber manipIntakePower = new LoggedTunableNumber("Arm/Manipulator/Intake Power", -0.5);
    private final LoggedTunableNumber manipScorePower = new LoggedTunableNumber("Arm/Manipulator/Scoring Power", +1.0);
    private final LoggedTunableNumber manipHedgePower = new LoggedTunableNumber("Arm/Manipulator/Hedge Power",   +0.5);

    private void updateTunables() {
        if(elbowPIDkP.hasChanged(hashCode()) || elbowPIDkI.hasChanged(hashCode()) || elbowPIDkD.hasChanged(hashCode())) {
            elbowPID.setPID(elbowPIDkP.get(), elbowPIDkI.get(), elbowPIDkD.get());
        }
        if(elbowPIDkV.hasChanged(hashCode()) || elbowPIDkA.hasChanged(hashCode())) {
            elbowPID.setConstraints(new Constraints(elbowPIDkV.get(), elbowPIDkA.get()));
        }
        if(elbowFFkS.hasChanged(hashCode()) || elbowFFkV.hasChanged(hashCode())) {
            elbowFF = new SimpleMotorFeedforward(elbowFFkS.get(), elbowFFkV.get());
        }
    }


    public Arm(ElbowIO elbowIO, ManipulatorIO manipIO) {
        this.elbowIO = elbowIO;
        this.manipIO = manipIO;
    }

    @Override
    public void periodic() {
        elbowIO.updateInputs(elbowIOInputs);
        manipIO.updateInputs(manipIOInputs);
        updateTunables();

        elbowIO.setElbowVoltage(
            elbowFF.calculate(elbowIOInputs.elbowVelocityRadPerSec) + 
            elbowPID.calculate(elbowIOInputs.elbowPositionRad, 0)
        );
    }

    public InstantCommand setElbowPos(ElbowPos pos) {
        return new InstantCommand(() -> targetPos = pos, this);
    }

    // public FunctionalCommand score() {
    //     return new FunctionalCommand(null, , (Boolean i)->{}, ()->false, this);
    // }
}
