package frc.robot.subsystems.arm.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO manipIO;
    private final ManipulatorIOInputsAutoLogged manipIOInputs = new ManipulatorIOInputsAutoLogged();

    private final LoggedTunableNumber manipIntakePower = new LoggedTunableNumber("Arm/Manipulator/Intake Power", -0.75);
    private final LoggedTunableNumber manipScorePower = new LoggedTunableNumber("Arm/Manipulator/Scoring Power", +0.75);
    private final LoggedTunableNumber manipHedgePower = new LoggedTunableNumber("Arm/Manipulator/Hedge Power",   +0.5);

    public Manipulator(ManipulatorIO manipIO) {
        this.manipIO = manipIO;
    }

    @Override
    public void periodic() {
        manipIO.updateInputs(manipIOInputs);
        Logger.getInstance().processInputs("Manip", manipIOInputs);
    }

    public Command intake() {
        return new FunctionalCommand(
            ()->{},
            ()->{
                manipIO.setVoltage(manipIntakePower.get());
            },
            (Boolean interrupted)->{
                manipIO.setVoltage(-0.25);
            },
            ()->false,
            this
        );
    }
    public Command score() {
        return new FunctionalCommand(
            ()->{},
            ()->{
                manipIO.setVoltage(manipScorePower.get());
            },
            (Boolean interrupted)->{
                manipIO.setVoltage(0);
            },
            ()->false,
            this
        );
    }
    public Command hedge() {
        return new FunctionalCommand(
            ()->{},
            ()->{
                manipIO.setVoltage(manipHedgePower.get());
            },
            (Boolean interrupted)->{
                manipIO.setVoltage(0);
            },
            ()->false,
            this
        );
    }
}
