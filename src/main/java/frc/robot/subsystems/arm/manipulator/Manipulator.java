package frc.robot.subsystems.arm.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO manipIO;
    private final ManipulatorIOInputsAutoLogged manipIOInputs = new ManipulatorIOInputsAutoLogged();

    public final MechanismLigament2d rootLig =              new MechanismLigament2d("Root",             0, 0, 0, new Color8Bit(Color.kBlack));
    private final MechanismLigament2d clawLig =             new MechanismLigament2d("Claw",             0.325425, 0, 5, new Color8Bit(Color.kWhiteSmoke));
    // private final MechanismLigament2d setpointClawLig =     new MechanismLigament2d("Claw",             0.325425, 0, 5, new Color8Bit(Color.kLightGreen));

    private final double rollerDistance = 0.02;
    private final MechanismLigament2d topRollerFixedLig =   new MechanismLigament2d("Top Roller Fixed", Math.hypot(rollerDistance, 0.101600/2), Units.radiansToDegrees(Math.atan2(rollerDistance, -0.101600/2)), 0, new Color8Bit(Color.kBlack));
    private final MechanismLigament2d topRollerLig =        new MechanismLigament2d("Top Roller",       0.101600, -Units.radiansToDegrees(Math.atan2(rollerDistance, -0.101600/2)), 5, new Color8Bit(Color.kGray));

    private final MechanismLigament2d botRollerFixedLig =   new MechanismLigament2d("Bot Roller Fixed", Math.hypot(rollerDistance, 0.101600/2), -Units.radiansToDegrees(Math.atan2(rollerDistance, -0.101600/2)), 0, new Color8Bit(Color.kBlack));
    private final MechanismLigament2d botRollerLig =        new MechanismLigament2d("Bot Roller",       0.101600, Units.radiansToDegrees(Math.atan2(rollerDistance, -0.101600/2)), 5, new Color8Bit(Color.kGray));
    
    private final MechanismLigament2d ballFixedLig =        new MechanismLigament2d("Ball Fixed",       0.2, 0, 0, new Color8Bit(Color.kBlack));
    private final MechanismLigament2d ballLig =             new MechanismLigament2d("Ball",             0, 0, 100, new Color8Bit(Color.kRed));

    private final LoggedTunableNumber manipIntakePower =    new LoggedTunableNumber("Manipulator/Intake Power",     -0.75);
    private final LoggedTunableNumber manipScorePower =     new LoggedTunableNumber("Manipulator/Scoring Power",    +0.75);
    private final LoggedTunableNumber manipHedgePower =     new LoggedTunableNumber("Manipulator/Hedge Power",      +0.5);

    private final LoggedTunableNumber spikeThreshold =  new LoggedTunableNumber("Manipulator/Current Spike Threshold",  +5.0);
    private final LoggedTunableNumber spikeTime =       new LoggedTunableNumber("Manipulator/Current Spike Time",       +0.5);

    private final Timer currentSpikeTimer = new Timer();
    private boolean hasBall = false;

    public Manipulator(ManipulatorIO manipIO) {
        this.manipIO = manipIO;

        clawLig.append(topRollerFixedLig).append(topRollerLig);
        clawLig.append(botRollerFixedLig).append(botRollerLig);

        rootLig.append(clawLig);
        rootLig.append(ballFixedLig).append(ballLig);
    }

    @Override
    public void periodic() {
        manipIO.updateInputs(manipIOInputs);
        Logger.getInstance().processInputs("Manip", manipIOInputs);
        if(Math.abs(manipIOInputs.manipCurrentAmps) >= spikeThreshold.get()) {
            currentSpikeTimer.start();
        } else {
            currentSpikeTimer.stop();
            currentSpikeTimer.reset();
        }

        ballLig.setLineWeight(hasBall ? 50 : 0);
        if(manipIOInputs.manipAppliedVolts < -3) {
            topRollerLig.setColor(new Color8Bit(Color.kGreen));
            botRollerLig.setColor(new Color8Bit(Color.kGreen));
        } else if (manipIOInputs.manipAppliedVolts > 3) {
            topRollerLig.setColor(new Color8Bit(Color.kRed));
            botRollerLig.setColor(new Color8Bit(Color.kRed));
        } else {
            topRollerLig.setColor(new Color8Bit(Color.kGray));
            botRollerLig.setColor(new Color8Bit(Color.kGray));
        }
    }

    public Command intake() {
        return new FunctionalCommand(
            ()->{},
            ()->{
                manipIO.setVoltage(manipIntakePower.get());
            },
            (Boolean interrupted)->{
                if(!interrupted) {
                    hasBall = true;
                    manipIO.setVoltage(-0.25);
                } else {
                    manipIO.setVoltage(0);
                }
            },
            ()->currentSpikeTimer.hasElapsed(spikeTime.get()),
            this
        );
    }
    public Command score() {
        return new FunctionalCommand(
            ()->{},
            ()->{
                hasBall = false;
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
                hasBall = false;
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
