// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bunnyIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class BunnyIntake extends SubsystemBase {
  private final BunnyIntakeIO bunnyIntakeIO;
  private final BunnyIntakeIOInputsAutoLogged bunnyIntakeIOInputs = new BunnyIntakeIOInputsAutoLogged();

  private final LoggedTunableNumber spikeThreshold =    new LoggedTunableNumber("BunnyIntake/Current Spike Threshold",  +10.0);
  private final LoggedTunableNumber spikeTime =         new LoggedTunableNumber("BunnyIntake/Current Spike Time",       +0.5);

  private final Timer currentSpikeTimer = new Timer();

  private static final ShuffleboardTab SBTab = Shuffleboard.getTab("BunnyIntake");
  public BunnyIntake(BunnyIntakeIO bunnyIntakeIO) {
    System.out.println("[Init BunnyIntake] Instantiating BunnyIntake");
    this.bunnyIntakeIO = bunnyIntakeIO;
    System.out.println("[Init BunnyIntake] BunnyIntake IO: " + this.bunnyIntakeIO.getClass().getSimpleName());
    SBTab.add("BunnyIntake Subsystem", this);
  }

  @Override
  public void periodic() {
    bunnyIntakeIO.updateInputs(bunnyIntakeIOInputs);
    Logger.processInputs("BunnyIntake", bunnyIntakeIOInputs);
    if (Math.abs(bunnyIntakeIOInputs.bunnyIntakeCurrentAmps) >= spikeThreshold.get()) {
      currentSpikeTimer.start();
    } else {
      currentSpikeTimer.stop();
      currentSpikeTimer.restart();
    }
  }

  private final LoggedTunableNumber manualBunnyIntakeVolts = new LoggedTunableNumber("BunnyIntake/Manual Arm Volts", 2);
  public Command setBunnyIntakeVolts(double dir) {
      return new StartEndCommand(
          () -> bunnyIntakeIO.setVoltage(manualBunnyIntakeVolts.get() * dir),
          () -> bunnyIntakeIO.setVoltage(0),
          this
      ).withName("Manual Volts: " + (manualBunnyIntakeVolts.get() * dir));
  }

  public Command lower() {
    return new FunctionalCommand(
      ()->{},
      ()->{
          bunnyIntakeIO.setVoltage(manualBunnyIntakeVolts.get());
      },
      (Boolean interrupted)->{
          bunnyIntakeIO.setVoltage(0);
      },
      ()->currentSpikeTimer.hasElapsed(spikeTime.get()),
      this
    );
  }

  public Command rise() {
    return new FunctionalCommand(
      ()->{},
      ()->{
          bunnyIntakeIO.setVoltage(-1 * manualBunnyIntakeVolts.get());
      },
      (Boolean interrupted)->{
          bunnyIntakeIO.setVoltage(0);
      },
      ()->currentSpikeTimer.hasElapsed(spikeTime.get()),
      this
    );
  }
}
