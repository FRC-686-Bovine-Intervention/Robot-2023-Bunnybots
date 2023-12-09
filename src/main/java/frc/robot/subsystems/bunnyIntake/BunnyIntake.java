// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bunnyIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.LoggedTunableNumber;

public class BunnyIntake extends SubsystemBase {
  private final BunnyIntakeIO bunnyIntakeIO;
  private final BunnyIntakeIOInputsAutoLogged bunnyIntakeIOInputs = new BunnyIntakeIOInputsAutoLogged();

  public static enum BunnyIntakePos {
    Defense (0),
    Score   (0);

    private final LoggedTunableNumber val;
    BunnyIntakePos(double defaultRads) {
      val = new LoggedTunableNumber("BunnyIntake/Positions/" + this.name(), defaultRads);
    }
    public double getRads() {return val.get();}
  }

  private final LoggedTunableNumber bunnyIntakePIDkP =  new LoggedTunableNumber("BunnyIntake/PID/kP",  0);
  private final LoggedTunableNumber bunnyIntakePIDkI =  new LoggedTunableNumber("BunnyIntake/PID/kI",  0);
  private final LoggedTunableNumber bunnyIntakePIDkD =  new LoggedTunableNumber("BunnyIntake/PID/kD",  0);
  private final LoggedTunableNumber bunnyIntakePIDkV =  new LoggedTunableNumber("BunnyIntake/PID/kV",  0);
  private final LoggedTunableNumber bunnyIntakePIDkA =  new LoggedTunableNumber("BunnyIntake/PID/kA",  0);
  private final LoggedTunableNumber bunnyIntakePIDToleranceDeg = new LoggedTunableNumber("BunnyIntake/PID/Tolerance (Deg)",  0);
  private final ProfiledPIDController bunnyIntakePID =  new ProfiledPIDController(
      bunnyIntakePIDkP.get(),
      bunnyIntakePIDkI.get(),
      bunnyIntakePIDkD.get(),
      new Constraints(
          bunnyIntakePIDkV.get(),
          bunnyIntakePIDkA.get()
      )
  );

  private void updateTunables() {
    if(bunnyIntakePIDkP.hasChanged(hashCode()) || bunnyIntakePIDkI.hasChanged(hashCode()) || bunnyIntakePIDkD.hasChanged(hashCode())) {
        bunnyIntakePID.setPID(bunnyIntakePIDkP.get(), bunnyIntakePIDkI.get(), bunnyIntakePIDkD.get());
    }
    if(bunnyIntakePIDkV.hasChanged(hashCode()) || bunnyIntakePIDkA.hasChanged(hashCode())) {
      bunnyIntakePID.setConstraints(new Constraints(bunnyIntakePIDkV.get(), bunnyIntakePIDkA.get()));
    }
    if(bunnyIntakePIDToleranceDeg.hasChanged(hashCode())) {
        bunnyIntakePID.setTolerance(Units.degreesToRadians(bunnyIntakePIDToleranceDeg.get()));
    }
}

  // private final LoggedTunableNumber spikeThreshold =  new LoggedTunableNumber("BunnyIntake/Current Spike Threshold",  +40.0);
  // private final LoggedTunableNumber spikeTime =       new LoggedTunableNumber("BunnyIntake/Current Spike Time",       +0.5);

  // private final Timer currentSpikeTimer = new Timer();

  private static final ShuffleboardTab SBTab = Shuffleboard.getTab("BunnyIntake");
  public BunnyIntake(BunnyIntakeIO bunnyIntakeIO) {
    System.out.println("[Init BunnyIntake] Instantiating BunnyIntake");
    this.bunnyIntakeIO = bunnyIntakeIO;
    System.out.println("[Init BunnyIntake] BunnyIntake IO: " + this.bunnyIntakeIO.getClass().getSimpleName());
    SBTab.add("BunnyIntake Subsystem", this);
    for(BunnyIntakePos pos : BunnyIntakePos.values()) {
      SBTab.add(pos.name(), setBunnyIntakePos(pos));
    }
  }

  @Override
  public void periodic() {
    bunnyIntakeIO.updateInputs(bunnyIntakeIOInputs);
    Logger.processInputs("BunnyIntake", bunnyIntakeIOInputs);
    updateTunables();
  }

  // private final LoggedTunableNumber manualBunnyIntakeVolts = new LoggedTunableNumber("BunnyIntake/Manual Arm Volts", 2);
  // public Command setBunnyIntakeVolts(double dir) {
  //     return new StartEndCommand(
  //         () -> bunnyIntakeIO.setVoltage(manualBunnyIntakeVolts.get() * dir),
  //         () -> bunnyIntakeIO.setVoltage(0),
  //         this
  //     ).withName("Manual Volts: " + (manualBunnyIntakeVolts.get() * dir));
  // }

  public Command waitUntilAtGoal() {
    return new WaitUntilCommand(bunnyIntakePID::atGoal);
  }
  
  public Command setBunnyIntakePos(BunnyIntakePos pos) {
    return new ProfiledPIDCommand(
        bunnyIntakePID,
        () -> bunnyIntakeIOInputs.bunnyIntakePositionRad, pos.getRads(),
        (output, setpoint) -> bunnyIntakeIO.setVoltage(output),
        this
    ).withName(pos.name());
  }

  public Command gotoArmPos(BunnyIntakePos pos) {
    return Commands.runOnce(() -> setBunnyIntakePos(pos).schedule());
  }
  public Command gotoArmPosWithWait(BunnyIntakePos pos) {
    return gotoArmPos(pos).andThen(waitUntilAtGoal());
  }
}
