// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bunnyIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class BunnyIntake extends SubsystemBase {
  private final BunnyIntakeIO bunnyIntakeIO;
  private final BunnyIntakeIOInputsAutoLogged bunnyIntakeIOInputs = new BunnyIntakeIOInputsAutoLogged();

  private final ProfiledPIDController pid =  new ProfiledPIDController(
    10,
    0,
    0,
    new Constraints(
      6,
      15
    )
  );
  // private final Mechanism2d bunnyIntakeMech = new Mechanism2d(1, 1, new Color8Bit(Color.kBlack));
  public final MechanismLigament2d measuredBunnyIntakeLig = new MechanismLigament2d("BunnyIntake Measured", Units.inchesToMeters(7.604), 0, 5, new Color8Bit(Color.kPink));

  public BunnyIntake(BunnyIntakeIO bunnyIntakeIO) {
    System.out.println("[Init BunnyIntake] Instantiating BunnyIntake");
    this.bunnyIntakeIO = bunnyIntakeIO;
    System.out.println("[Init BunnyIntake] BunnyIntake IO: " + this.bunnyIntakeIO.getClass().getSimpleName());
    pid.setTolerance(Units.degreesToRadians(5));
    // var mechRoot = bunnyIntakeMech.getRoot("Pivot", 0, 0).append(new MechanismLigament2d("Fixed", 0, 90, 0, new Color8Bit(Color.kBlack)));
    // mechRoot.append(measuredBunnyIntakeLig.append(new MechanismLigament2d("BunnyIntake Extension", Units.inchesToMeters(7.748), 45, 5, new Color8Bit(Color.kLightPink))));
  }

  @Override
  public void periodic() {
    bunnyIntakeIO.updateInputs(bunnyIntakeIOInputs);
    Logger.processInputs("BunnyIntake", bunnyIntakeIOInputs);
    measuredBunnyIntakeLig.setAngle(-Units.radiansToDegrees(bunnyIntakeIOInputs.bunnyIntakePositionRad));
  }

  public static enum BunnyPos {
    Inside(0),
    Floor(3.74),
    Bar(3.91),
    ;
    public final double pos;
    BunnyPos(double pos) {
      this.pos = pos;
    }
  }

  public Command setVolts(double volts) {
    return Commands.runEnd(() -> bunnyIntakeIO.setVoltage(volts), () -> bunnyIntakeIO.setVoltage(0), this);
  }

  public Command setPos(BunnyPos pos) {
    return new ProfiledPIDCommand(
      pid,
      () -> bunnyIntakeIOInputs.bunnyIntakePositionRad, pos.pos,
      (output, setpoint) -> bunnyIntakeIO.setVoltage(output),
      this
    ).withName(pos.name());
  }

  public Command gotoPos(BunnyPos pos) {
    return Commands.runOnce(() -> setPos(pos).schedule());
  }
  public Command gotoPosWithWait(BunnyPos pos) {
    return gotoPos(pos).andThen(waitUntilAtGoal());
  }

  public Command waitUntilAtGoal() {
    return new WaitUntilCommand(pid::atGoal);
  }

  public void calibrate() {
    bunnyIntakeIO.zeroEncoders();
  }
}
