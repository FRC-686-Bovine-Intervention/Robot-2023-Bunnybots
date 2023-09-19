// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private SwerveModulePosition prevModulePosition;

  private static final LoggedTunableNumber wheelRadius = new LoggedTunableNumber("Drive/Module/WheelRadius");
  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/Module/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/Module/DriveKd");
  private static final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drive/Module/DriveKs");
  private static final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drive/Module/DriveKv");
  private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/Module/TurnKp");
  private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/Module/TurnKd");

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final PIDController turnFeedback =  new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  static {
      wheelRadius.initDefault(Constants.DriveConstants.wheelRadiusMeters);
      driveKp.initDefault(0.1);
      driveKd.initDefault(0.0);
      driveKs.initDefault(0.14456);
      driveKv.initDefault(0.14931);
      turnKp.initDefault(5.0);
      turnKd.initDefault(0.0);
  }

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    prevModulePosition = getPosition();

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Updates inputs and checks tunable numbers. */
  public void periodic() {
    prevModulePosition = getPosition();

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Update controllers if tunable numbers have changed
    if (driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
      driveFeedback.setPID(driveKp.get(), 0.0, driveKd.get());
    }
    if (turnKp.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
      turnFeedback.setPID(turnKp.get(), 0.0, turnKd.get());
    }
    if (driveKs.hasChanged(hashCode()) || driveKv.hasChanged(hashCode())) {
      driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    }
  }

  /**
   * Runs the module with the specified setpoint state. Must be called periodically. Returns the
   * optimized state.
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Run turn controller
    io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius.get();
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    return optimizedState;
  }

  /**
   * Runs the module with the specified voltage while controlling to zero degrees. Must be called
   * periodically.
   */
  public void runCharacterization(double volts) {
    io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
    io.setDriveVoltage(volts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnPositionRad));
  }

  /** Returns the current drive position of the module in radians. */
  public double getPositionRadians() {
    return inputs.drivePositionRad;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius.get();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius.get();
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns change in module position since last tick */
  public SwerveModulePosition getPositionDelta() {
    var currentModulePosition = getPosition();
    return new SwerveModulePosition(currentModulePosition.distanceMeters - prevModulePosition.distanceMeters,
                                    currentModulePosition.angle); 
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the drive wheel radius. */
  public static double getWheelRadius() {
    return wheelRadius.get();
  }

  /** Zeros module encoders. */
  public void zeroEncoders() {
    io.zeroEncoders();
    // need to also reset prevModulePosition because drive is driven by deltas in position
    prevModulePosition = getPosition();
  }

}
