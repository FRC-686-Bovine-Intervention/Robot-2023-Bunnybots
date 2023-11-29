package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveJoysticks.ProcessedJoysticks;

public class DriveWithCustomFlick extends CommandBase {

	private final Drive drive;
    private final Supplier<ProcessedJoysticks> joystickSupplier;
	private final Supplier<Optional<Double>> headingSupplier; // rotation
	private final BooleanSupplier precisionSupplier; // slow-down for precision positioning

	private double desiredHeadingRadians;
	private final double headingKp = 0.3 /* / DriveConstants.maxTurnRateRadiansPerSec */;
	private final double headingKi = 0;
	private final double headingKd = 0;
	private final double headingTolerance = Units.degreesToRadians(1.0);
	private final PIDController headingPID;

	public static Supplier<Optional<Double>> headingFromJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, double radialDeadband) {
		return new Supplier<Optional<Double>>() {
			private final Timer preciseTurnTimer = new Timer();
			private final double preciseTurnTimeThreshold = 0.5;
			private final double[] snapPoints = new double[]{
				Units.degreesToRadians(0),
				Units.degreesToRadians(90),
				Units.degreesToRadians(135),
				Units.degreesToRadians(180),
				Units.degreesToRadians(270),
			};
			@Override
			public Optional<Double> get() {
				double joyX = xSupplier.getAsDouble();
				double joyY = ySupplier.getAsDouble();
				if(Math.hypot(joyX, joyY) <= radialDeadband) {
					preciseTurnTimer.stop();
					preciseTurnTimer.reset();
					return Optional.empty();
				} else {
					preciseTurnTimer.start();
				}
				double joyHeading = -Math.atan2(-joyX, joyY);
				if (preciseTurnTimer.hasElapsed(preciseTurnTimeThreshold)) {
					return Optional.of(joyHeading);
				}
				double[] distancesToSnapPoints = new double[snapPoints.length];
				for(int i = 0; i < snapPoints.length; i++) {
					double snapPoint = snapPoints[i]/*  + (DriverStation.getAlliance() == Alliance.Red ? 180 : 0) */;
					double distanceToPoint = Math.abs(joyHeading - snapPoint);
					double distanceToPointWithPosRot = Math.abs(joyHeading - (snapPoint + 2 * Math.PI));
					double distanceToPointWithNegRot = Math.abs(joyHeading - (snapPoint - 2 * Math.PI));
					distancesToSnapPoints[i] = Math.min(Math.min(distanceToPoint, distanceToPointWithPosRot), distanceToPointWithNegRot);
				}
				int smallestIndex = 0;
				for(int i = 1; i < distancesToSnapPoints.length; i++) {
					if(distancesToSnapPoints[i] < distancesToSnapPoints[smallestIndex]) {
						smallestIndex = i;
					}
				}
				return Optional.of(snapPoints[smallestIndex]);
			}
		};
	}

  	/** Creates a new DriveWithJoysticks. */
	public DriveWithCustomFlick(Drive drive, Supplier<ProcessedJoysticks> joystickSupplier, Supplier<Optional<Double>> headingSupplier, BooleanSupplier precisionSupplier) {
		addRequirements(drive);
		this.drive = drive;
        this.joystickSupplier = joystickSupplier;
        this.headingSupplier = headingSupplier;
		this.precisionSupplier = precisionSupplier;

		headingPID = new PIDController(headingKp, headingKd, headingKi);
		headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
		headingPID.setTolerance(headingTolerance);
	}

	@Override
	public void initialize() {
		desiredHeadingRadians = drive.getPose().getRotation().getRadians();
	}

	@Override
	public void execute() {

        ProcessedJoysticks processedJoysticks = joystickSupplier.get();

		// update desired direction
		Optional<Double> desiredHeading = headingSupplier.get();
		if (desiredHeading.isPresent()) {
			desiredHeadingRadians = desiredHeading.get();
		}

		// PID control of turn
		double turnInput = headingPID.calculate(drive.getPose().getRotation().getRadians(), desiredHeadingRadians);
		turnInput = headingPID.atSetpoint() ? 0 : turnInput;
		turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);

        // Convert to meters/sec and radians/sec
        double vxMetersPerSecond = processedJoysticks.getX() * drive.getMaxLinearSpeedMetersPerSec();
        double vyMetersPerSecond = processedJoysticks.getY() * drive.getMaxLinearSpeedMetersPerSec();
        double omegaRadiansPerSecond = turnInput * drive.getMaxAngularSpeedRadiansPerSec();

        if (precisionSupplier.getAsBoolean()) {
            vxMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            vyMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            omegaRadiansPerSecond *= DriveConstants.precisionTurnMulitiplier;
        }

		// robot relative controls
		ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

		// field relative controls
		var driveRotation = drive.getRotation(); // angle from alliance wall normal
		if (DriverStation.getAlliance() == Alliance.Red) {
			driveRotation = driveRotation.rotateBy(new Rotation2d(Math.PI));
		}
		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driveRotation);

		drive.driveVelocity(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}
