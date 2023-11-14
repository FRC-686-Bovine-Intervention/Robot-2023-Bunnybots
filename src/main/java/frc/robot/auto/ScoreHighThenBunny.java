package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.pathplannerBackport.FollowPathHolonomic;

public class ScoreHighThenBunny extends AutoRoutine {
    private static final RobotState robotState = RobotState.getInstance();
    private static final PathPlannerPath leftStartCloseBurrow = PathPlannerPath.fromPathFile("ALeft Start Close Burrow");
    private static final PathPlannerPath rightStartCloseBurrow = PathPlannerPath.fromPathFile("ARight Start Close Burrow");
    private static final PathPlannerPath rightStartFarBurrow = PathPlannerPath.fromPathFile("ARight Start Far Burrow");
    private static final PathPlannerPath closeDenExit = PathPlannerPath.fromPathFile("AClose Den Exit");
    private static final PathPlannerPath farDenExit = PathPlannerPath.fromPathFile("AFar Den Exit");
    private static final PathPlannerPath closeDenLeftYard = PathPlannerPath.fromPathFile("AClose Den Left Yard");
    private static final PathPlannerPath farDenLeftYard = PathPlannerPath.fromPathFile("AFar Den Left Yard");
    private static final PathPlannerPath closeDenRightYard = PathPlannerPath.fromPathFile("AClose Den Right Yard");
    private static final PathPlannerPath farDenRightYard = PathPlannerPath.fromPathFile("AFar Den Right Yard");
    private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(Constants.DriveConstants.maxDriveSpeedMetersPerSec, 0.46, new ReplanningConfig());
    private enum LeftRight {
        Left(leftStartCloseBurrow.getPoint(0).position),
        Right(rightStartCloseBurrow.getPoint(0).position);
        public final Pose2d startPose;
        LeftRight(Translation2d startTrans) {
            this.startPose = new Pose2d(startTrans, new Rotation2d(Units.degreesToRadians(180)));
        }
    }
    private enum CloseFar {
        Close,
        Far,
    }

    public ScoreHighThenBunny(Drive drive) {
        super(
            "Score High then Bunny",
            List.of(
                new AutoQuestion("Start position",  (a)->new String[]{LeftRight.Left.name(), LeftRight.Right.name()}),
                new AutoQuestion("Burrow Approach", (responses)->{
                    List<String> a = new ArrayList<>();
                    a.add(CloseFar.Close.name());
                    if(responses.get(0).equals(LeftRight.Right.name())) {
                        a.add(CloseFar.Far.name());
                    }
                    return a.toArray(String[]::new);
                }),
                new AutoQuestion("Den Exit",        (a)->new String[]{CloseFar.Close.name(), CloseFar.Far.name()}),
                new AutoQuestion("Yard Side",       (a)->new String[]{LeftRight.Left.name(), LeftRight.Right.name()})
            ),
            (responses)->{
                LeftRight startPosition = LeftRight.valueOf(responses.get(0));
                CloseFar burrow = CloseFar.valueOf(responses.get(1));
                CloseFar den = CloseFar.valueOf(responses.get(2));
                LeftRight yard = LeftRight.valueOf(responses.get(3));

                Command autoCommand = new InstantCommand(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), startPosition.startPose));
                if (startPosition == LeftRight.Left){
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(leftStartCloseBurrow, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                }
                else{
                    if (burrow == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(rightStartCloseBurrow, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(rightStartFarBurrow, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                }
                if (den == CloseFar.Close){
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenExit, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                }
                else{
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenExit, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                }
                if (yard == LeftRight.Left){
                    if (den == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenLeftYard, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenLeftYard, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                }
                else{
                    if (den == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenRightYard, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenRightYard, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                }
                return autoCommand;
            }
        );
    }
}
