package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.pathplannerBackport.FollowPathHolonomic;

public class ScoreHighThenBunny extends AutoRoutine {
    private static final RobotState robotState = RobotState.getInstance();

    private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(Constants.DriveConstants.maxDriveSpeedMetersPerSec, 0.46, new ReplanningConfig());
    private static final String path1format = "A%s Start %s Burrow";
    private static final String path2format = "A%s Den Exit";
    private static final String path3format = "A%s Den %s Yard";

    private enum LeftRight {
        Left,
        Right,
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
                LeftRight startPosition =   LeftRight.valueOf(responses.get(0));
                CloseFar burrow =           CloseFar.valueOf(responses.get(1));
                CloseFar den =              CloseFar.valueOf(responses.get(2));
                LeftRight yard =            LeftRight.valueOf(responses.get(3));

                var path1 = PathPlannerPath.fromPathFile(String.format(path1format, startPosition.name(), burrow.name()));
                var path2 = PathPlannerPath.fromPathFile(String.format(path2format, den.name()));
                var path3 = PathPlannerPath.fromPathFile(String.format(path3format, den.name(), yard.name()));

                return
                    new InstantCommand(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), new Pose2d(path1.getPoint(0).position, new Rotation2d(Units.degreesToRadians(180)))))
                    .andThen(new FollowPathHolonomic(path1, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive))
                    .andThen(new PrintCommand("Finished Path 1"))
                    .andThen(new FollowPathHolonomic(path2, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive))
                    .andThen(new PrintCommand("Finished Path 2"))
                    .andThen(new FollowPathHolonomic(path3, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive))
                    .andThen(new PrintCommand("Finished Path 3"));
            }
        );
    }
}
