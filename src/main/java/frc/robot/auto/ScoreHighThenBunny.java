package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;

public class ScoreHighThenBunny extends AutoRoutine {
    final static PathPlannerPath leftStartCloseBurrow = PathPlannerPath.fromPathFile("leftStartCloseBurrow");
    final static PathPlannerPath rightStartCloseBurrow = PathPlannerPath.fromPathFile("rightStartCloseBurrow");
    final static PathPlannerPath rightStartFarBurrow = PathPlannerPath.fromPathFile("rightStartFarBurrow");
    final static PathPlannerPath closeDenExit = PathPlannerPath.fromPathFile("closeDenExit");
    final static PathPlannerPath farDenExit = PathPlannerPath.fromPathFile("farDenExit");
    final static PathPlannerPath closeDenLeftYard = PathPlannerPath.fromPathFile("closeDenLeftYard");
    final static PathPlannerPath farDenLeftYard = PathPlannerPath.fromPathFile("farDenLeftYard");
    final static PathPlannerPath closeDenRightYard = PathPlannerPath.fromPathFile("closeDenRightYard");
    final static PathPlannerPath farDenRightYard = PathPlannerPath.fromPathFile("farDenRightYard");
    final static HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(Constants.DriveConstants.maxDriveSpeedMetersPerSec, 0.46, new ReplanningConfig());
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
                Command autoCommand = Commands.none();
                LeftRight startPosition = LeftRight.valueOf(responses.get(0));
                CloseFar burrow = CloseFar.valueOf(responses.get(1));
                CloseFar den = CloseFar.valueOf(responses.get(2));
                LeftRight yard = LeftRight.valueOf(responses.get(3));
                if (startPosition == LeftRight.Left){
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(leftStartCloseBurrow, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                }
                else{
                    if (burrow == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(rightStartCloseBurrow, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(rightStartFarBurrow, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                }
                if (den == CloseFar.Close){
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenExit, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                }
                else{
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenExit, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                }
                if (yard == LeftRight.Left){
                    if (den == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenLeftYard, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenLeftYard, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                }
                else{
                    if (den == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenRightYard, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenRightYard, RobotState.getInstance()::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive));
                    }
                }
                return Commands.none();
            }
        );
    }
}
