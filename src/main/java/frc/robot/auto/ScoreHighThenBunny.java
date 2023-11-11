package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;

public class ScoreHighThenBunny extends AutoRoutine {
    PathPlannerPath leftStartCloseBurrow = PathPlannerPath.fromPathFile("leftStartCloseBurrow");
    PathPlannerPath rightStartCloseBurrow = PathPlannerPath.fromPathFile("rightStartCloseBurrow");
    PathPlannerPath rightStartFarBurrow = PathPlannerPath.fromPathFile("rightStartFarBurrow");
    PathPlannerPath closeDenExit = PathPlannerPath.fromPathFile("closeDenExit");
    PathPlannerPath farDenExit = PathPlannerPath.fromPathFile("farDenExit");
    PathPlannerPath closeDenLeftYard = PathPlannerPath.fromPathFile("closeDenLeftYard");
    PathPlannerPath farDenLeftYard = PathPlannerPath.fromPathFile("farDenLeftYard");
    PathPlannerPath closeDenRightYard = PathPlannerPath.fromPathFile("closeDenRightYard");
    PathPlannerPath farDenRightYard = PathPlannerPath.fromPathFile("farDenRightYard");
    private enum LeftRight {
        Left,
        Right,
    }
    private enum CloseFar {
        Close,
        Far,
    }

    public ScoreHighThenBunny() {
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
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(leftStartCloseBurrow));
                }
                else{
                    if (burrow == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(rightStartCloseBurrow));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(rightStartFarBurrow));
                    }
                }
                if (den == CloseFar.Close){
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenExit));
                }
                else{
                    autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenExit));
                }
                if (yard == LeftRight.Left){
                    if (den == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenLeftYard));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenLeftYard));
                    }
                }
                else{
                    if (den == CloseFar.Close){
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(closeDenRightYard));
                    }
                    else{
                        autoCommand = autoCommand.andThen(new FollowPathHolonomic(farDenRightYard));
                    }
                }
                return Commands.none();
            }
        );
    }
}
