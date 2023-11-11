package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;

public class ScoreHighThenBunny extends AutoRoutine {
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

                return Commands.none();
            }
        );
    }
}
