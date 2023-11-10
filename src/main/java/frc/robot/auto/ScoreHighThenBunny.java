package frc.robot.auto;

import java.util.List;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoSelector.AutoRoutine;

public class ScoreHighThenBunny extends AutoRoutine {

    public ScoreHighThenBunny() {
        super(
            "Score High then Bunny",
            Map.of(
                "Start position", new String[]{"Left", "Right"},
                "Start position", new String[]{"Left", "Right"},
            ),

        );
    }
}
