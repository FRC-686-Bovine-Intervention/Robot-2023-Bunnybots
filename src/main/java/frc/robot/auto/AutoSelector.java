package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.VirtualSubsystem;

public class AutoSelector extends VirtualSubsystem {

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
    }

    public static class AutoRoutine {
        public final String name;
        public final Map<String, List<String>> questions;
        public final Function<List<String>, ? extends Command> autoCommandGenerator;

        public AutoRoutine(String name, Map<String, List<String>> questions, Function<List<String>, ? extends Command> autoCommandGenerator) {
            this.name = name;
            this.questions = questions;
            this.autoCommandGenerator = autoCommandGenerator;
        }
    }
}
