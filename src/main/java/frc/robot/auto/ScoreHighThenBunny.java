package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.arm.arm.Arm.ArmPos;
import frc.robot.subsystems.arm.manipulator.Manipulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.pathplannerBackport.FollowPathHolonomic;

public class ScoreHighThenBunny extends AutoRoutine {
    private static final RobotState robotState = RobotState.getInstance();

    private static final AutoQuestion<LeftRight> startPositionQuestion = new AutoQuestion<>("Start position", () -> LeftRight.values());
    private static final AutoQuestion<CloseFar> burrowApproachQuestion = new AutoQuestion<>("Burrow Approach", () -> {
        List<CloseFar> a = new ArrayList<>();
        a.add(CloseFar.Close);
        if(startPositionQuestion.getResponse().equals(LeftRight.Right)) {
            a.add(CloseFar.Far);
        }
        return a.toArray(CloseFar[]::new);
    });
    private static final AutoQuestion<CloseFar> denExitQuestion = new AutoQuestion<>("Den Exit", () -> CloseFar.values());
    private static final AutoQuestion<LeftRight> yardSideQuestion = new AutoQuestion<>("Yard Side", () -> LeftRight.values());

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

    public ScoreHighThenBunny(final Drive drive, final Arm arm, final Manipulator manip) {
        super(
            "Score High then Bunny",

            List.of(
                startPositionQuestion,
                burrowApproachQuestion,
                denExitQuestion,
                yardSideQuestion
            ),

            () -> {
                LeftRight startPosition =  startPositionQuestion.getResponse();
                CloseFar  burrow        =  burrowApproachQuestion.getResponse();
                CloseFar  den           =  denExitQuestion.getResponse();
                LeftRight yard          =  yardSideQuestion.getResponse();

                Function<PathPlannerPath, Command> followPathConstructor = (path) -> new FollowPathHolonomic(path, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, config, drive);
                var startToBurrow = PathPlannerPath.fromPathFile(String.format(path1format, startPosition.name(), burrow.name()));
                var burrowToExit =  PathPlannerPath.fromPathFile(String.format(path2format, den.name()));
                var exitToYard =    PathPlannerPath.fromPathFile(String.format(path3format, den.name(), yard.name()));

                return
                    Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), new Pose2d(startToBurrow.getPoint(0).position, new Rotation2d(Units.degreesToRadians(180)))))
                    .andThen(followPathConstructor.apply(startToBurrow))
                    .alongWith(arm.gotoArmPosWithWait(ArmPos.HighFront))
                    .andThen(manip.score().withTimeout(0.5))
                    .andThen(arm.gotoArmPos(ArmPos.Defense))
                    .andThen(followPathConstructor.apply(burrowToExit))
                    .andThen(followPathConstructor.apply(exitToYard));
            }
        );
    }
}
