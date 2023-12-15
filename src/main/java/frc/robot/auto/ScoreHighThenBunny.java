package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotType;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.arm.arm.Arm.ArmPos;
import frc.robot.subsystems.arm.manipulator.Manipulator;
import frc.robot.subsystems.bunnyIntake.BunnyIntake;
import frc.robot.subsystems.bunnyIntake.BunnyIntake.BunnyPos;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class ScoreHighThenBunny extends AutoRoutine {
    private static final RobotState robotState = RobotState.getInstance();

    private static final AutoQuestion<LeftRight> startPositionQuestion = new AutoQuestion<>("Start position", () -> LeftRight.values());
    private static final AutoQuestion<CloseFar> burrowApproachQuestion = new AutoQuestion<>("Burrow Approach", () -> {
        // List<CloseFar> a = new ArrayList<>();
        // a.add(CloseFar.Close);
        // if(startPositionQuestion.getResponse().equals(LeftRight.Right)) {
        //     a.add(CloseFar.Far);
        // }
        // return a.toArray(CloseFar[]::new);
        return CloseFar.values();
    });
    private static final AutoQuestion<CloseFar> denExitQuestion = new AutoQuestion<>("Den Exit", () -> CloseFar.values());
    private static final AutoQuestion<LeftRight> yardSideQuestion = new AutoQuestion<>("Yard Side", () -> LeftRight.values());

    private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        new PIDConstants(
            1,
            0,
            0
        ),
        new PIDConstants(
            1.5,
            0,
            0
        ),
        Constants.DriveConstants.maxDriveSpeedMetersPerSec,
        0.46,
        new ReplanningConfig()
    );
    private static final String pracPath1Format = "A%s Start %s Burrow";
    private static final String pracPath2Format = "A%s Den Exit";
    private static final String pracPath3Format = "A%s Den %s Yard";
    private static final String pracPath4Format = "A%s Yard Driveaway";
    private static final String compPath1Format = "%s Start %s Burrow";
    private static final String compPath2Format = "%s Den Exit";
    private static final String compPath3format = "%s Den %s Yard";
    private static final String compPath4format = "%s Yard Driveaway";

    private enum LeftRight {
        Left,
        Right,
    }
    private enum CloseFar {
        Close,
        Far,
    }

    public ScoreHighThenBunny(Drive drive, Arm arm, Manipulator manip, BunnyIntake bunny) {
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

                // startPosition = LeftRight.Left;
                // burrow =        CloseFar.Close;
                // den =           CloseFar.Far;
                // yard =          LeftRight.Left;

                Function<PathPlannerPath, Command> followPathConstructor = (path) -> new FollowPathHolonomic(path, () -> AllianceFlipUtil.apply(robotState.getPose()), drive::getChassisSpeeds, drive::driveVelocity, config, drive);
                var startToBurrow = PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath1Format : compPath1Format), startPosition.name(), burrow.name()));
                var burrowToExit =  PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath2Format : compPath2Format), den.name()));
                var exitToYard =    PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath3Format : compPath3format), den.name(), yard.name()));
                var yardDriveaway = PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath4Format : compPath4format), yard.name()));

                return
                    Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(startToBurrow.getPoint(0).position, new Rotation2d(Units.degreesToRadians(-90))))))
                    .andThen(
                        bunny.gotoPosWithWait(BunnyPos.Floor),
                        bunny.gotoPosWithWait(BunnyPos.Inside)
                        .alongWith(
                            Commands.waitSeconds(0.5)
                            .andThen(
                                followPathConstructor.apply(startToBurrow)
                                .alongWith(arm.gotoArmPosWithWait(ArmPos.HighBack)),
                                manip.score().withTimeout(0.5),
                                arm.gotoArmPos(ArmPos.Defense),
                                followPathConstructor.apply(burrowToExit),
                                followPathConstructor.apply(exitToYard),
                                bunny.gotoPosWithWait(BunnyPos.Bar),
                                followPathConstructor.apply(yardDriveaway),
                                bunny.gotoPosWithWait(BunnyPos.Inside)
                            )
                        )
                    )
                    .withName("Score High Then Bunny");
            }
        );
    }
}
