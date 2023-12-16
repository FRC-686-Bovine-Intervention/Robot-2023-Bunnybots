package frc.robot.auto;

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
import frc.robot.subsystems.bunnyIntake.BunnyIntake;
import frc.robot.subsystems.bunnyIntake.BunnyIntake.BunnyPos;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class ScoreBunny extends AutoRoutine {
    private static final RobotState robotState = RobotState.getInstance();
    
    private static final AutoQuestion<LeftRight> startPositionQuestion = new AutoQuestion<>("Start Position", () -> LeftRight.values());
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
    private static final String pracPath1Format = "A%s Start %s Den Exit";
    private static final String pracPath2Format = "A%s Den %s Yard";
    private static final String pracPath3Format = "A%s Yard Driveaway";

    private static final String compPath1Format = "%s Start %s Den Exit";
    private static final String compPath2Format = "%s Den %s Yard";
    private static final String compPath3Format = "%s Yard Driveaway";


    private enum LeftRight {
        Left,
        Right,
    }

    private enum CloseFar {
        Close,
        Far,
    }

    public ScoreBunny(Drive drive, Arm arm, BunnyIntake bunny) {
        super(
            "Score Bunny",
            List.of(
                startPositionQuestion,
                denExitQuestion,
                yardSideQuestion
            ),
            () -> {
                LeftRight startPosition =  startPositionQuestion.getResponse();
                CloseFar  den           =  denExitQuestion.getResponse();
                LeftRight yard          =  yardSideQuestion.getResponse();

                Function<PathPlannerPath, Command> followPathConstructor = (path) -> new FollowPathHolonomic(path, () -> AllianceFlipUtil.apply(robotState.getPose()), drive::getChassisSpeeds, drive::driveVelocity, config, drive);
                var startToExit = PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath1Format : compPath1Format), startPosition.name(), den.name()));
                var exitToYard =  PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath2Format : compPath2Format), den.name(), yard.name()));
                var yardDriveaway = PathPlannerPath.fromPathFile(String.format((RobotType.getRobot() == RobotType.ROBOT_2023_PRAC ? pracPath3Format : compPath3Format), yard.name()));

                return 
                    Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(startToExit.getPoint(0).position, new Rotation2d(Units.degreesToRadians(-90))))))
                    .andThen(
                        arm.gotoArmPos(ArmPos.Defense),
                        bunny.gotoPosWithWait(BunnyPos.Floor),
                        bunny.gotoPosWithWait(BunnyPos.Inside)
                        .alongWith(
                            Commands.waitSeconds(0.5)
                            .andThen(
                                followPathConstructor.apply(startToExit),
                                followPathConstructor.apply(exitToYard),
                                bunny.gotoPosWithWait(BunnyPos.Bar),
                                followPathConstructor.apply(yardDriveaway),
                                bunny.gotoPosWithWait(BunnyPos.Inside)
                            )
                        )
                    )
                    .withName("Score Bunny");
            }
        );
    }

}
