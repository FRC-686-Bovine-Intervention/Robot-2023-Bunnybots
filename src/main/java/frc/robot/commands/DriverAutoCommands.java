package frc.robot.commands;

import java.util.function.BiFunction;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.arm.arm.Arm.ArmPos;
import frc.robot.subsystems.arm.manipulator.Manipulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

public class DriverAutoCommands {
    private static final LoggedTunableNumber tP = new LoggedTunableNumber("AutoDrive/tP", 1);
    private static final LoggedTunableNumber tI = new LoggedTunableNumber("AutoDrive/tI", 0);
    private static final LoggedTunableNumber tD = new LoggedTunableNumber("AutoDrive/tD", 0);
    private static final LoggedTunableNumber rP = new LoggedTunableNumber("AutoDrive/rP", 1.5);
    private static final LoggedTunableNumber rI = new LoggedTunableNumber("AutoDrive/rI", 0);
    private static final LoggedTunableNumber rD = new LoggedTunableNumber("AutoDrive/rD", 0);
    private static final Supplier<HolonomicPathFollowerConfig> configSup = () -> {
        return new HolonomicPathFollowerConfig(
            new PIDConstants(
                tP.get(),
                tI.get(),
                tD.get()
            ),
            new PIDConstants(
                rP.get(),
                rI.get(),
                rD.get()
            ),
            Constants.DriveConstants.maxDriveSpeedMetersPerSec,
            0.46,
            new ReplanningConfig()
        );
    };
    // private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(Constants.DriveConstants.maxDriveSpeedMetersPerSec, 0.46, new ReplanningConfig());
    private static final BiFunction<PathPlannerPath, Drive, Command> followPathConstructor = (path, drive) -> new FollowPathHolonomic(path, () -> AllianceFlipUtil.apply(RobotState.getInstance().getPose()), drive::getChassisSpeeds, drive::driveVelocity, configSup.get(), drive);

    private static final PathPlannerPath hedgeToBushPath = PathPlannerPath.fromPathFile("Hedge to Bush");
    private static final PathPlannerPath bushToHedgePath = PathPlannerPath.fromPathFile("Bush to Hedge");

    public static Command hedgeToBush(Drive drive, Arm arm, Manipulator manip) {
        return
            followPathConstructor.apply(hedgeToBushPath, drive)
            .alongWith(arm.gotoArmPosWithWait(ArmPos.LowBack))
            .andThen(manip.intake())
            // .andThen(arm.gotoArmPos(ArmPos.Defense))
            .withName("Hedge To Bush")
            ;
    }

    public static Command bushToHedge(Drive drive, Arm arm, Manipulator manip) {
        return
            followPathConstructor.apply(bushToHedgePath, drive)
            .deadlineWith(
                Commands.waitSeconds(1.5)
                .andThen(arm.gotoArmPosWithWait(ArmPos.Defense))
            )
            .andThen(arm.gotoArmPosWithWait(ArmPos.Hedge))
            .andThen(manip.hedge().withTimeout(0.5))
            .andThen(arm.gotoArmPos(ArmPos.Defense))
            .withName("Bush To Hedge")
            ;
    }
}
