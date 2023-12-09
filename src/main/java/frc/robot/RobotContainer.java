// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.auto.ScoreHighThenBunny;
import frc.robot.commands.DriveWithCustomFlick;
import frc.robot.commands.DriverAutoCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.arm.arm.ArmIO;
import frc.robot.subsystems.arm.arm.ArmIOFalcon;
import frc.robot.subsystems.arm.arm.ArmIOSim;
import frc.robot.subsystems.arm.manipulator.Manipulator;
import frc.robot.subsystems.arm.manipulator.ManipulatorIO;
import frc.robot.subsystems.arm.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.arm.manipulator.ManipulatorIOTalon;
import frc.robot.subsystems.bunnyIntake.BunnyIntake;
import frc.robot.subsystems.bunnyIntake.BunnyIntakeIO;
import frc.robot.subsystems.bunnyIntake.BunnyIntakeIONeo;
import frc.robot.subsystems.bunnyIntake.BunnyIntakeIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO550Falcon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.SwerveJoysticks;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.LedData;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.vision.AprilTagCamera;
import frc.robot.subsystems.vision.AprilTagCameraIO;
import frc.robot.subsystems.vision.AprilTagCameraIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer implements IRobotContainer {
    // Subsystems
    private final Drive drive;
    @SuppressWarnings("unused")
    private final Vision vision;
    private final Arm arm;
    private final Manipulator manip;
    private final BunnyIntake bunnyIntake;
    @SuppressWarnings("unused")
    private final ManualOverrides manuOverrides;
    private final Leds ledSystem;

    private final AutoSelector autoSelector = new AutoSelector("Auto");

    private final Mechanism2d robotSideProfile = new Mechanism2d(3, 2, new Color8Bit(Color.kBlack));

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getRobot());
        switch(RobotType.getRobot()) {
            case ROBOT_2023_COMP:
            case ROBOT_2023_PRAC:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIO550Falcon(DriveModulePosition.FRONT_LEFT),
                    new ModuleIO550Falcon(DriveModulePosition.FRONT_RIGHT),
                    new ModuleIO550Falcon(DriveModulePosition.BACK_LEFT),
                    new ModuleIO550Falcon(DriveModulePosition.BACK_RIGHT)
                );
                // vision = null;
                vision = new Vision(
                    // new AprilTagCamera(Camera.Front.name(), new AprilTagCameraIO() {}),
                    // new AprilTagCamera(Camera.Back.name(), new AprilTagCameraIO() {})
                    new AprilTagCamera(Camera.Front.name(), new AprilTagCameraIOPhotonVision(Camera.Front.hardwareName, Camera.Front.robotToCamera))
                    // new AprilTagCamera(Camera.Back.name(), new AprilTagCameraIOPhotonVision(Camera.Back.hardwareName, Camera.Back.robotToCamera))
                );
                manip = new Manipulator(new ManipulatorIOTalon());
                arm = new Arm(new ArmIOFalcon());
                bunnyIntake = new BunnyIntake(new BunnyIntakeIONeo());
                manuOverrides = new ManualOverrides(arm, drive);
                ledSystem = new Leds(new LedData(
                    manip::hasBall,
                    manip::intaking,
                    () -> drive.getCurrentCommand() != null && drive.getCurrentCommand() == drive.getDefaultCommand(),
                    () -> manuOverrides.armOverridingBrake,
                    () -> manuOverrides.driveOverridingBreak
                ));
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                vision = new Vision();
                manip = new Manipulator(new ManipulatorIOSim());
                bunnyIntake = new BunnyIntake(new BunnyIntakeIOSim());
                arm = new Arm(new ArmIOSim());
                manuOverrides = null;
                ledSystem = null;
            break;
            default:
            case REPLAY:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                );
                vision = new Vision();
                manip = new Manipulator(new ManipulatorIO() {});
                bunnyIntake = new BunnyIntake(new BunnyIntakeIO() {});
                arm = new Arm(new ArmIO() {});
                manuOverrides = null;
                ledSystem = null;
            break;
        }
        

        var armPivot = robotSideProfile.getRoot("Arm Pivot", 1.5 + 0.26670000, +0.90805000).append(new MechanismLigament2d("Root", 0, 90, 0, new Color8Bit(Color.kBlack)));
        armPivot.append(arm.setpointArmLig);
        armPivot.append(arm.measuredArmLig).append(manip.rootLig);

        var superStructureRoot = robotSideProfile.getRoot("Super Structure", 1.5 + 0.34925, +0.15875000);
        var frontBeam = superStructureRoot.append(new MechanismLigament2d("Front Beam", 0.74930000, 90, 5, new Color8Bit(Color.kDarkGray)));
        var topBeam = frontBeam.append(new MechanismLigament2d("Top Beam", 0.16510000, 90, 5, new Color8Bit(Color.kDarkGray)));
        topBeam.append(new MechanismLigament2d("Mid Beam", 0.74930000, 90, 5, new Color8Bit(Color.kDarkGray)));
        topBeam.append(new MechanismLigament2d("Angled Beam", 0.74930000 / Math.sin(Units.degreesToRadians(56.098899)), 56.098899, 5, new Color8Bit(Color.kDarkGray)));

        robotSideProfile.getRoot("Bumper", 1.5 + 0.384175, 0.1143).append(new MechanismLigament2d("Bumper", 0.384175 * 2, 180, 50, new Color8Bit(Color.kBlue)));

        // Configure the button bindings
        System.out.println("[Init RobotContainer] Configuring Button Bindings");
        configureButtonBindings();

        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        // Set up auto routines
        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();

        // Alert if in tuning mode
        if (Constants.tuningMode) {
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        var aCom = new ProxyCommand(() -> {
            var pos =  arm.getTargetPos();
            if(pos == null) return Arm.ArmPos.Defense.goTo(arm);
            switch(pos) {
                case Defense:   return Arm.ArmPos.HighFront.goTo(arm);
                case Ground:    return Arm.ArmPos.Ground.goTo(arm);
                case Hedge:     return Arm.ArmPos.Ground.goTo(arm);
                case HighBack:  return Arm.ArmPos.Defense.goTo(arm);
                case HighFront: return Arm.ArmPos.LowFront.goTo(arm);
                case LowBack:   return Arm.ArmPos.HighBack.goTo(arm);
                case LowFront:  return Arm.ArmPos.Hedge.goTo(arm);
                default:        return Arm.ArmPos.Defense.goTo(arm);
            }
        });
        driveController.a().onTrue(aCom).onFalse(Commands.runOnce(() -> aCom.end(false)));
        var yCom = new ProxyCommand(() -> {
            var pos =  arm.getTargetPos();
            if(pos == null) return Arm.ArmPos.Defense.goTo(arm);
            switch(pos) {
                case Defense:   return Arm.ArmPos.HighBack.goTo(arm);
                case Ground:    return Arm.ArmPos.Hedge.goTo(arm);
                case Hedge:     return Arm.ArmPos.LowFront.goTo(arm);
                case HighBack:  return Arm.ArmPos.LowBack.goTo(arm);
                case HighFront: return Arm.ArmPos.Defense.goTo(arm);
                case LowBack:   return Arm.ArmPos.LowBack.goTo(arm);
                case LowFront:  return Arm.ArmPos.HighFront.goTo(arm);
                default:        return Arm.ArmPos.Defense.goTo(arm);
            }
        });
        driveController.y().onTrue(yCom).onFalse(Commands.runOnce(() -> yCom.end(false)));

        driveController.b().whileTrue(manip.intake());
        driveController.x().whileTrue(manip.score());

        driveController.rightBumper().toggleOnTrue(
            Commands.either(
                DriverAutoCommands.bushToHedge(drive, arm, manip).asProxy().andThen(DriverAutoCommands.hedgeToBush(drive, arm, manip).asProxy()).asProxy().repeatedly(),
                DriverAutoCommands.hedgeToBush(drive, arm, manip).asProxy().andThen(DriverAutoCommands.bushToHedge(drive, arm, manip).asProxy()).asProxy().repeatedly(),
                manip::hasBall
            )
        );
    }


    private void configureSubsystems() {
        drive.setDefaultCommand(
            new DriveWithCustomFlick(
                drive,
                SwerveJoysticks.process(
                    () -> driveController.getLeftY(),  // forward is field +x axis
                    () -> driveController.getLeftX(),  //   right is field +y axis
                    () -> 0,
                    true,           // squareLinearInputs
                    true,             // squareTurnInputs
                    DriveConstants.joystickSlewRateLimit
                ),
                DriveWithCustomFlick.headingFromJoystick(
                    () -> -driveController.getRightX(),
                    () -> -driveController.getRightY(),
                    0.85,
                    () -> {
                        switch (arm.getTargetPos()) {
                            default:        return Units.degreesToRadians(0);
                            case HighBack:
                            case LowBack:   return Units.degreesToRadians(180);
                        }
                    }
                ),
                driveController.leftBumper()
            )
        );
    }

    private void configureAutos() {
        autoSelector.addRoutine(new ScoreHighThenBunny(drive, arm, manip));
        autoSelector.addRoutine(new AutoRoutine(
            "Drive Characterization",
            new ArrayList<>(0),
            () -> new FeedForwardCharacterization(
                drive,
                true,
                new FeedForwardCharacterizationData("drive"),
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity
            )
        ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // AutoRoutine routine = autoChooser.get();
        // drive.setPose(routine.position.getPose());
        // return routine.command;
        return autoSelector.getSelectedAutoCommand();
    }

    public void robotPeriodic() {
        RobotState.getInstance().logOdometry();
        Logger.recordOutput("Mechanism2d/Robot Side Profile", robotSideProfile);
    }
}

