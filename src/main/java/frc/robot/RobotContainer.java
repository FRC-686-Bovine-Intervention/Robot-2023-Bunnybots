// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.ScoreHighThenBunny;
import frc.robot.commands.DriveWithCustomFlick;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.arm.arm.ArmIOFalcon;
import frc.robot.subsystems.arm.manipulator.Manipulator;
import frc.robot.subsystems.arm.manipulator.ManipulatorIOTalon;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO550Falcon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.SwerveJoysticks;
import frc.robot.subsystems.leds.LEDFrameworkSystem;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.led.functions.Gradient;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    @SuppressWarnings("unused")
    private final Drive drive;
    @SuppressWarnings("unused")
    private final Vision vision = null;//new Vision();
    private final Arm arm;
    private final Manipulator manip;
    @SuppressWarnings("unused")
    private final ManualOverrides manuOverrides;
    private final LEDFrameworkSystem ledSystem;

    private final AutoSelector autoSelector = new AutoSelector("Auto");

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);

    // TODO: add LED and Brake switches
    // private DigitalInput brakeSwitch = new
    // DigitalInput(DIOPorts.brakeSwitchPort);
    // private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.ledSwitchPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.mode) {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIO550Falcon(DriveModulePosition.FRONT_LEFT),
                        new ModuleIO550Falcon(DriveModulePosition.FRONT_RIGHT),
                        new ModuleIO550Falcon(DriveModulePosition.BACK_LEFT),
                        new ModuleIO550Falcon(DriveModulePosition.BACK_RIGHT));

                arm = new Arm(new ArmIOFalcon());
                manip = new Manipulator(new ManipulatorIOTalon());
                manuOverrides = new ManualOverrides(arm, drive);
                ledSystem = null;//new LEDFrameworkSystem();
            break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                arm = null;
                manip = null;
                manuOverrides = null;
                ledSystem = null;
            break;

            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                arm = null;
                manip = null;
                manuOverrides = null;
                ledSystem = null;
            break;
        }

        // Configure the button bindings
        configureButtonBindings();

        configureSubsystems();

        // Set up auto routines
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
        // driveController.a().whileTrue(arm.setTargetPosAndWait(ArmPos.Ground).andThen(manip.intake())).onFalse(arm.setTargetPos(ArmPos.Defense));
        driveController.leftBumper().whileTrue(new FunctionalCommand(
            ()->{},
            ()->{
                if (CommandScheduler.getInstance().requiring(drive) instanceof DriveWithCustomFlick) {
                    ((DriveWithCustomFlick)CommandScheduler.getInstance().requiring(drive)).setPrecision(true);
                }
            },
            (Boolean i)->{
                if (CommandScheduler.getInstance().requiring(drive) instanceof DriveWithCustomFlick) {
                    ((DriveWithCustomFlick)CommandScheduler.getInstance().requiring(drive)).setPrecision(false);
                }
            },
            ()->false
        ));

        driveController.a().whileTrue(arm.setArmVolts(-1));
        driveController.y().whileTrue(arm.setArmVolts(1));

        driveController.b().whileTrue(manip.intake());
        driveController.x().whileTrue(manip.score());
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
                    0.85
                )
            )
        );
    }


    private void configureAutos() {
        autoSelector.addRoutine(new ScoreHighThenBunny(drive));
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

    public void disabledInit() {
    }

    public void disabledPeriodic() {
    }

    public void enabledInit() {
        if (ledSystem != null) {
            ledSystem.playOffboardScrolling(Gradient.rainbow);
        }
    }


}

