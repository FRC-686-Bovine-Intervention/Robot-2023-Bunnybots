package frc.robot.subsystems.manualOverrides;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DisabledInstantCommand;
import frc.robot.util.VirtualSubsystem;

public class ManualOverrides extends VirtualSubsystem {
    private DigitalInput ArmBrakeModeSwitch = new DigitalInput(Constants.OverrideConstants.armBrakeModeButton);
    private DigitalInput DriveBrakeModeSwitch = new DigitalInput(Constants.OverrideConstants.armBrakeModeButton);

    private Arm armSubsystem;
    private Drive driveSubsystem;

    public ManualOverrides(Arm armSubsystem, Drive driveSubsystem) {
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void periodic() {
        new Trigger(ArmBrakeModeSwitch::get)
            .toggleOnTrue(new DisabledInstantCommand(() -> this.armSubsystem.setBrakeMode(false), new Subsystem[0]))
            .toggleOnFalse(new DisabledInstantCommand(() -> this.armSubsystem.setBrakeMode(true), new Subsystem[0]));

        new Trigger(DriveBrakeModeSwitch::get)
            .toggleOnTrue(new DisabledInstantCommand(() -> this.driveSubsystem.setBrakeMode(false), new Subsystem[0]))
            .toggleOnFalse(new DisabledInstantCommand(() -> this.driveSubsystem.setBrakeMode(true), new Subsystem[0]));
    }
}
