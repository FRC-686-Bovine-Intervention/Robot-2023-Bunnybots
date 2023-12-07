package frc.robot.subsystems.manualOverrides;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.VirtualSubsystem;

public class ManualOverrides extends VirtualSubsystem {
    private final Arm arm;
    private final Drive drive;

    private final DigitalInput armBrakeSwitch = new DigitalInput(Constants.OverrideConstants.armBrakeModeButton);
    private boolean lastArmBrakeSwitch = false;
    private final Timer armBrakeHoldTimer = new Timer();
    public Boolean armOverridingBrake = null;

    private final DigitalInput driveBrakeSwitch = new DigitalInput(Constants.OverrideConstants.driveBrakeModeButton);
    private boolean lastDriveBrakeSwitch = false;
    private final Timer driveBrakeHoldTimer = new Timer();
    public Boolean driveOverridingBreak = null;

    public ManualOverrides(Arm arm, Drive drive) {
        System.out.println("[Init ManualOverrides] Instantiating Manual Overrides");
        this.arm = arm;
        this.drive = drive;
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            armOverridingBrake = null;
            driveOverridingBreak = null;
            return;
        }

        var armBrakeSwitchVal = !armBrakeSwitch.get();
        var driveBrakeSwitchVal = !driveBrakeSwitch.get();
        Logger.recordOutput("Manual Overrides/Arm Brake", armBrakeSwitchVal);
        Logger.recordOutput("Manual Overrides/Drive Brake", driveBrakeSwitchVal);

        if(armBrakeSwitchVal && !lastArmBrakeSwitch) {
            armBrakeHoldTimer.restart();
        }
        if(!armBrakeSwitchVal && lastArmBrakeSwitch) {
            armBrakeHoldTimer.stop();
            if(armBrakeHoldTimer.hasElapsed(1)) {
                armOverridingBrake = true;
            } else {
                if(armOverridingBrake == null) {
                    armOverridingBrake = false;
                } else {
                    armOverridingBrake = null;
                }
            }
        }

        if(driveBrakeSwitchVal && !lastDriveBrakeSwitch) {
            driveBrakeHoldTimer.restart();
        }
        if(!driveBrakeSwitchVal && lastDriveBrakeSwitch) {
            driveBrakeHoldTimer.stop();
            if(driveBrakeHoldTimer.hasElapsed(1)) {
                driveOverridingBreak = true;
            } else {
                if(driveOverridingBreak == null) {
                    driveOverridingBreak = false;
                } else {
                    driveOverridingBreak = null;
                }
            }
        }

        arm.setBrakeMode(armOverridingBrake);
        drive.setBrakeMode(driveOverridingBreak);

        lastArmBrakeSwitch = armBrakeSwitchVal;
        lastDriveBrakeSwitch = driveBrakeSwitchVal;
    }
}
