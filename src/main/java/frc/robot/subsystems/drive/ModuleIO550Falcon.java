package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;

public class ModuleIO550Falcon implements ModuleIO {
    private final TalonFX  driveMotor;
    private final CANSparkMax turnMotor;
    private final AbsoluteEncoder turnAbsoluteEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final double initialOffsetRadians;
    private final InvertedValue driveInverted;

    public ModuleIO550Falcon(DriveModulePosition position) {
        driveMotor = new TalonFX(position.driveMotorID, CANDevices.driveCanBusName);
        turnMotor = new CANSparkMax(position.turnMotorID, MotorType.kBrushless);
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        turnRelativeEncoder = turnMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        driveInverted = position.driveInverted;
        initialOffsetRadians = Units.rotationsToRadians(position.cancoderOffsetRotations);

        /** Configure Drive Motors */
        var driveConfig = new TalonFXConfiguration();
        // change factory defaults here
        driveConfig.MotorOutput.Inverted = driveInverted;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 70.0;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);

        /** Configure Turn Motors */
        turnMotor.setInverted(false);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(40);

        setFramePeriods(driveMotor, true);

        zeroEncoders();
    }

    public void updateInputs(ModuleIOInputs inputs) {

        inputs.drivePositionRad =       Units.rotationsToRadians(driveMotor.getPosition().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.driveAppliedVolts =      driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps =       driveMotor.getSupplyCurrent().getValue();

        inputs.turnPositionRad =        MathUtil.angleModulus(Units.rotationsToRadians(turnRelativeEncoder.getPosition())) - initialOffsetRadians;
        inputs.turnVelocityRadPerSec =  Units.rotationsToRadians(turnRelativeEncoder.getVelocity());
        inputs.turnAppliedVolts =       turnMotor.getAppliedOutput();
        inputs.turnCurrentAmps =        turnMotor.getOutputCurrent();
    }

    public void zeroEncoders() {
        driveMotor.setRotorPosition(0.0);
        turnRelativeEncoder.setPosition(turnAbsoluteEncoder.getPosition());
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setControl(new DutyCycleOut(volts / 12));
    }

    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        // reduce rates of most status frames

        // TODO: revisit figuring out what getters to slow down
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        // if (!needMotorSensor) {
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        // }
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);

        talon.getPosition().setUpdateFrequency(Constants.loopFrequencyHz);
    }
}