package frc.robot.subsystems.leds;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.EndgameNotificationAnim;
import frc.robot.util.led.animation.EndgameTimerAnimation;
import frc.robot.util.led.animation.FillAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;
import frc.robot.util.led.strips.hardware.CANdleStrip;

public class Leds extends VirtualSubsystem {
    private final LEDManager ledManager = LEDManager.getInstance();
    private final CANdle m_candle = new CANdle(Constants.CANDevices.candleCanID, "rio");
    private final CANdleStrip candleLEDs =  new CANdleStrip(m_candle, 60*2);
    private final LEDStrip onboardLEDs =    candleLEDs.getOnboardLEDs();
    private final LEDStrip offboardLEDs =   candleLEDs.getOffboardLEDs();
    private final LEDStrip rightStrip =     offboardLEDs.substrip(0, offboardLEDs.getLength() / 2);
    private final LEDStrip leftStrip =      offboardLEDs.substrip(offboardLEDs.getLength() / 2).reverse();
    private final LEDStrip parallelStrip =  rightStrip.parallel(leftStrip);

    private final LEDAnimation hasBallAnimation = new FillAnimation(Color.kGreen, parallelStrip);
    private final LEDAnimation intakingAnimation = new FillAnimation(Color.kPurple, parallelStrip);
    private final LEDAnimation endgameNotification = new EndgameTimerAnimation(parallelStrip);
    private final ScrollingAnimation robotAutonomousAnimation = new ScrollingAnimation(new BasicGradient(InterpolationStyle.Step, Color.kRed, Color.kBlue), TilingFunction.Modulo, parallelStrip);
    private final LEDAnimation driverStationConnected = new FillAnimation(() -> (DriverStation.isDSAttached() ? Color.kGreen : Color.kOrange), parallelStrip.substrip(0, 10));

    private final ScrollingAnimation allianceColorAnimation = new ScrollingAnimation((x) -> {
        var colors = new Color[]{
            (DriverStation.getAlliance().isEmpty() ? Color.kRed : Color.kBlack),
            (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Color.kRed : Color.kFirstBlue)
        };
        return InterpolationStyle.Linear.interpolate(x, colors);
    }, TilingFunction.Sinusoidal, parallelStrip);

    private final LEDStrip armManualStrip = parallelStrip.substrip(55);
    private final LEDAnimation armCoast = new FillAnimation(Color.kGreen, armManualStrip);
    private final LEDAnimation armBrake = new FillAnimation(Color.kOrange, armManualStrip);

    private final LEDAnimation onboardBlank = new FillAnimation(Color.kBlack, onboardLEDs);
    private final LEDAnimation driveCoast = new FillAnimation(Color.kGreen, onboardLEDs);
    private final LEDAnimation driveBrake = new FillAnimation(Color.kOrange, onboardLEDs);

    private final AnimationRunner[] runners;

    public Leds(LedData data) {
        System.out.println("[Init Leds] Instantiating Leds");
        ledManager.register(candleLEDs);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configFactoryDefault();
        m_candle.clearAnimation(0);
        m_candle.configAllSettings(configAll, 100);

        this.runners = new AnimationRunner[]{
            new AnimationRunner(data.hasBall, hasBallAnimation),
            new AnimationRunner(data.intaking, intakingAnimation),
            new AnimationRunner(() -> Boolean.FALSE.equals(data.armManual.get()), armCoast),
            new AnimationRunner(() -> Boolean.TRUE.equals(data.armManual.get()), armBrake),
            new AnimationRunner(() -> Boolean.FALSE.equals(data.driveManual.get()), driveCoast),
            new AnimationRunner(() -> Boolean.TRUE.equals(data.driveManual.get()), driveBrake),
            new AnimationRunner(() -> DriverStation.isTeleopEnabled() /* && DriverStation.getMatchType() != MatchType.None */ && DriverStation.getMatchTime() <= 30, endgameNotification),
            new AnimationRunner(data.auto, robotAutonomousAnimation),
            new AnimationRunner(DriverStation::isDisabled, driverStationConnected),
        };

        allianceColorAnimation.setWavelength(4);
        robotAutonomousAnimation.setWavelength(4);
        robotAutonomousAnimation.setVelocity(2);

        allianceColorAnimation.setPriority(0);
        onboardBlank.setPriority(0);
        driverStationConnected.setPriority(1);
        armCoast.setPriority(2);
        armBrake.setPriority(2);
        driveCoast.setPriority(2);
        driveBrake.setPriority(2);
        intakingAnimation.setPriority(3);
        hasBallAnimation.setPriority(4);
        robotAutonomousAnimation.setPriority(5);
        endgameNotification.setPriority(10);

        allianceColorAnimation.start();
        onboardBlank.start();
    }

    @Override
    public void periodic() {
        for(AnimationRunner runner : runners) {
            runner.update();
        }
        ledManager.runLEDs();
    }

    public static class LedData {
        public final BooleanSupplier hasBall;
        public final BooleanSupplier intaking;
        public final BooleanSupplier auto;
        public final Supplier<Boolean> armManual;
        public final Supplier<Boolean> driveManual;
        public LedData(BooleanSupplier hasBall, BooleanSupplier intaking, BooleanSupplier auto, Supplier<Boolean> armManual, Supplier<Boolean> driveManual) {
            this.hasBall = hasBall;
            this.intaking = intaking;
            this.auto = auto;
            this.armManual = armManual;
            this.driveManual = driveManual;
        }
    }

    public static class AnimationRunner {
        private final BooleanSupplier runAnimationSupplier;
        private final LEDAnimation animation;
        private boolean lastVal;

        public AnimationRunner(BooleanSupplier runAnimationSupplier, LEDAnimation animation) {
            this.runAnimationSupplier = runAnimationSupplier;
            this.animation = animation;
        }

        public void update() {
            var curVal = runAnimationSupplier.getAsBoolean();
            if(curVal ^ lastVal) {
                if(curVal) {
                    animation.start();
                } else {
                    animation.stop();
                }
            }
            lastVal = curVal;
        }
    }
}
