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
import frc.robot.Robot;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.EndgameTimerAnimation;
import frc.robot.util.led.animation.FillAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;
import frc.robot.util.led.strips.hardware.SimulatedStrip;
import frc.robot.util.led.strips.hardware.CANdleStrip;

public class Leds extends VirtualSubsystem {
    private final LEDManager ledManager = LEDManager.getInstance();
    private final LEDStrip onboardLEDs;
    private final LEDStrip offboardLEDs;

    private final AnimationRunner[] runners;

    public Leds(LedData data) {
        System.out.println("[Init Leds] Instantiating Leds");
        if(Robot.isReal()) {
            var m_candle = new CANdle(Constants.CANDevices.candleCanID, "rio");
            var candleStrip = new CANdleStrip(m_candle, 60*2);

            ledManager.register(candleStrip);

            onboardLEDs = candleStrip.getOnboardLEDs();
            offboardLEDs = candleStrip.getOffboardLEDs();
            
            CANdleConfiguration configAll = new CANdleConfiguration();
            configAll.statusLedOffWhenActive = true;
            configAll.disableWhenLOS = false;
            configAll.stripType = LEDStripType.GRB;
            configAll.brightnessScalar = 0.5;
            configAll.vBatOutputMode = VBatOutputMode.Modulated;
            m_candle.configFactoryDefault();
            m_candle.clearAnimation(0);
            m_candle.configAllSettings(configAll, 100);
        } else {
            var m_addressable = new SimulatedStrip(0, 60*2+8);

            ledManager.register(m_addressable);

            onboardLEDs = m_addressable.substrip(0, 8);
            offboardLEDs = m_addressable.substrip(8);
        }

        var rightStrip =    offboardLEDs.substrip(0, offboardLEDs.getLength() / 2);
        var leftStrip =     offboardLEDs.substrip(offboardLEDs.getLength() / 2).reverse();
        var parallelStrip = rightStrip.parallel(leftStrip);

        var hasBallAnimation = new FillAnimation(Color.kGreen, parallelStrip);
        var intakingAnimation = new ScrollingAnimation(new BasicGradient(InterpolationStyle.Linear, Color.kRed, Color.kYellow), TilingFunction.Sinusoidal, parallelStrip.substrip(0, 30).parallel(parallelStrip.substrip(30).reverse()));
        var endgameNotification = new EndgameTimerAnimation(parallelStrip);
        var robotAutonomousAnimation = new ScrollingAnimation(new BasicGradient(InterpolationStyle.Step, Color.kRed, Color.kBlue), TilingFunction.Modulo, parallelStrip);
        var driverStationConnected = new FillAnimation(() -> (DriverStation.isDSAttached() ? Color.kGreen : Color.kOrange), parallelStrip.substrip(0, 10));

        var allianceColorAnimation = new ScrollingAnimation((x) -> {
            var colors = new Color[]{
                (DriverStation.getAlliance().isEmpty() ? Color.kRed : Color.kBlack),
                (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Color.kRed : Color.kFirstBlue)
            };
            return InterpolationStyle.Linear.interpolate(x, colors);
        }, TilingFunction.Sinusoidal, parallelStrip);

        var armManualStrip = parallelStrip.substrip(55);
        var armCoast = new FillAnimation(Color.kGreen, armManualStrip);
        var armBrake = new FillAnimation(Color.kOrange, armManualStrip);
        var onboardBlank = new FillAnimation(Color.kBlack, onboardLEDs);
        var driveCoast = new FillAnimation(Color.kGreen, onboardLEDs);
        var driveBrake = new FillAnimation(Color.kOrange, onboardLEDs);


        var autoroutineStrip = parallelStrip.substrip(10, 13);
        var question1Strip = parallelStrip.substrip(13, 16);
        var question2Strip = parallelStrip.substrip(16, 19);
        var question3Strip = parallelStrip.substrip(19, 22);
        var question4Strip = parallelStrip.substrip(22, 25);

        this.runners = new AnimationRunner[]{
            new AnimationRunner(data.hasBall, hasBallAnimation),
            new AnimationRunner(data.intaking, intakingAnimation),
            new AnimationRunner(() -> Boolean.FALSE.equals(data.armManual.get()), armCoast),
            new AnimationRunner(() -> Boolean.TRUE.equals(data.armManual.get()), armBrake),
            new AnimationRunner(() -> Boolean.FALSE.equals(data.driveManual.get()), driveCoast),
            new AnimationRunner(() -> Boolean.TRUE.equals(data.driveManual.get()), driveBrake),
            new AnimationRunner(() -> DriverStation.isTeleopEnabled() && (DriverStation.getMatchType() != MatchType.None ? DriverStation.getMatchTime() <= 30 : !endgameNotification.animationTimer.hasElapsed(30)), endgameNotification),
            new AnimationRunner(data.auto, robotAutonomousAnimation),
            new AnimationRunner(DriverStation::isDisabled, driverStationConnected),
            new AnimationRunner(DriverStation::isDisabled, new LEDAnimation() {
                @Override
                protected void runAnimation(LEDManager manager) {
                    autoroutineStrip.foreach((i) -> {
                        autoroutineStrip.setLED(i, data.autoColors.get()[0]);
                    });
                }
            }),
            new AnimationRunner(DriverStation::isDisabled, new LEDAnimation() {
                @Override
                protected void runAnimation(LEDManager manager) {
                    question1Strip.foreach((i) -> {
                        question1Strip.setLED(i, data.autoColors.get()[1]);
                    });
                }
            }),
            new AnimationRunner(DriverStation::isDisabled, new LEDAnimation() {
                @Override
                protected void runAnimation(LEDManager manager) {
                    question2Strip.foreach((i) -> {
                        question2Strip.setLED(i, data.autoColors.get()[2]);
                    });
                }
            }),
            new AnimationRunner(DriverStation::isDisabled, new LEDAnimation() {
                @Override
                protected void runAnimation(LEDManager manager) {
                    question3Strip.foreach((i) -> {
                        question3Strip.setLED(i, data.autoColors.get()[3]);
                    });
                }
            }),
            new AnimationRunner(DriverStation::isDisabled, new LEDAnimation() {
                @Override
                protected void runAnimation(LEDManager manager) {
                    question4Strip.foreach((i) -> {
                        question4Strip.setLED(i, data.autoColors.get()[4]);
                    });
                }
            }),
        };

        allianceColorAnimation.setWavelength(4);
        intakingAnimation.setWavelength(2);
        intakingAnimation.setVelocity(2);
        robotAutonomousAnimation.setWavelength(4);
        robotAutonomousAnimation.setVelocity(2);

        allianceColorAnimation.setPriority(0);
        onboardBlank.setPriority(0);
        driverStationConnected.setPriority(1);
        armCoast.setPriority(2);
        armBrake.setPriority(2);
        driveCoast.setPriority(2);
        driveBrake.setPriority(2);
        hasBallAnimation.setPriority(3);
        robotAutonomousAnimation.setPriority(4);
        intakingAnimation.setPriority(5);
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
        public final Supplier<Color[]> autoColors;
        public LedData(BooleanSupplier hasBall, BooleanSupplier intaking, BooleanSupplier auto, Supplier<Boolean> armManual, Supplier<Boolean> driveManual, Supplier<Color[]> autoColors) {
            this.hasBall = hasBall;
            this.intaking = intaking;
            this.auto = auto;
            this.armManual = armManual;
            this.driveManual = driveManual;
            this.autoColors = autoColors;
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
                    animation.pause();
                }
            }
            lastVal = curVal;
        }
    }
}
