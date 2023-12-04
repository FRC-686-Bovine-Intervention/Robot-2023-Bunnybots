package frc.robot.subsystems.leds;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.EndgameNotificationAnim;
import frc.robot.util.led.animation.FillAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient;
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

    // private final LEDAnimation defaultOffboardAnimation = new ScrollingAnimation(Gradient.rainbow, parallelStrip);
    // private final LEDAnimation defaultOnboardAnimation = new FlashingAnimation(Gradient.blackToWhite, onboardLEDs);
    private final LEDAnimation hasBallAnimation = new FillAnimation(Color.kGreen, parallelStrip);
    private final LEDAnimation intakingAnimation = new FillAnimation(Color.kPurple, parallelStrip);
    private final LEDAnimation endgameNotification = new EndgameNotificationAnim(parallelStrip);
    private final ScrollingAnimation allianceColorAnimation = new ScrollingAnimation((x) -> {
        var colors = new Color[]{
            Color.kBlack,
            (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Color.kRed : Color.kBlue)
        };
        return InterpolationStyle.Linear.interpolate(x, colors);
    }, TilingFunction.Sinusoidal, parallelStrip);
    private LedData data = null;
    private boolean endGameNotificationSent = false;

    public Leds() {
        super();
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
        // defaultOffboardAnimation.start();
        // defaultOnboardAnimation.start();
        allianceColorAnimation.setWavelength(4);
        endgameNotification.setPriority(4);
        hasBallAnimation.setPriority(3);
        intakingAnimation.setPriority(2);
        allianceColorAnimation.setPriority(1);
        allianceColorAnimation.start();
    }

    @Override
    public void periodic() {
        ledManager.runLEDs();
        if (data.hasBall.getAsBoolean()){
            hasBallAnimation.start();
        }
        else{
            hasBallAnimation.stop();
        }
        if (data.intaking.getAsBoolean()){
            intakingAnimation.start();
        }
        else{
            intakingAnimation.stop();
        }
        if (DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 30 && ! endGameNotificationSent){
            endGameNotificationSent = true;
            endgameNotification.start();
        }
        if (DriverStation.isDisabled()){
            endGameNotificationSent = false;
        }
    }

    public void playOffboardScrolling(Gradient gradient) {
        ledManager.stopAll();
        ledManager.play(new ScrollingAnimation(gradient, offboardLEDs));
    }

    public static class LedData{
        public BooleanSupplier hasBall;
        public BooleanSupplier intaking;
        public LedData(BooleanSupplier hasBall, BooleanSupplier intaking){
            this.hasBall = hasBall;
            this.intaking = intaking;
        }
    }

    public void setData(LedData data){
        this.data = data;
    }
}
