package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.FlashingAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient;
import frc.robot.util.led.strips.LEDStrip;
import frc.robot.util.led.strips.hardware.CANdleStrip;

public class Leds extends VirtualSubsystem {
    private final LEDManager ledManager = LEDManager.getInstance();
    private final CANdle m_candle = new CANdle(Constants.CANDevices.candleCanID, "rio");
    private final CANdleStrip candleLEDs =  new CANdleStrip(m_candle, 18*2);
    private final LEDStrip onboardLEDs =    candleLEDs.getOnboardLEDs();
    private final LEDStrip offboardLEDs =   candleLEDs.getOffboardLEDs();
    private final LEDStrip rightStrip =     offboardLEDs.substrip(0, offboardLEDs.getLength() / 2);
    private final LEDStrip leftStrip =      offboardLEDs.substrip(offboardLEDs.getLength() / 2).reverse();
    private final LEDStrip parallelStrip =  rightStrip.parallel(leftStrip);

    private final LEDAnimation defaultOffboardAnimation = new ScrollingAnimation(Gradient.rainbow, parallelStrip);
    private final LEDAnimation defaultOnboardAnimation = new FlashingAnimation(Gradient.blackToWhite, onboardLEDs);

    public Leds() {
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
        defaultOffboardAnimation.start();
        defaultOnboardAnimation.start();
    }

    @Override
    public void periodic() {
        ledManager.runLEDs();
    }

    public void playOffboardScrolling(Gradient gradient) {
        ledManager.stopAll();
        ledManager.play(new ScrollingAnimation(gradient, offboardLEDs));
    }
}
