package frc.robot.util.led.animation;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;

public class EndgameNotificationAnim extends FlashingAnimation {
    public EndgameNotificationAnim(LEDStrip... strips) {
        super(new BasicGradient(InterpolationStyle.Linear, Color.kBlack, Color.kYellow), TilingFunction.Sawtooth, strips);
        setPeriod(0.25);
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        if(animationTimer.hasElapsed(1)) {
            manager.stop(this);
            return;
        }
        super.runAnimation(manager);
    }
}
