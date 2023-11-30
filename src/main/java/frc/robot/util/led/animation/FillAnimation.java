package frc.robot.util.led.animation;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class FillAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final Color color;
    
    public FillAnimation(Color color, LEDStrip... strips) {
        this.strips = strips;
        this.color = color;
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                ledStrip.setLED(i, color);
            });
        }
    }
}
