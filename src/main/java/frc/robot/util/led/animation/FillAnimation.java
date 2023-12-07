package frc.robot.util.led.animation;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class FillAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final Supplier<Color> color;
    
    public FillAnimation(Color color, LEDStrip... strips) {
        this(() -> color, strips);
    }

    public FillAnimation(Supplier<Color> color, LEDStrip... strips) {
        this.strips = strips;
        this.color = color;
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                ledStrip.setLED(i, color.get());
            });
        }
    }
}
