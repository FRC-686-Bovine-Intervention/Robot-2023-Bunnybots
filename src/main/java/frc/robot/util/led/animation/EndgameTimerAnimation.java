package frc.robot.util.led.animation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class EndgameTimerAnimation extends LEDAnimation {
    private final LEDStrip strip;

    public EndgameTimerAnimation(LEDStrip strip) {
        this.strip = strip;
    }

    private static final Color[] colors = {Color.kBlack, Color.kRed, Color.kYellow, Color.kGreen};
    private static final double darkScalar = 0.2;

    @Override
    protected void runAnimation(LEDManager manager) {
        strip.foreach((index) -> {
            int seg = index * 10 / strip.getLength();
            double timeDec = DriverStation.getMatchTime();
            int timeMod = (int)Math.ceil(timeDec) % 10;
            int topColorInd = MathUtil.clamp((int)Math.ceil(timeDec / 10), 0, colors.length - 1);
            int botColorInd = MathUtil.clamp((int)Math.floor(timeDec / 10), 0, colors.length - 1);
            var color = colors[seg < timeMod ? topColorInd : botColorInd];
            if(timeMod == 0) {
                color = colors[topColorInd];
                double scalar = (Math.cos(4 * Math.PI * (timeDec % 1) - Math.PI) + 1) / 2;
                color = new Color(color.red * scalar, color.green * scalar, color.blue * scalar);
            } else if (seg % 2 == 0) {
                color = new Color(
                    color.red * darkScalar + 255 * (1 - darkScalar), 
                    color.green * darkScalar + 255 * (1 - darkScalar),
                    color.blue * darkScalar + 255 * (1 - darkScalar)
                );
            }
            strip.setLED(index, color);
        });
    }
}
