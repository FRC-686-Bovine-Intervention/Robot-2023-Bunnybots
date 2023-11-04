package frc.robot.util.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The X and Y axes should be perpendicular to each other
 */
public class Joystick {
    private final Axis x;
    private final Axis y;

    public Joystick(Axis x, Axis y) {
        this.x = x;
        this.y = y;
    }

    public Axis x() {
        return x;
    }

    public Axis y() {
        return y;
    }

    public Joystick multiplyX(DoubleSupplier xCoef) {
        return new Joystick(x.multiply(xCoef), y);
    }
    public Joystick multiplyX(double xCoef) {
        return new Joystick(x.multiply(xCoef), y);
    }
    public Joystick invertX() {
        return new Joystick(x.invert(), y);
    }

    public Joystick multiplyY(DoubleSupplier yCoef) {
        return new Joystick(x, y.multiply(yCoef));
    }
    public Joystick multiplyY(double yCoef) {
        return new Joystick(x, y.multiply(yCoef));
    }
    public Joystick invertY() {
        return new Joystick(x, y.invert());
    }

    public Joystick multiply(DoubleSupplier xCoef, DoubleSupplier yCoef) {
        return new Joystick(x.multiply(xCoef), y.multiply(yCoef));
    }
    public Joystick multiply(double xCoef, double yCoef) {
        return new Joystick(x.multiply(xCoef), y.multiply(yCoef));
    }

    public DoubleSupplier magnitude() {
        return () -> Math.hypot(x.getAsDouble(), y.getAsDouble());
    }

    public DoubleSupplier radsFromPosXCCW() {
        return () -> Math.atan2(y.getAsDouble(), x.getAsDouble());
    }
    public DoubleSupplier radsFromPosYCCW() {
        return () -> Math.atan2(-x.getAsDouble(), y.getAsDouble());
    }
    public DoubleSupplier radsFromNegXCCW() {
        return () -> Math.atan2(-y.getAsDouble(), -x.getAsDouble());
    }
    public DoubleSupplier radsFromNegYCCW() {
        return () -> Math.atan2(x.getAsDouble(), -y.getAsDouble());
    }

    public DoubleSupplier radsFromPosXCW() {
        return () -> MathUtil.inputModulus(-Math.atan2(y.getAsDouble(), x.getAsDouble()), 0, 2*Math.PI);
    }
    public DoubleSupplier radsFromPosYCW() {
        return () -> MathUtil.inputModulus(-Math.atan2(-x.getAsDouble(), y.getAsDouble()), 0, 2*Math.PI);
    }
    public DoubleSupplier radsFromNegXCW() {
        return () -> MathUtil.inputModulus(-Math.atan2(-y.getAsDouble(), -x.getAsDouble()), 0, 2*Math.PI);
    }
    public DoubleSupplier radsFromNegYCW() {
        return () -> MathUtil.inputModulus(-Math.atan2(x.getAsDouble(), -y.getAsDouble()), 0, 2*Math.PI);
    }

    // public Joystick roughRadialDeadband() {

    // }

    public Joystick smoothRadialDeadband(double deadband) {
        DoubleSupplier magnitude = ()->MathUtil.applyDeadband(magnitude().getAsDouble(), deadband);
        DoubleSupplier angle = radsFromPosXCCW();
        return new Axis(()->Math.cos(angle.getAsDouble())*magnitude.getAsDouble()).asXwithY(new Axis(()->Math.sin(angle.getAsDouble())*magnitude.getAsDouble()));
    }

    public static class Axis implements DoubleSupplier {
        private final DoubleSupplier source;

        public Axis(DoubleSupplier source) {
            this.source = source;
        }

        @Override
        public double getAsDouble() {
            return source.getAsDouble();
        }

        public Axis multiply(DoubleSupplier coef) {
            return new Axis(() -> source.getAsDouble() * coef.getAsDouble());
        }

        public Axis multiply(double coef) {
            return multiply(() -> coef);
        }

        public Axis invert() {
            return multiply(-1);
        }

        public Axis add(DoubleSupplier other) {
            return new Axis(() -> source.getAsDouble() + other.getAsDouble());
        }

        public Axis add(double other) {
            return add(() -> other);
        }

        public Axis roughDeadband(double deadband) {
            return new Axis(() -> Math.abs(source.getAsDouble()) < deadband ? 0 : source.getAsDouble());
        }

        public Axis smoothDeadband(double deadband) {
            return new Axis(() -> Math.copySign(Math.max((Math.abs(source.getAsDouble()) - deadband) / (1 - deadband), 0), source.getAsDouble()));
        }

        public Trigger aboveThreshold(double threshold) {
            return aboveThreshold(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
        }

        public Trigger aboveThreshold(EventLoop loop, double threshold) {
            return new Trigger(loop, () -> source.getAsDouble() >= threshold);
        }

        public Joystick asXwithY(Axis yAxis) {
            return new Joystick(this, yAxis);
        }

        public Joystick asYwithX(Axis xAxis) {
            return new Joystick(xAxis, this);
        }
    }
}

