package frc.robot.util.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    public static class Axis implements DoubleSupplier {
        private final DoubleSupplier return_value;

        public Axis(DoubleSupplier return_value) {
            this.return_value = return_value;
        }

        @Override
        public double getAsDouble() {
            return return_value.getAsDouble();
        }

        public Axis multiply(DoubleSupplier other) {
            return new Axis(() -> return_value.getAsDouble() * other.getAsDouble());
        }

        public Axis multiply(double other) {
            return multiply(() -> other);
        }

        public Axis invert() {
            return multiply(-1);
        }

        public Axis add(DoubleSupplier other) {
            return new Axis(() -> return_value.getAsDouble() + other.getAsDouble());
        }

        public Axis add(double other) {
            return add(() -> other);
        }

        public Axis roughDeadband(double deadband) {
            return new Axis(() -> Math.abs(return_value.getAsDouble()) < deadband ? 0 : return_value.getAsDouble());
        }

        public Axis smoothDeadband(double deadband) {
            return new Axis(() -> Math.copySign(Math.max((Math.abs(return_value.getAsDouble()) - deadband) / (1 - deadband), 0), return_value.getAsDouble()));
        }

        public Trigger aboveThreshold(double threshold) {
            return aboveThreshold(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
        }

        public Trigger aboveThreshold(EventLoop loop, double threshold) {
            return new Trigger(loop, () -> return_value.getAsDouble() >= threshold);
        }
    }
}
