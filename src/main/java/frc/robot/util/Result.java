package frc.robot.util;

public class Result<T,E> {
    private final T ok;
    private final E err;

    private Result(T ok, E err) {
        this.ok = ok;
        this.err = err;
    }

    
}
