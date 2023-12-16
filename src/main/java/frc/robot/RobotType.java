package frc.robot;

public enum RobotType {
    ROBOT_2023_COMP(Mode.REAL),
    ROBOT_2023_PRAC(Mode.REAL),
    SIM(Mode.SIM),
    REPLAY(Mode.REPLAY),
    ;
    public final Mode mode;
    RobotType(Mode mode) {
        this.mode = mode;
    }
    private static final RobotType realDefault =        RobotType.ROBOT_2023_COMP;
    private static final RobotType simulationDefault =  RobotType.SIM;
    public static RobotType getRobot() {
        return Robot.isReal() ? realDefault : simulationDefault;
    }

    public static enum Mode {
        REAL,
        SIM,
        REPLAY,
        ;

    }
}
