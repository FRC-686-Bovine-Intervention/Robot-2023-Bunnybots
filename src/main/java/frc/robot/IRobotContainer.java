package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface IRobotContainer {
    public default void robotPeriodic() {}

    public default void enabledInit() {}
    public default void enabledPeriodic() {}
    
    public default Command getAutonomousCommand() {return null;}
    public default void autonomousInit() {}
    public default void autonomousPeriodic() {}
    
    public default void teleopInit() {}
    public default void teleopPeriodic() {}
    
    public default void disabledInit() {}
    public default void disabledPeriodic() {}
    
    public default void testInit() {}
    public default void testPeriodic() {}

}
