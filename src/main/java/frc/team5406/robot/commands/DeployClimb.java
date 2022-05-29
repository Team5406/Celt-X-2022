package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class DeployClimb extends CommandBase {
    private final ClimbSubsystem climb;
    private final DoubleSupplier m_joystick;

    public DeployClimb(ClimbSubsystem subsystem, DoubleSupplier joystick) {
        climb = subsystem;
        m_joystick = joystick;
        
        addRequirements(climb);
    }

    public void execute() {
        new RotateClimber(climb, m_joystick)
        .andThen(
            new WaitCommand(5),

            new RotateClimber(climb, m_joystick)
            );
    }
}