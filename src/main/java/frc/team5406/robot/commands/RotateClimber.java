package frc.team5406.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class RotateClimber extends CommandBase {
    private final ClimbSubsystem climb;
    private final DoubleSupplier m_joystick;

    public RotateClimber(ClimbSubsystem subsystem, DoubleSupplier joystick) {
        climb = subsystem;
        m_joystick = joystick;
        
        addRequirements(climb);
    }

    public void execute() {
        //ClimbSubsystem.setRotateClimberPosition(-1*m_joystick.getAsDouble());
        ClimbSubsystem.setRotateClimberSpeed(-1*m_joystick.getAsDouble()*Constants.CLIMBER_MAX_SPEED);
    }

    public void end(boolean interrupted){
        ClimbSubsystem.stopClimber();
    }
}
