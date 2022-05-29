package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class StopClimber extends CommandBase {
    private final ClimbSubsystem climb;

    public StopClimber(ClimbSubsystem _climb) {
        climb = _climb;
    }
    @Override
    public void initialize() {
        ClimbSubsystem.stopClimber();
    }

    public void end(boolean interrupted){
    }
}
