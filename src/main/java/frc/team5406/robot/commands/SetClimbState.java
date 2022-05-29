package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class SetClimbState extends CommandBase {
    private final ClimbSubsystem climb;
    private final int climbState;

    public SetClimbState(ClimbSubsystem _climb, int _climbState) {
        climb = _climb;
        climbState = _climbState;
        addRequirements(climb);
    }
    @Override
    public void initialize() {
        ClimbSubsystem.setClimbState(climbState);
    }

    @Override
    public boolean isFinished() {
  
      return true;
    }
  
    @Override
    public void end(boolean interrupted) {
  
    }
    
}