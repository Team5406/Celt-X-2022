package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class SetHook extends CommandBase {
    private final ClimbSubsystem climb;
    private final int hook;
    private final boolean extend;

    public SetHook(ClimbSubsystem subsystem, int _hook, boolean _extend) {
        climb = subsystem;
        hook = _hook;
        extend = _extend;
        //addRequirements(climb);
    }

    @Override
    public void initialize() {
        if (extend) {
            ClimbSubsystem.closeHook(hook);
         
        } else {
            ClimbSubsystem.openHook(hook); 
        }

    }
    
@Override
    public boolean isFinished() {
  
      return true;
    }
  
    @Override
    public void end(boolean interrupted) {
  
    }
    
}
