package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class AbortClimb extends CommandBase {

   
    private final ClimbSubsystem climb;
    
    public AbortClimb (ClimbSubsystem _climb){
       climb = _climb;  
       addRequirements(climb);
    }

    @Override
    public void initialize() {
      System.out.println("Abort Climb");
      ClimbSubsystem.setClimbState(-1);
      ClimbSubsystem.stopClimber();
    }
    @Override
    public boolean isFinished() {
  
      return true;
    }
  
    @Override
    public void end(boolean interrupted) {
  
    }

}