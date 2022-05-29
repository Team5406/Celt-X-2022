package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class ClimbState extends Trigger {
    private final int climbstate;
    
    public ClimbState (int _climbState){
       climbstate  = _climbState;        
    }

    @Override
    public boolean get() {
      return climbstate == ClimbSubsystem.getClimbState();
    }
  }