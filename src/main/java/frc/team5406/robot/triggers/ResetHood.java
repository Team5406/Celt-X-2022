package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.subsystems.ShooterSubsystem;



public class ResetHood extends Trigger {
    
    public ResetHood (){  
    }

    @Override
    public boolean get() {
      return !ShooterSubsystem.hoodReset;
    }
  }