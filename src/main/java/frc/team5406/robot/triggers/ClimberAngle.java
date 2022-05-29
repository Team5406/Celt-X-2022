package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberAngle extends Trigger {
    private final double threshold;
    private final ClimbSubsystem climber;

    
    public ClimberAngle (ClimbSubsystem _climber, double _threshold){
       threshold  = _threshold;    
       climber = _climber;    
    }

    @Override
    public boolean get() {
      double climberPosition = ClimbSubsystem.getClimberPosition();
      SmartDashboard.putNumber("Climber Position", climberPosition);
      return  climberPosition > threshold;
    }
  }