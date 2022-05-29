package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DistanceSensor extends Trigger {
  private final double threshold;
  private final int sensor;
    
    public DistanceSensor (int _sensor, double _threshold){
      threshold  = _threshold;    
      sensor  = _sensor;    
    }

    @Override
    public boolean get() {
      double distance = 0; //ClimbSubsystem.distanceReading(sensor);
      return  distance < threshold;
    }
  }