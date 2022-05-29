package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.Constants;
import java.util.function.DoubleSupplier;


public class TriggerPressed extends Trigger {
    private final DoubleSupplier triggerPosition;
    
    public TriggerPressed (DoubleSupplier _triggerPosition){
        triggerPosition = _triggerPosition;        
    }

    @Override
    public boolean get() {
      return triggerPosition.getAsDouble() > Constants.TRIGGER_THRESHOLD;
    }
  }