package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;


public class DPadPressed extends Trigger {
    private final DoubleSupplier dPad;
    
    public DPadPressed (DoubleSupplier _dPad){
        dPad = _dPad;        
    }

    @Override
    public boolean get() {
      return dPad.getAsDouble() != -1;
    }
  }