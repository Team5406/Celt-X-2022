package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.Constants;
import java.util.function.DoubleSupplier;


public class JoystickMoved extends Trigger {
    private final DoubleSupplier joystickPosition;
    
    public JoystickMoved (DoubleSupplier _joystickPosition){
        joystickPosition = _joystickPosition;        
    }

    @Override
    public boolean get() {
      return Math.abs(joystickPosition.getAsDouble()) > Constants.JOYSTICK_THRESHOLD;
    }
  }