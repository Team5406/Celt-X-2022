package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ConveyorSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.Timer;

public class Convey extends CommandBase {
    private final ConveyorSubsystem conveyor;
  
    private final DoubleSupplier triggerPress;
    private final boolean manual;
    private final boolean reverse;
    private final boolean shooting;
    private final BooleanSupplier reject;
    private int count = 0;
  
    public Convey (ConveyorSubsystem _conveyor, DoubleSupplier _triggerPress, boolean _reverse, boolean _manual, boolean _shooting, BooleanSupplier _reject) {
      conveyor = _conveyor;
      manual = _manual;
      reverse = _reverse;
      shooting = _shooting;
      reject = _reject;
      triggerPress = _triggerPress;
      addRequirements(conveyor);
    }
      
    
    @Override
    public void initialize() {

      
    }


    @Override
    public void execute() {
      if (shooting){
         count++;
         ConveyorSubsystem.setConveyorBottomSpeed((count % 10 < 7 ? 1 : -0.4) * Constants.CONVEYOR_SPEED_BOTTOM);
         ConveyorSubsystem.keepSpinning = false;
      }else if (!manual && triggerPress.getAsDouble() > (1-Constants.TRIGGER_THRESHOLD)) {
        //System.out.println((reverse?"Convey Down":"Convey Up") + " - auto");
        ConveyorSubsystem.setConveyorBottomSpeed((reverse?-1:1)*Constants.CONVEYOR_SPEED_BOTTOM);
        ConveyorSubsystem.lastTime = Timer.getFPGATimestamp();
        ConveyorSubsystem.keepSpinning = true;
      }else if (manual && triggerPress.getAsDouble() > Constants.TRIGGER_THRESHOLD){
        //System.out.println((reverse?"Convey Down":"Convey Up") + " - manual");
        ConveyorSubsystem.setConveyorBottomSpeed((reverse?-1:1)*triggerPress.getAsDouble()*Constants.CONVEYOR_MAX_SPEED);
        ConveyorSubsystem.keepSpinning = false;
      }else if(reject.getAsBoolean()){
        ConveyorSubsystem.setConveyorBottomSpeed(-1*Constants.CONVEYOR_MAX_SPEED);        
        ConveyorSubsystem.keepSpinning = false;
      }else{
        if (!(ConveyorSubsystem.keepSpinning && (Timer.getFPGATimestamp() - ConveyorSubsystem.lastTime) < Constants.KEEP_SPINNING_TIME_CONVEY)) {
          ConveyorSubsystem.stopConveyorBottom();
        }
      }

    }

    @Override
    public void end(boolean interrupted) {
      ConveyorSubsystem.stopConveyorBottom();
    }

}
