package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ConveyorSubsystem;

public class AutoConvey extends CommandBase {
    private final ConveyorSubsystem conveyor;
    private int count = 0;

  
    public AutoConvey (ConveyorSubsystem _conveyor) {
      conveyor = _conveyor;
      addRequirements(conveyor);
    }
      
    
    @Override
    public void initialize() {

      
    }

    @Override
    public void execute() {
      count++;
      ConveyorSubsystem.setConveyorBottomSpeed((count % 10 < 9 ? 1 : -0.4) * Constants.CONVEYOR_SPEED_BOTTOM);
    }

    @Override
    public void end(boolean interrupted) {
    }

}
