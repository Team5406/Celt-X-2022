package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Rumble extends CommandBase {
    private final XboxController controller;
    private final Boolean side;
    private final Boolean state;

    public Rumble (XboxController _controller, Boolean _side, Boolean _state) {
      controller = _controller;
      side = _side;
      state = _state;
    }
      
    
    @Override
    public void initialize() {

      
    }

    @Override
    public void execute() {
      controller.setRumble(side?RumbleType.kLeftRumble:RumbleType.kRightRumble, state?1:0);
    }

    @Override
    public void end(boolean interrupted) {

    }

}
