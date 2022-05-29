package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AutoStopBoosterTopConvey extends CommandBase {

  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
    private final ConveyorSubsystem convey;

  public AutoStopBoosterTopConvey(ShooterSubsystem subsystem, ConveyorSubsystem _convey) {
    shooter = subsystem;
    convey = _convey;
  }

  @Override
  public void initialize() {
    ShooterSubsystem.stopBooster();
    ConveyorSubsystem.stopConveyor();
  }

  @Override
  public void execute() {

    
  }

  @Override
  public boolean isFinished() {
    return true;

  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("Stop Booster and Top Convey");
  }
}