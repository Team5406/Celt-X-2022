package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.Constants;


public class AutoIntake extends CommandBase {
  // The subsystem the command runs on
  private final IntakeSubsystem intake;
  private final ConveyorSubsystem convey;
  private final Boolean intaking;
  private int count = 0;


  public AutoIntake(IntakeSubsystem subsystem, ConveyorSubsystem _convey, Boolean _intaking) {
    intake = subsystem;
    convey = _convey;
    intaking = _intaking;
    addRequirements(intake);
    addRequirements(convey);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(intaking){
      count++;
      ConveyorSubsystem.setConveyorBottomSpeed((count % 10 < 9 ? 1 : -0.4) * Constants.CONVEYOR_SPEED_BOTTOM);
      IntakeSubsystem.intake();
  } else {
      IntakeSubsystem.stopIntake();
  }


  }

  @Override
  public boolean isFinished() {

    // NEEDS SOME EXTERNAL INPUT OR SEPARATE STOP OUT

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished Auto Intake");
      IntakeSubsystem.stopIntake();
  
     
  }
}