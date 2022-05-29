package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.IntakeSubsystem;


public class AutoEject extends CommandBase {
  // The subsystem the command runs on
  private final IntakeSubsystem intake;
  private final Boolean intaking;
  private int count = 0;


  public AutoEject(IntakeSubsystem subsystem, Boolean _intaking) {
    intake = subsystem;
    intaking = _intaking;
    addRequirements(intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    
    if(intaking){
      count++;
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
      IntakeSubsystem.stopIntake();
  }
}