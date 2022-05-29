package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.ConveyorSubsystem;

public class AutoShoot extends CommandBase {
  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem convey;
  double shooterSpeed, hoodPosition;

  public AutoShoot(ShooterSubsystem _shooter, ConveyorSubsystem _convey) {
    shooter = _shooter;
    convey = _convey;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    ShooterSubsystem.turnOnLimelight();
    ShooterSubsystem.stopBooster();
    ConveyorSubsystem.stopConveyorTop();
    //IntakeSubsystem.compressorDisabled();

  }

  @Override
  public void execute() {

    if(ShooterSubsystem.llHasValidTarget){
      shooterSpeed = ShooterSubsystem.getLLShooterSpeed();
      hoodPosition = ShooterSubsystem.getLLHoodPosition();
      ShooterSubsystem.setShooterSpeed(shooterSpeed);
      ShooterSubsystem.setHoodPosition(hoodPosition);

      if(Math.abs(shooterSpeed - ShooterSubsystem.getShooterSpeed()) < shooterSpeed*0.02
    && Math.abs(ShooterSubsystem.lltx) < 1.5
    && Math.abs(ShooterSubsystem.getLLHoodPosition() - hoodPosition) < 0.2){
            System.out.println("Auto Shoot Converged");
            ShooterSubsystem.setBoosterSpeed(Constants.BOOSTER_SPEED); 
            ConveyorSubsystem.setConveyorTopSpeed(Constants.CONVEYOR_SPEED_TOP);

    } else{
            //ShooterSubsystem.holdBoosterPosition();
            //ConveyorSubsystem.holdConveyorPosition();
            ShooterSubsystem.LLAlignTurret();
            ShooterSubsystem.stopBooster();
            ConveyorSubsystem.stopConveyorTop();
        }
  
    }else{
      //what to do if can't see the target? (turn)
    } 


    
    }


  @Override
  public boolean isFinished() {
    if(ShooterSubsystem.llHasValidTarget
    && Math.abs(shooterSpeed - ShooterSubsystem.getShooterSpeed()) < shooterSpeed*0.02
    && Math.abs(ShooterSubsystem.lltx) < 1.5
    && Math.abs(ShooterSubsystem.getLLHoodPosition() - hoodPosition) < 0.2){
      System.out.println("auto shoot finished");
      return true;
    } else {
      return false;
    }
  }
  

  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      ShooterSubsystem.setBoosterSpeed(Constants.BOOSTER_SPEED); 
      ConveyorSubsystem.setConveyorTopSpeed(Constants.CONVEYOR_SPEED_TOP);
    }
    
    System.out.println("Auto Shoot Finished");
    //IntakeSubsystem.compressorEnabled();
    //ShooterSubsystem.stopShooter();
    //ShooterSubsystem.setHoodPosition(0);

  }
}