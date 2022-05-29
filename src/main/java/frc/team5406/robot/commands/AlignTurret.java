package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignTurret extends CommandBase {
    private final ShooterSubsystem shooter;
  
    public AlignTurret (ShooterSubsystem _shooter) {
        shooter = _shooter;
        //addRequirements(shooter);
        
    }
    @Override
    public void execute(){

      ShooterSubsystem.LLAlignTurret();
      ShooterSubsystem.turnOnLimelight();
    }

    @Override
    public void initialize() {
    }

    @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(ShooterSubsystem.lltx) < 1 ;
  }

    @Override
    public void end(boolean interrupted) {

      ShooterSubsystem.stopTurret();
      ShooterSubsystem.turnOffLimelight();
    //  System.out.println("Aligned Turret");
  
    }
}