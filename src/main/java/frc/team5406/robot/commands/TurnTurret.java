package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import java.util.function.DoubleSupplier;

public class TurnTurret extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier joystick;
  
    public TurnTurret (ShooterSubsystem _shooter, DoubleSupplier _joystick) {
        shooter = _shooter;
        joystick = _joystick;
        addRequirements(shooter);
        
    }
    @Override
    public void execute(){
        if (Math.abs(joystick.getAsDouble()) > Constants.JOYSTICK_THRESHOLD) {
          ShooterSubsystem.setTurretSpeed(joystick.getAsDouble()*Constants.TURRET_MAX_SPEED);
          
        }else{
          //System.out.println("Stop Intake");
          ShooterSubsystem.stopTurret();
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.stopTurret();
  
    }
}