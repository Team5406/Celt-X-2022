package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import java.util.function.DoubleSupplier;

public class HighLob extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier triggerPress;
  
    public HighLob (ShooterSubsystem _shooter, DoubleSupplier _triggerPress) {
        shooter = _shooter;
        triggerPress = _triggerPress;
        addRequirements(shooter);
        
    }
    @Override
    public void execute(){
        if (triggerPress.getAsDouble() > (1-Constants.TRIGGER_THRESHOLD)) {
          //System.out.println(boosterSpeed);
          ShooterSubsystem.setShooterSpeed(Constants.FLYWHEEL_SPEED_FENDER_HIGH);
          ShooterSubsystem.setHoodPosition(Constants.HOOD_ANGLE_FENDER_HIGH);
          
        }else{
          //System.out.println("Stop Intake");
          ShooterSubsystem.stopShooter();

        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.stopShooter();

    }
}
