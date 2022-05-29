package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SetTurretPosition extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier dpad;
  
    public SetTurretPosition (ShooterSubsystem _shooter, DoubleSupplier _dpad) {
        shooter = _shooter;
        dpad = _dpad;
        addRequirements(shooter);
        
    }
    @Override
    public void execute(){
      SmartDashboard.putNumber("DPad", dpad.getAsDouble());
        if (dpad.getAsDouble() != -1) {
          ShooterSubsystem.setTurretPosition(dpad.getAsDouble()-90);  
        }else{
          //ShooterSubsystem.stopTurret();
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
      //ShooterSubsystem.stopTurret();
  
    }
}