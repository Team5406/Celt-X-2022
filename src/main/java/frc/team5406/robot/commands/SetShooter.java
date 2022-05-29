package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SetShooter extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier triggerPress;
    private final BooleanSupplier reject;
    private final BooleanSupplier shooting;
    private final XboxController controller;

    public SetShooter (ShooterSubsystem _shooter, DoubleSupplier _triggerPress, XboxController _controller, BooleanSupplier _reject, BooleanSupplier _shooting) {
        shooter = _shooter;
        triggerPress = _triggerPress;
        controller = _controller;
        reject = _reject;
        shooting = _shooting;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
      ShooterSubsystem.turnOnLimelight();
    }
    
    @Override
    public void execute(){
        SmartDashboard.putNumber("Hood Current", ShooterSubsystem.getHoodCurrent());
        if (reject.getAsBoolean()){
          ShooterSubsystem.setShooterSpeed(Constants.FLYWHEEL_SPEED_LOW_LOB);
          ShooterSubsystem.setHoodPosition(Constants.HOOD_ANGLE_LOW_LOB);
        }
       else if (triggerPress.getAsDouble() > (1-Constants.TRIGGER_THRESHOLD)) {
          double shooterSpeed = SmartDashboard.getNumber("Shooter Target RPM", 1000);
          double hoodPosition = SmartDashboard.getNumber("Hood Position", 0);
          ShooterSubsystem.updateLimelightTracking();
          if(ShooterSubsystem.llHasValidTarget){
            shooterSpeed = ShooterSubsystem.getLLShooterSpeed();
            hoodPosition = ShooterSubsystem.getLLHoodPosition();

            SmartDashboard.putNumber("ShooterSpeed Calc", ShooterSubsystem.getLLShooterSpeed());
            SmartDashboard.putNumber("HoodPosition Calc", ShooterSubsystem.getLLHoodPosition());


            SmartDashboard.putNumber("SS", Math.abs(shooterSpeed - ShooterSubsystem.getShooterSpeed()));
            SmartDashboard.putNumber("lltx", Math.abs(ShooterSubsystem.lltx));
            SmartDashboard.putNumber("HP", Math.abs(ShooterSubsystem.getLLHoodPosition() - hoodPosition));
            
            if(Math.abs(shooterSpeed - ShooterSubsystem.getShooterSpeed()) < shooterSpeed*0.02
              && Math.abs(ShooterSubsystem.lltx) < 1
              && Math.abs(ShooterSubsystem.getLLHoodPosition() - hoodPosition) < 0.2){
              controller.setRumble(RumbleType.kLeftRumble, 1);
              controller.setRumble(RumbleType.kRightRumble, 1);
        
            }else{
              controller.setRumble(RumbleType.kLeftRumble, 0);
              controller.setRumble(RumbleType.kRightRumble, 0);  
            }
        }else{
            shooterSpeed = Constants.FLYWHEEL_SPEED_FENDER_HIGH;
            hoodPosition = Constants.HOOD_ANGLE_FENDER_HIGH;
            controller.setRumble(RumbleType.kLeftRumble, 0);
            controller.setRumble(RumbleType.kRightRumble, 0);
          }

          ShooterSubsystem.setShooterSpeed(shooterSpeed);
          ShooterSubsystem.setHoodPosition(hoodPosition);

          /* if (ShooterSubsystem.getShooterSpeed() > shooterSpeed*0.95 && ShooterSubsystem.getShooterSpeed() < shooterSpeed*1.05){
            ShooterSubsystem.setBoosterSpeed(boosterSpeed);
          }else{
            ShooterSubsystem.stopBooster();
          }*/

        }else{
          //System.out.println("Stop Intake");
          if(!shooting.getAsBoolean()){
            ShooterSubsystem.stopShooter();
            controller.setRumble(RumbleType.kLeftRumble, 0);
            controller.setRumble(RumbleType.kRightRumble, 0);
          }
          


        }
    }



    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.stopShooter();
      ShooterSubsystem.stopBooster();          
      ShooterSubsystem.turnOffLimelight();
  
    }
}