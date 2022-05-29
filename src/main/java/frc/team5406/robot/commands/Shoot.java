package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shoot extends CommandBase {
    private final boolean shoot;
    private final BooleanSupplier reject;
    private int count = 0;
    
    
    public Shoot (boolean _shoot, BooleanSupplier _reject) {
        shoot = _shoot;
        reject = _reject;
        
    }
    @Override
    public void execute(){
        if(shoot){
            double boosterSpeed = SmartDashboard.getNumber("Booster Target RPM", 1000);
            ShooterSubsystem.setBoosterSpeed(boosterSpeed);
            ConveyorSubsystem.setConveyorTopSpeed(Constants.CONVEYOR_SPEED_TOP);

        }else if(reject.getAsBoolean()){
            ShooterSubsystem.setBoosterSpeed(Constants.BOOSTER_SPEED);
            ConveyorSubsystem.setConveyorTopSpeed(Constants.CONVEYOR_SPEED_TOP);
        }else{
            double boosterSpeed = SmartDashboard.getNumber("Booster Target RPM", 1000);
            ShooterSubsystem.setBoosterSpeed(-1 * boosterSpeed);
            count++;
            ConveyorSubsystem.setConveyorTopSpeed((count % 10 < 5 ? -1 : 0.5) * Constants.CONVEYOR_SPEED_TOP);

        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.stopBooster();
      ConveyorSubsystem.stopConveyorTop();
      //ShooterSubsystem.holdBoosterPosition();
      //ConveyorSubsystem.holdConveyorPosition();
    
  
    }
}