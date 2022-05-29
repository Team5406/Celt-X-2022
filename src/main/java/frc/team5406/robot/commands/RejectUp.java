package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;

public class RejectUp extends CommandBase {
    private final ShooterSubsystem shooter;

    public RejectUp (ShooterSubsystem _shooter) {
        shooter = _shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        ShooterSubsystem.setShooterSpeed(Constants.FLYWHEEL_SPEED_LOW_LOB);
        ShooterSubsystem.setHoodPosition(Constants.HOOD_ANGLE_LOW_LOB);
        ShooterSubsystem.setBoosterSpeed(Constants.BOOSTER_SPEED);
        ConveyorSubsystem.setConveyorTopSpeed(Constants.CONVEYOR_SPEED_TOP);
    }

    @Override
    public boolean isFinished() {


        return false;
    }

    @Override
    public void end(boolean interrupted) {

        ShooterSubsystem.stopShooter();
        ConveyorSubsystem.stopConveyor();
        ShooterSubsystem.stopBooster();
    }
    
}
