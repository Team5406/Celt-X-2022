package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class ResetHoodEncoder extends CommandBase {

    private final ShooterSubsystem shooter;
    private int currentCounter = 0;
    public ResetHoodEncoder(ShooterSubsystem _shooter) {
        shooter = _shooter;
   //     addRequirements(shooter);
    }

    @Override
    public void initialize() {
        ShooterSubsystem.disableHoodLimits();
        currentCounter = 0;
    }

    @Override
    public void execute() {
        ShooterSubsystem.setHoodSpeed(Constants.HOOD_ZEROING_SPEED); // 1.65 is magic number
        if(ShooterSubsystem.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE){
            currentCounter++;
        }
    }
    @Override
    public boolean isFinished() {
        //return ShooterSubsystem.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE && Math.abs(ShooterSubsystem.getHoodVelocity()) <= 10;
        return currentCounter >= 3;
    }

    @Override
    public void end(boolean interrupted){
        ShooterSubsystem.resetHood();
        ShooterSubsystem.stopHood();
        ShooterSubsystem.enableHoodLimits();
    }
}
