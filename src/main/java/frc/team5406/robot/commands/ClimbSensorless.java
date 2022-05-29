package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.Constants;

public class ClimbSensorless extends CommandBase {
    private final ClimbSubsystem climb;
    private final double angle;

    public ClimbSensorless(ClimbSubsystem subsystem, double _angle) {
        climb = subsystem;
        angle = _angle;
        
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        if(ClimbSubsystem.getClimbState() != -1){
            ClimbSubsystem.setClimberPosition(angle);
        }
    }

    @Override
    public void execute() {
        if(ClimbSubsystem.getClimbState() != -1){
            ClimbSubsystem.setClimberPosition(angle);
        }
    }

    @Override
    public boolean isFinished() {
            return (ClimbSubsystem.getClimbState() == -1) || (Math.abs(ClimbSubsystem.getClimberPosition() - ((angle/360) * Constants.GEAR_RATIO_CLIMBER)) <= 2);
    }

    @Override
    public void end(boolean interrupted) {
        //if(interrupted || ClimbSubsystem.getClimbState() == -1){
            ClimbSubsystem.stopClimber();
        //}
    }

}