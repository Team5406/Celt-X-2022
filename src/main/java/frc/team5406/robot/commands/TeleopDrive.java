package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.Constants;

public class TeleopDrive extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;
    private final DoubleSupplier slowMode;
    private final BooleanSupplier shooting;
    private final Boolean turnInPlace;

    public TeleopDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, Boolean _turnInPlace, DoubleSupplier _slowMode, BooleanSupplier _shooting) {
        drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        turnInPlace = _turnInPlace;
        slowMode= _slowMode;
        shooting= _shooting;
        addRequirements(drive);
    }

    public void execute() {
        
        
        if(Math.abs(m_forward.getAsDouble()) <= 0.03 && Math.abs( m_rotation.getAsDouble()) <= 0.05 && (Math.abs(DriveSubsystem.getAverageSpeed()) <= Constants.BL_DRIVETRAIN_SPEED || DriveSubsystem.baselockStarted)){
            DriveSubsystem.baselock();
        } else {
            double input = -1*m_forward.getAsDouble();
            double vTarget = Constants.MAX_ACTUAL_SPEED_METERS_PER_SECOND * input;
            //System.out.println("Joystick Input: " + input + ", vTarget: " + vTarget + ", Avg Speed: " + DriveSubsystem.getAverageSpeed());
            double turningRatio = 1;
            if(Math.abs(DriveSubsystem.getLeftSpeed()) > Math.abs(DriveSubsystem.getRightSpeed())){

                turningRatio = Math.abs(DriveSubsystem.getLeftSpeed()-DriveSubsystem.getRightSpeed())/Math.abs(DriveSubsystem.getLeftSpeed());
            }else{
                turningRatio = Math.abs(DriveSubsystem.getLeftSpeed()-DriveSubsystem.getRightSpeed())/Math.abs(DriveSubsystem.getRightSpeed());
            }
            if(Math.abs(input) < 0.03){
                vTarget = 0;
            }else if(turningRatio < 1.15 && Math.signum(input) != Math.signum(DriveSubsystem.getAverageSpeed()) && Math.abs((vTarget - DriveSubsystem.getAverageSpeed())) > Constants.DELTA_V_THRESHOLD)
            {
                vTarget = DriveSubsystem.getAverageSpeed() +Math.signum(vTarget)* Constants.DELTA_V_THRESHOLD;
            }

            input = -1*vTarget / Constants.MAX_ACTUAL_SPEED_METERS_PER_SECOND;
            //System.out.println("New Input: " + input + ", vTarget: " + vTarget);


            DriveSubsystem.unsetBaselock("drive");
            drive.curvatureDrive(input,  (1-0.5*slowMode.getAsDouble()) * m_rotation.getAsDouble(), turnInPlace );
        }
        
    }
}