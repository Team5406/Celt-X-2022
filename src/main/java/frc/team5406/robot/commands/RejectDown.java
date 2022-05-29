package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;

public class RejectDown extends CommandBase {
    private final IntakeSubsystem intake;
    private final ConveyorSubsystem convey;

    public RejectDown (IntakeSubsystem _intake, ConveyorSubsystem _convey) {
        intake = _intake;
        convey = _convey;
        addRequirements(intake);
        addRequirements(convey);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {    
        IntakeSubsystem.intakeExtend();  
        IntakeSubsystem.setIntakeRollersSpeed(-0.5*Constants.INTAKE_ROLLER_MAX_SPEED);
        ConveyorSubsystem.setConveyorBottomSpeed(-1*Constants.CONVEYOR_MAX_SPEED);
    }

    @Override
    public boolean isFinished() {


    return true;
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.intakeRetract();
        IntakeSubsystem.stopIntakeRollers();
        ConveyorSubsystem.stopConveyor();
    }
}
