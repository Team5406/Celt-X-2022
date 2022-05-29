package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class Intake extends CommandBase {
    private final IntakeSubsystem intake;
    private final DoubleSupplier triggerPress;
    private final BooleanSupplier reject;
    private final boolean manual;
    private final boolean reverse;


  
    public Intake (IntakeSubsystem _intake, DoubleSupplier _triggerPress, boolean _reverse, boolean _manual, BooleanSupplier _reject) {
        intake = _intake;
        triggerPress = _triggerPress;
        manual = _manual;
        reverse = _reverse;
        reject = _reject;
        addRequirements(intake);
        
    }
    @Override
    public void execute(){
        if (!manual && triggerPress.getAsDouble() > (1-Constants.TRIGGER_THRESHOLD)) {
          //System.out.println((reverse?"Outake":"Intake") + " - auto");
          IntakeSubsystem.intakeExtend();
          IntakeSubsystem.setIntakeRollersSpeed((reverse?-0.5:1)*Constants.INTAKE_ROLLER_SPEED);
          IntakeSubsystem.lastTime = Timer.getFPGATimestamp();
          IntakeSubsystem.keepSpinning = true;
        }else if (manual && triggerPress.getAsDouble() > Constants.TRIGGER_THRESHOLD){
          //System.out.println((reverse?"Outake":"Intake") + " - manual");
          IntakeSubsystem.intakeExtend();  
          IntakeSubsystem.setIntakeRollersSpeed((reverse?-1:1)*triggerPress.getAsDouble()*Constants.INTAKE_ROLLER_MAX_SPEED);
          IntakeSubsystem.keepSpinning = false;
        }else if (reject.getAsBoolean()){
          IntakeSubsystem.intakeExtend();  
          IntakeSubsystem.setIntakeRollersSpeed(-0.5*Constants.INTAKE_ROLLER_MAX_SPEED);
          IntakeSubsystem.keepSpinning = false;
        }else{
          //System.out.println("Stop Intake");
          IntakeSubsystem.intakeRetract();
        
          if (!(IntakeSubsystem.keepSpinning && (Timer.getFPGATimestamp() - IntakeSubsystem.lastTime) < Constants.KEEP_SPINNING_TIME_INTAKE)){
            IntakeSubsystem.stopIntakeRollers();
          }
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Intake - End");
     /* IntakeSubsystem.intakeRetract();
      IntakeSubsystem.stopIntakeRollers();*/
    }
}