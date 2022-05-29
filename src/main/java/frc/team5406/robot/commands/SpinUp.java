package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class SpinUp extends CommandBase {
  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
  private final double flywheelSpeed;
  private final double hoodAngle;

  public SpinUp(ShooterSubsystem subsystem, double _flywheelSpeed, double _hoodAngle) {
    shooter = subsystem;
    flywheelSpeed = _flywheelSpeed;
    hoodAngle = _hoodAngle;
    //addRequirements(shooter);
  }

  @Override
  public void initialize() {
    ShooterSubsystem.stopBooster();
    ConveyorSubsystem.stopConveyorTop();

  }

  @Override
  public void execute() {

    ShooterSubsystem.setShooterSpeed(flywheelSpeed);
    ShooterSubsystem.setHoodPosition(hoodAngle);
  //  IntakeSubsystem.compressorDisabled();

  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Done Spin Up");

    /*
     * ShooterSubsystem.stopBooster(); ShooterSubsystem.stopShooter();
     * ShooterSubsystem.compressorEnabled();
     */

  }
}