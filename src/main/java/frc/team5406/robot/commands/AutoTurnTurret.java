package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AutoTurnTurret extends CommandBase {

  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
  private final double turretAngle;

  public AutoTurnTurret(ShooterSubsystem subsystem, double angle) {
    shooter = subsystem;
    turretAngle = angle;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    ShooterSubsystem.setTurretPosition(turretAngle);
  }

  @Override
  public boolean isFinished() {
    return (ShooterSubsystem.getTurretPosition() > turretAngle - 1)
        && (ShooterSubsystem.getTurretPosition() < turretAngle + 1) && (ShooterSubsystem.getTurretVelocity() < 1);

  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("Turn Turret Done");
  }
}