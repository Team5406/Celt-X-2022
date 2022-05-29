// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public static Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {

    //Retrieve the selected auto from RobotContainer.java 
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    DriveSubsystem.setBrakeMode(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
      ShooterSubsystem.stopBooster();
      ShooterSubsystem.setShooterSpeed(0);
      ShooterSubsystem.setHoodPosition(0);
      ConveyorSubsystem.stopConveyorTop();
    ClimbSubsystem.resetPosition();
    DriveSubsystem.setNormalMode();
    DriveSubsystem.setHeading();
    DriveSubsystem.setPose(new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(-24), Rotation2d.fromDegrees(0)));
    DriveSubsystem.resetOdometry(new Pose2d(9, 1, Rotation2d.fromDegrees(-90)));
    ShooterSubsystem.turnOffLimelight();
    ConveyorSubsystem.stopConveyorBottom();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Hood Angle", ShooterSubsystem.getHoodAngle());
    SmartDashboard.putNumber("Turret Position", ShooterSubsystem.getTurretPosition());

    SmartDashboard.putNumber("Flywheel Speed", ShooterSubsystem.getShooterSpeed());
    SmartDashboard.putNumber("Booster Speed", ShooterSubsystem.getBoosterSpeed());
    
    SmartDashboard.putNumber("Compressor Pressure", IntakeSubsystem.getPressureValue());

  }

  @Override
  public void disabledInit() {
    ShooterSubsystem.hoodReset = false; 
    DriveSubsystem.setBrakeMode(false);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
     // Cancels all running commands at the start of test mode.
     CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
