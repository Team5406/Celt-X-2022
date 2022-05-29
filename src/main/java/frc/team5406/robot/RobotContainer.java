package frc.team5406.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.wpilibj.XboxController.Button;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.subsystems.*;
import frc.team5406.robot.autos.*;
import frc.team5406.robot.commands.*;
import frc.team5406.robot.triggers.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  public final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final DriveStraight driveStraight = new DriveStraight(m_robotDrive);
  private final OneBall oneBall = new OneBall(m_robotDrive, m_shooterSubsystem, m_conveyorSubsystem);
  private final TwoBall twoBall = new TwoBall(m_robotDrive, m_shooterSubsystem, m_conveyorSubsystem, m_intakeSubsystem);
  private final ThreeBallRight threeBallRight = new ThreeBallRight(m_robotDrive, m_shooterSubsystem, m_conveyorSubsystem, m_intakeSubsystem);
  private final FourBallLeft fourBallLeft = new FourBallLeft(m_robotDrive, m_shooterSubsystem, m_conveyorSubsystem, m_intakeSubsystem);
  private final FourBallRight fourBallRight = new FourBallRight(m_robotDrive, m_shooterSubsystem, m_conveyorSubsystem, m_intakeSubsystem);
  private final FiveBallRight fiveBallRight = new FiveBallRight(m_robotDrive, m_shooterSubsystem, m_conveyorSubsystem, m_intakeSubsystem);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  

  // The driver's controller
  XboxController operatorGamepad = new XboxController(Constants.OPERATOR_CONTROLLER);
  XboxController driverGamepad = new XboxController(Constants.DRIVER_CONTROLLER);

  Trigger driverLeftTrigger = new TriggerPressed(driverGamepad::getLeftTriggerAxis);
  Trigger driverRightTrigger = new TriggerPressed(driverGamepad::getRightTriggerAxis);
  Trigger operatorLeftTrigger = new TriggerPressed(operatorGamepad::getLeftTriggerAxis);
  Trigger operatorRightTrigger = new TriggerPressed(operatorGamepad::getRightTriggerAxis);
  Trigger operatorJoystickRight = new JoystickMoved(operatorGamepad::getRightY);
  Trigger operatorDPad = new DPadPressed(operatorGamepad::getPOV);
  Trigger operatorTurret= new JoystickMoved(operatorGamepad::getLeftX);
  Trigger climbState3 = new ClimbState(3);
  Trigger climbState5 = new ClimbState(5);
  Trigger climbStateAborted = new ClimbState(-1);
  Trigger resetHoodTrigger = new ResetHood();
  JoystickButton operatorLeftBumper = new JoystickButton(operatorGamepad, Button.kLeftBumper.value);
  JoystickButton operatorRightModifier = new JoystickButton(operatorGamepad, Button.kStart.value);
  JoystickButton operatorLeftModifier = new JoystickButton(operatorGamepad, Button.kBack.value);
  JoystickButton operatorRightBumper = new JoystickButton(operatorGamepad, Button.kRightBumper.value);
  JoystickButton operatorYButton = new JoystickButton(operatorGamepad, Button.kY.value);
  JoystickButton operatorAButton = new JoystickButton(operatorGamepad, Button.kA.value);
  JoystickButton operatorBButton = new JoystickButton(operatorGamepad, Button.kB.value);
  JoystickButton operatorXButton = new JoystickButton(operatorGamepad, Button.kX.value);
  JoystickButton driverLeftBumper = new JoystickButton(driverGamepad, Button.kLeftBumper.value);
  JoystickButton driverRightBumper = new JoystickButton(driverGamepad, Button.kRightBumper.value);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new TeleopDrive(m_robotDrive, driverGamepad::getLeftY, driverGamepad::getRightX, true, driverGamepad::getLeftTriggerAxis, operatorGamepad::getXButton)
    );

    m_intakeSubsystem.setDefaultCommand(
        new Intake(m_intakeSubsystem, driverGamepad::getRightTriggerAxis, false, false, driverGamepad::getLeftBumper)
    );
    m_conveyorSubsystem.setDefaultCommand(
        new Convey(m_conveyorSubsystem, driverGamepad::getRightTriggerAxis, false, false, false,  driverGamepad::getLeftBumper)
    );

    
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Drive Straight Auto", driveStraight.getAutonomousCommand());
    m_chooser.addOption("OneBall", oneBall.getAutonomousCommand());
    m_chooser.addOption("TwoBall", twoBall.getAutonomousCommand());
    m_chooser.addOption("ThreeBallRight", threeBallRight.getAutonomousCommand());
    m_chooser.addOption("FourBallLeft", fourBallLeft.getAutonomousCommand());
    m_chooser.addOption("FourBallRight", fourBallRight.getAutonomousCommand());
    m_chooser.addOption("FiveBallRight", fiveBallRight.getAutonomousCommand());

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * JoystickButton}.
   */
  private void configureButtonBindings() {

    //outake
    driverRightTrigger.and(driverLeftBumper).whileActiveContinuous(
      new ParallelCommandGroup(
        new Intake(m_intakeSubsystem, driverGamepad::getRightTriggerAxis, true, false ,driverGamepad::getLeftBumper),
        new Convey(m_conveyorSubsystem, driverGamepad::getRightTriggerAxis, true, false, false,driverGamepad::getLeftBumper)
      )
    );


    //shoot and convey
    operatorXButton.whileActiveContinuous(
      new ParallelCommandGroup(
        new Shoot(true, operatorGamepad::getRightBumper),
        new Convey(m_conveyorSubsystem, operatorGamepad::getRightTriggerAxis, false, false, true,operatorGamepad::getLeftBumper)
      )
    );

    //manual - analog intake
    operatorRightTrigger.and(operatorRightModifier).whileActiveContinuous (
      new Intake(m_intakeSubsystem, operatorGamepad::getRightTriggerAxis, false, true,operatorGamepad::getLeftBumper)
    );

    //manual - analog outake
    operatorLeftTrigger.and(operatorRightModifier).whileActiveContinuous( 
      new Intake(m_intakeSubsystem, operatorGamepad::getLeftTriggerAxis, true, true,operatorGamepad::getLeftBumper)
    );

    //manual - analog conveyor up
    operatorRightTrigger.and(operatorLeftModifier).whileActiveContinuous( 
      new Convey(m_conveyorSubsystem, operatorGamepad::getRightTriggerAxis, false, true, false,operatorGamepad::getLeftBumper)
    );

    //manual - analog conveyor down
    operatorLeftTrigger.and(operatorLeftModifier).whileActiveContinuous( 
      new Convey(m_conveyorSubsystem, operatorGamepad::getLeftTriggerAxis, true, true, false,operatorGamepad::getLeftBumper)
    );

    //manual - move climber foward and backwards
    climbStateAborted.and(operatorJoystickRight).and(operatorLeftBumper).and(operatorLeftModifier).whileActiveContinuous(
      new RotateClimber(m_climbSubsystem, operatorGamepad::getRightY)
    );
  
    //ballSensorOne.whileActiveContinuous(new Rumble(driverGamepad, true, true)).whenInactive(new Rumble(driverGamepad, true, false));
    //ballSensorTwo.whileActiveContinuous(new Rumble(driverGamepad, false, true)).whenInactive(new Rumble(driverGamepad, false, false));


    //deploy climber
    operatorRightBumper.and(operatorLeftModifier).and(operatorXButton).whenActive(
      new SequentialCommandGroup(
        new ClimbSensorless(m_climbSubsystem, 30),
        new ClimbSensorless(m_climbSubsystem, 0)
      )
    );

    //abort climb
    operatorLeftBumper.and(operatorLeftModifier).whenActive(
      new AbortClimb(m_climbSubsystem)
      );

    operatorRightBumper.and(operatorLeftModifier).and(operatorBButton).whenActive(
      new SequentialCommandGroup(
        new ClimbSensorless(m_climbSubsystem, -170 *1.125),
        new ClimbSensorless(m_climbSubsystem, -110 *1.125),
        new SetHook(m_climbSubsystem, 2, Constants.CLIMBER_RETRACT),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
          new ClimbSensorless(m_climbSubsystem, -320 *1.125),
          new SequentialCommandGroup(
            new WaitCommand(1),
            new SetHook(m_climbSubsystem, 2, Constants.CLIMBER_EXTEND)
          )
        ),
        new WaitCommand(0.5),
        //new ClimbSensorless(m_climbSubsystem, -280 *1.125), //274
        new SetHook(m_climbSubsystem, 1, Constants.CLIMBER_RETRACT)
        //new WaitCommand(1),
        //new ClimbSensorless(m_climbSubsystem, -310 *1.125)

      )
    );

    /*operatorRightBumper.and(operatorLeftModifier).and(operatorBButton).whenActive(
      new SequentialCommandGroup(
        new ClimbSensorless(m_climbSubsystem, -175*1.125),
        new ClimbSensorless(m_climbSubsystem, -105*1.125),
        new ClimbSensorless(m_climbSubsystem, -335*1.125),
        new ClimbSensorless(m_climbSubsystem, -270*1.125),
        new ClimbSensorless(m_climbSubsystem, -336*1.125)
      )
    );*/
    
    operatorRightBumper.and(operatorLeftModifier).and(operatorAButton).whenActive(
      new SequentialCommandGroup( 
      new ClimbSensorless(m_climbSubsystem, -170 *1.125),
      new ClimbSensorless(m_climbSubsystem, -110 *1.125)
      )
  );
    
    //Retract Hook 1, set climb state to 4
    climbState3.and(operatorLeftBumper).and(operatorRightModifier).whenActive(
      new SetHook(m_climbSubsystem, 1, Constants.CLIMBER_RETRACT)
      .andThen(new SetClimbState(m_climbSubsystem, 4))
    );

    //Retract Hook 2, set climb state to 6
    climbState5.and(operatorLeftBumper).and(operatorRightModifier).whenActive(
      new SetHook(m_climbSubsystem, 2, Constants.CLIMBER_RETRACT)
      .andThen(new SetClimbState(m_climbSubsystem, 6))
    );

    //
    climbStateAborted.and(operatorLeftBumper).and(operatorLeftModifier).and(operatorYButton).whenActive(
      new SetHook(m_climbSubsystem, 1, Constants.CLIMBER_RETRACT)
    ).whenInactive(
      new SetHook(m_climbSubsystem, 1, Constants.CLIMBER_EXTEND)
    );
    
    
    climbStateAborted.and(operatorLeftBumper).and(operatorLeftModifier).and(operatorAButton).whenActive(
      new SetHook(m_climbSubsystem, 2, Constants.CLIMBER_RETRACT)
    ).whenInactive(
      new SetHook(m_climbSubsystem, 2, Constants.CLIMBER_EXTEND)
    );

    operatorRightTrigger.and(operatorRightModifier.negate()).whileActiveContinuous (
      new ParallelCommandGroup(
        new AlignTurret(m_shooterSubsystem),
        new SetShooter(m_shooterSubsystem, operatorGamepad::getRightTriggerAxis, operatorGamepad, operatorGamepad::getRightBumper, operatorGamepad::getXButton)
      )
    );


    /*operatorShootButton.whileActiveContinuous (
      new Shoot(m_shooterSubsystem)
    );*/

    /*operatorBButton.and(operatorRightBumper.negate()).whileActiveContinuous(
      new TrackWhileDriving(m_robotDrive, m_shooterSubsystem)
    );*/

    operatorTurret.whileActiveContinuous (
      new TurnTurret(m_shooterSubsystem, operatorGamepad::getLeftX)
    );

    operatorDPad.whenActive (
      new SetTurretPosition(m_shooterSubsystem, operatorGamepad::getPOV)
    );

    /*operatorShootButton.whileActiveContinuous(
      new Shoot(true) 
    );
*/

    resetHoodTrigger.whileActiveContinuous(
      new ResetHoodEncoder(m_shooterSubsystem)
    );
    

    

    operatorLeftTrigger.and(operatorRightModifier.negate()).whileActiveContinuous (
      new LowLob(m_shooterSubsystem, operatorGamepad::getLeftTriggerAxis)
    );

    //Highlob
    operatorRightTrigger.and(operatorLeftBumper).whileActiveContinuous (
      new HighLob(m_shooterSubsystem, operatorGamepad::getRightTriggerAxis)
    );

    /*operatorLeftBumper.and(operatorLeftModifier.negate()).whileActiveContinuous(
      new RejectDown(m_intakeSubsystem, m_conveyorSubsystem)
    );*/
    driverRightBumper.whileActiveContinuous(
      new ParallelCommandGroup(
      new SetShooter(m_shooterSubsystem, operatorGamepad::getRightTriggerAxis, operatorGamepad, driverGamepad::getRightBumper, operatorGamepad::getXButton),
      new Shoot(false, driverGamepad::getRightBumper)
      )
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_chooser.getSelected();

  }

}