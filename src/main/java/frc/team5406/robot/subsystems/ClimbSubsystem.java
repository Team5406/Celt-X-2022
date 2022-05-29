
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   * 
   */
  private static CANSparkMax climbMotor = new CANSparkMax(Constants.MOTOR_CLIMBER_ONE, MotorType.kBrushless);
  private static CANSparkMax climbMotorFollower = new CANSparkMax(Constants.MOTOR_CLIMBER_TWO, MotorType.kBrushless);
  private static RelativeEncoder climbEncoder,climbEncoder2;
  private static SparkMaxPIDController climbPID, climbPID2;
  static SimpleMotorFeedforward climbFF = new SimpleMotorFeedforward(Constants.CLIMBER_KS, Constants.CLIMBER_KV, Constants.CLIMBER_KA);
  private static ProfiledPIDController climbPIDController = new ProfiledPIDController(Constants.CLIMBER_PID4_P*10, Constants.CLIMBER_PID4_I, 0, 
  new Constraints(5000, 500));
  //private final static DistanceReader m_sensor1 = new DistanceReader(0, 0);
  //private final static DistanceReader m_sensor2 = new DistanceReader(0, 1);


  private static Solenoid climbHookOne, climbHookTwo;

  private static int climbState;

  public static void setupMotors() {
    climbMotorFollower.follow(climbMotor, true);
    climbMotor.setInverted(false);

    climbHookOne = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_CLIMB_HOOK_ONE);
    climbHookTwo = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_CLIMB_HOOK_TWO);

    climbMotor.setIdleMode(IdleMode.kCoast);
    climbMotorFollower.setIdleMode(IdleMode.kCoast);

    climbMotorFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CLIMBER);
    climbMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CLIMBER);

    climbEncoder = climbMotor.getEncoder();
    climbPID = climbMotor.getPIDController();

    climbEncoder2 = climbMotorFollower.getEncoder();
    climbPID2 = climbMotorFollower.getPIDController();

    climbPID.setP(Constants.CLIMBER_PID1_P, 1);
    climbPID.setI(Constants.CLIMBER_PID1_I, 1);
    climbPID.setD(Constants.CLIMBER_PID1_D, 1);
    climbPID.setIZone(0, 1);
    climbPID.setFF(Constants.CLIMBER_PID1_F, 1);
    climbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);


    climbPID.setP(Constants.CLIMBER_PID2_P, 2);
    climbPID.setI(Constants.CLIMBER_PID2_I, 2);
    climbPID.setD(Constants.CLIMBER_PID2_D, 2);
    climbPID.setIZone(0, 2);
    climbPID.setFF(Constants.CLIMBER_PID2_F, 2);
    climbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 2);

    climbPID2.setP(Constants.CLIMBER_PID2_P, 2);
    climbPID2.setI(Constants.CLIMBER_PID2_I, 2);
    climbPID2.setD(Constants.CLIMBER_PID2_D, 2);
    climbPID2.setIZone(0, 2);
    climbPID2.setFF(Constants.CLIMBER_PID2_F, 2);
    climbPID2.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 2);

    climbPID.setP(Constants.CLIMBER_PID3_P, 3);
    climbPID.setI(Constants.CLIMBER_PID3_I, 3);
    climbPID.setD(Constants.CLIMBER_PID3_D, 3);
    climbPID.setIZone(0, 3);
    climbPID.setFF(Constants.CLIMBER_PID3_F, 3);
    climbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 3);

    climbPID.setP(Constants.CLIMBER_PID3_P, 4);
    climbPID.setI(Constants.CLIMBER_PID3_I, 4);
    climbPID.setD(Constants.CLIMBER_PID3_D, 4);
    climbPID.setIZone(0, 4);
    climbPID.setFF(Constants.CLIMBER_PID3_F, 4);
    climbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 4);


    climbPID.setSmartMotionMaxVelocity(4500, 4);
    climbPID.setSmartMotionMaxAccel(5000, 4);
    climbPID.setSmartMotionAllowedClosedLoopError(1, 4);


    resetPosition();

    climbMotor.burnFlash();
    climbMotorFollower.burnFlash();

    climbPIDController.setTolerance(5);

  }

  public static void setClimberPosition(double angle) {
    angle = angle/360 * Constants.GEAR_RATIO_CLIMBER;
    double pos = getClimberPosition();
    double arbFF;
    double pidOutput = climbPIDController.calculate(pos, angle);
    arbFF = climbFF.calculate(pidOutput);
    SmartDashboard.putNumber("Climb angle", angle);
    SmartDashboard.putNumber("Climb position", pos);
    SmartDashboard.putNumber("Climb arbFF", arbFF);
    SmartDashboard.putNumber("Climb pidOutput", pidOutput);
    
    climbPID.setReference(angle, ControlType.kPosition, 4, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public static void setClimberExtendSpeed() {

  }

  public static void stopClimberExtend() {

  }
/*
  public static void displayDistanceSensors(){
    SmartDashboard.putData("Sensor 1 Distance", m_sensor1);
    SmartDashboard.putData("Sensor 2 Distance", m_sensor2);
}*/

public static void displayClimbPositions(){
  getClimberPosition();
  getClimberPosition2();
}


  public static int getClimbState() {
    SmartDashboard.putNumber("Climb State", climbState);

    return climbState;
  }

  public static void setClimbState(int num) {
    climbState = num;
  }

  public static void openHook(int num){
    if(num == 1){
      climbHookOne.set(Constants.CLIMB_HOOK_ONE_RETRACT);
    }else if(num == 2){
      climbHookTwo.set(Constants.CLIMB_HOOK_TWO_RETRACT);
    }
  }
  public static void closeHook(int num){
    if(num == 1){
      climbHookOne.set(Constants.CLIMB_HOOK_ONE_EXTEND);
    }else if(num == 2){
      climbHookTwo.set(Constants.CLIMB_HOOK_TWO_EXTEND);
    }
  }

  public static void setRotateClimberSpeed(double RPM) {
    if (RPM == 0) {
      stopClimber();
    } else {
      getClimberVelocity();
      double arbFF = climbFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
      climbPID.setReference(RPM * Constants.GEAR_RATIO_CLIMBER, ControlType.kVelocity, 3, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }
  }

  public static void setRotateClimberPosition(double deltaRevs) {
      double position = deltaRevs + getClimberPosition();  
      SmartDashboard.putNumber("Climb Target", position);
      SmartDashboard.putNumber("Climb Pos", getClimberPosition());
      SmartDashboard.putNumber("Climb Delta", deltaRevs);

      climbPID.setReference(position, ControlType.kPosition, 2);
      climbPID2.setReference(position, ControlType.kPosition, 2);
  }

  public static void stopClimber() {
   // System.out.println("Stop Climber");
   double position = getClimberPosition();  
   double position2 = getClimberPosition2();  

   climbPID.setReference(position, ControlType.kPosition, 2);
   climbPID2.setReference(position2, ControlType.kPosition, 2);

    /*climbMotor.set(0);
    climbMotorFollower.set(0);*/
  }


  // returns the position
  public static double getClimberPosition() {
    SmartDashboard.putNumber("Climb 1 Pos", climbEncoder.getPosition());
    return climbEncoder.getPosition();
  }

  public static double getClimberPosition2() {
    SmartDashboard.putNumber("Climb 2 Pos", climbEncoder2.getPosition());
    return climbEncoder2.getPosition();
  }

  public static double getClimberVelocity() {
    SmartDashboard.putNumber("Climber Vel", climbEncoder.getVelocity());
    return climbEncoder.getVelocity();
  }

  public static void resetPosition() {
    //System.out.println("Reset Climb Position");
    climbEncoder.setPosition(0);
    climbEncoder2.setPosition(0);
    climbMotor.setVoltage(-0.5);
    climbMotorFollower.setVoltage(-0.5);
  }

  /*public static double distanceReading(int sensor){
    if(sensor==1){
      return m_sensor1.getRange();
    }else{
      return m_sensor2.getRange();
    }
  }*/

  public ClimbSubsystem() {
    setupMotors();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb 1 Pos", climbEncoder.getPosition());
    SmartDashboard.putNumber("Climb 1 Vel", climbEncoder.getVelocity());
  }
}