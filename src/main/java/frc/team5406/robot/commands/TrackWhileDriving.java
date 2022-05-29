// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.DriveSubsystem;

/* tracks target like TrackTargetStationary command but accounts for the robots movement by leading the shot
so that you can shoot while moving*/
public class TrackWhileDriving extends CommandBase {
  DriveSubsystem drive;
  ShooterSubsystem shooter;

  //outputs of command
  //double desiredTurretV = 0, desiredTurretAngle = 0;
  //double hoodAdjust = 0, desiredHoodAngle = 0;

  //from chassis
  double paraV = 0, perpV = 0, angV = 0, vx = 0, vy = 0;
  
  //from limelight
  double rawDistance = 3, x = 0, y = 0, lastX = 0, lastY = 0;

  double gyroYaw = 0, ax = 0, ay = 0;

  double turretAngle = 0;
  //boolean isAllianceBall = true;

  //leading stuff
  double offsetAngle = 0; //radians
  double effectiveDistance = 3; //meters
  double actualDistance;
  double airTime = 3; //seconds
  double temp = 0;

  //approximating derivative stuff
  double prevOffset = offsetAngle, offsetDerivative = 0;
  double prevAirtime = airTime, airTimeDerivative = 0;
  double effectiveDistanceDerivative = 0, effectiveDistancePrediction = effectiveDistance;

  //pose stuff
  private Pose2d robotPose = new Pose2d();
  double poseX = 0, poseY = 0;
  //private Transform2d robotToGoal = new Transform2d();
  //private Translation2d robotLocation = new Translation2d();
  //double estimatedActualDistance = 0;


  /** Creates a new TrackTargetLeading. */
  public TrackWhileDriving(DriveSubsystem _drive, ShooterSubsystem _shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = _drive;
    shooter = _shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroYaw = DriveSubsystem.getHeading().getDegrees();
    turretAngle = ShooterSubsystem.getTurretPosition();
    ShooterSubsystem.updateLimelightTracking();
    if (ShooterSubsystem.llHasValidTarget) { //runs when the LL can see the target
      rawDistance = ShooterSubsystem.lldist + Constants.HUB_RADIUS;
      x = ShooterSubsystem.lltx;
      SmartDashboard.putNumber("SWD distance", rawDistance);
      
      DriveSubsystem.updateOdometryWithVision(rawDistance, gyroYaw, turretAngle, x);
    }

    robotPose = DriveSubsystem.getPoseEstimate();
    poseX = robotPose.getX();
    poseY = robotPose.getY();

    SmartDashboard.putNumber("SWD poseX", poseX);
    SmartDashboard.putNumber("SWD poseY", poseY);

    actualDistance = robotPose.getTranslation().getDistance(Constants.HUB_LOCATION);
    SmartDashboard.putNumber("SWD Actual Distance", actualDistance);
    //robotToGoal = new Transform2d(robotPose, goalPose);
    //distance = Math.sqrt(Math.pow(robotToGoal.getX(), 2) + Math.pow(robotToGoal.getY(), 2)); 

    //velocities with respect to target
    paraV = DriveSubsystem.getParaV(turretAngle);
    perpV = DriveSubsystem.getPerpV(turretAngle);

    

    vx = DriveSubsystem.getVx();
    vy = DriveSubsystem.getVy();
    
    angV = DriveSubsystem.getAngV();

    ax = DriveSubsystem.getGyroAccelX();
    ay = DriveSubsystem.getGyroAccelY();


    SmartDashboard.putNumber("SWD paraV", paraV);
    SmartDashboard.putNumber("SWD perpV", perpV);
    SmartDashboard.putNumber("SWD vx", vx);
    SmartDashboard.putNumber("SWD vy", vy);
    SmartDashboard.putNumber("SWD angV", angV);
    SmartDashboard.putNumber("SWD ax", ax);
    SmartDashboard.putNumber("SWD ay", ay);
    SmartDashboard.putNumber("SWD airTime", airTime);


    //guess airtime 0.020 sec into future? linear approximation?
    // airtime = airtime + 0.020 * derivative
      airTime = getAirTime(actualDistance)/5;

    //temp value storing a calculation so that it doesnt have recalc it every time i need the value
    temp = airTime * (vy * (poseX - Constants.HUB_LOCATION.getX()) 
      + vx * (poseY - Constants.HUB_LOCATION.getY()));

    //temp = airTime * (vy * Math.sin(gyroYaw + turretAngle - x) + vx * Math.cos(gyroYaw + turretAngle - x));
    //TODO: compare these temp values and make sure they are close

    //this effectiveDistance calculation uses the offset angle prediction from previous loop
    //effectiveDistance = temp / (actualDistance * Math.sin(offsetAngle));
    //once we know the effective distance, then we can determine what the actual offset angle should be


    Transform2d distanceOffset = new Transform2d(new Translation2d(vy*airTime, vx*airTime), Rotation2d.fromDegrees(0));

    Pose2d effectivePose = robotPose.plus(distanceOffset);
    effectiveDistance = effectivePose.getTranslation().getDistance(Constants.HUB_LOCATION);
    //offsetAngle = Math.atan2(effectivePose.getY(), effectivePose.getX()) - Math.atan2(poseY, poseX);
    offsetAngle = Math.asin(temp / effectiveDistance);
    

        effectiveDistance -= Constants.HUB_RADIUS;


    SmartDashboard.putNumber("SWD temp", temp);
    SmartDashboard.putNumber("SWD offsetAngle", offsetAngle);
    SmartDashboard.putNumber("SWD effectiveDistance", effectiveDistance);


    //take derivate of effective distance formula to get distance adjustment approximation to plug into airtime. a = 0?

    //predictedDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

    //desiredHoodAngle = -3 * distance + 85; //degrees

    //ShooterSubsystem.setHoodPosPID(effectiveDistance, paraV);

    //ShooterSubsystem.setTurretPosPID(x - Units.radiansToDegrees(offsetAngle), perpV, effectiveDistance, angV); 
    SmartDashboard.putNumber("SWD Calc TY", ShooterSubsystem.calcTyFromDist(effectiveDistance));

    SmartDashboard.putNumber("SWD Target Hood Pos", ShooterSubsystem.calculateHoodAngle(ShooterSubsystem.calcTyFromDist(effectiveDistance)));
    SmartDashboard.putNumber("SWD Target Shooter Pos", ShooterSubsystem.calculateShooterSpeed(ShooterSubsystem.calcTyFromDist(effectiveDistance)));
    SmartDashboard.putNumber("SWD Target Turret Pos", x - Units.radiansToDegrees(offsetAngle));
    ShooterSubsystem.setHoodPosition(ShooterSubsystem.calculateHoodAngle(ShooterSubsystem.calcTyFromDist(effectiveDistance)));
    ShooterSubsystem.setShooterSpeed(ShooterSubsystem.calculateShooterSpeed(ShooterSubsystem.calcTyFromDist(effectiveDistance)));
    //ShooterSubsystem.setTurretPosition(x - Units.radiansToDegrees(offsetAngle));
    /*if (!ShooterSubsystem.llHasValidTarget) { //runs when the LL can see the target
      SmartDashboard.putNumber("SWD rotation", robotPose.getRotation().getDegrees());
      SmartDashboard.putNumber("SWD angle calc", Units.radiansToDegrees(Math.atan2(poseX, poseY)));
      SmartDashboard.putNumber("SWD angle calc other way", Units.radiansToDegrees(Math.atan2(poseY, poseX)));
      
      ShooterSubsystem.setTurretPosition(Units.radiansToDegrees(robotPose.getRotation().getRadians() +Math.atan2(poseX, poseY)));
    }else{
      ShooterSubsystem.setMotorPosPID(x - Units.radiansToDegrees(offsetAngle), perpV, effectiveDistance, angV);

    }*/

    if (!ShooterSubsystem.llHasValidTarget){
      x = 0;
    }

      ShooterSubsystem.setTurretPosition((x - Units.radiansToDegrees(offsetAngle)) + Units.radiansToDegrees(robotPose.getRotation().getRadians() +Math.atan2(poseX, poseY)));

    //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
    //shooter.setMotorsVelPID(predictedDistance);
/*
    prevOffset = offsetAngle;
    prevAirtime = airTime;

    //effectiveDistance += 0.020 * derivative gives the effective distance in 0.020 seconds.
    //then you can get the air time and offset angle for the next loop
    airTime = getAirTime(effectiveDistance);

    //these derivatives are very approximate, find better way to do them?
    offsetDerivative = (offsetAngle - prevOffset) / Constants.LOOP_TIME;
    airTimeDerivative = (airTime - prevAirtime) / Constants.LOOP_TIME;

    //found the formula for this by differentiating by hand
    effectiveDistanceDerivative = airTimeDerivative * effectiveDistance / airTime
      + airTime *  ((poseX * ay - poseY * ax) / (actualDistance * Math.sin(offsetAngle)) 
      - (poseX * vy - poseY * vx) * (Math.sin(offsetAngle) * (poseX * vx + poseY * vy) / actualDistance + actualDistance * Math.cos(offsetAngle) * offsetDerivative)) 
      / Math.pow(actualDistance * Math.sin(offsetAngle), 2);

    //predicts what the effective distance will be in 20 ms when the loop runs again
    effectiveDistancePrediction = effectiveDistance + Constants.LOOP_TIME * effectiveDistanceDerivative;

    //predicts airtime for the next loop based on the effective distance prediction
    airTime = getAirTime(effectiveDistancePrediction);
    //predicts the next offset angle 
    offsetAngle = Math.asin((airTime * temp / prevAirtime) / effectiveDistancePrediction);
*/

  }


  double getAirTime(double distance){
    return -0.0238*distance*distance + 0.5952*distance + 1.4286;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}