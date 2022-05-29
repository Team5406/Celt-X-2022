package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    private static CANSparkMax shooter = new CANSparkMax(Constants.MOTOR_SHOOTER_WHEEL_ONE, MotorType.kBrushless);
    private static CANSparkMax shooterFollower = new CANSparkMax(Constants.MOTOR_SHOOTER_WHEEL_TWO,  MotorType.kBrushless);
    private static CANSparkMax booster = new CANSparkMax(Constants.MOTOR_BOOSTER_ONE, MotorType.kBrushless);
    private static CANSparkMax hood = new CANSparkMax(Constants.MOTOR_HOOD_ONE, MotorType.kBrushless);
    private static CANSparkMax turret = new CANSparkMax(Constants.MOTOR_TURRET_AZIMUTH_ONE, MotorType.kBrushless);

    private static CANCoder turretAbsoluteEncoder;
    private static RelativeEncoder shooterEncoder, boosterEncoder, hoodEncoder, turretEncoder;
    private static SparkMaxPIDController shooterPID, boosterPID, hoodPID, turretPID;

    static SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);
    static SimpleMotorFeedforward boosterFF = new SimpleMotorFeedforward(Constants.BOOSTER_KS, Constants.BOOSTER_KV, Constants.BOOSTER_KA);
    static SimpleMotorFeedforward hoodFF = new SimpleMotorFeedforward(Constants.HOOD_KS, Constants.HOOD_KV, Constants.HOOD_KA);
    static SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(Constants.TURRET_KS, Constants.TURRET_KV, Constants.TURRET_KA);
    static SimpleMotorFeedforward turretFFDeg = new SimpleMotorFeedforward(0.38569, 0.02015, 0.0080983);
    public static boolean hoodReset = false;

    private static ProfiledPIDController posPIDController = new ProfiledPIDController(5, 0, 0.00000, 
    new Constraints(Constants.TURRET_MAX_V, Constants.TURRET_MAX_A/2));


    private static double tangentialFeedforward = 0;
    private static double rotationalFeedforward = 0;

    
  private static boolean resettingForward = false;
  private static boolean resettingBackward = false;


    public static double distanceToHub = 0;
    public static double turretAngleToHub = 0;

    private static double flywheelRPM = Constants.FLYWHEEL_SPEED_FENDER_HIGH; 
    public static boolean llHasValidTarget = false;
    /*public static double llSteer = 0.0;
  
    public static double llLastError = 0; 
    public static double llTotalError = 0;

    private static double hoodAngle = 0;
    private static double rpm = Constants.FLYWHEEL_SPEED_FENDER_HIGH;*/

    public static double lltv = 0;
    public static double lltx = 0;
    public static double llty = 0;
    public static double llts = 0;
    public static double lldist = 0;
    

    public static void setupMotors() {
        shooterEncoder = shooter.getEncoder();
        boosterEncoder = booster.getEncoder();
        hoodEncoder = hood.getEncoder();
        turretEncoder = turret.getEncoder();

        turretAbsoluteEncoder = new CANCoder(Constants.TURRET_ENCODER);

        shooterPID = shooter.getPIDController();
        boosterPID = booster.getPIDController();
        hoodPID = hood.getPIDController();
        turretPID = turret.getPIDController();

        shooter.setSmartCurrentLimit(Constants.CURRENT_LIMIT_SHOOTER_WHEEL);
        shooterFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_SHOOTER_WHEEL);
        booster.setSmartCurrentLimit(Constants.CURRENT_LIMIT_BOOSTER);
        hood.setSmartCurrentLimit(Constants.CURRENT_LIMIT_HOOD);
        turret.setSmartCurrentLimit(Constants.CURRENT_LIMIT_TURRET_AZIMUTH);

        shooterPID.setP(Constants.SHOOTER_PID0_P, 0);
        shooterPID.setI(Constants.SHOOTER_PID0_I, 0);
        shooterPID.setD(Constants.SHOOTER_PID0_D, 0);
        shooterPID.setIZone(0, 0);
        shooterPID.setFF(Constants.SHOOTER_PID0_F, 0);
        shooterPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
        boosterPID.setP(Constants.BOOSTER_PID0_P);
        boosterPID.setI(Constants.BOOSTER_PID0_I, 0);
        boosterPID.setD(Constants.BOOSTER_PID0_D, 0);
        boosterPID.setIZone(0, 0);
        boosterPID.setFF(Constants.BOOSTER_PID0_F, 0);
        boosterPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
        hoodPID.setP(Constants.HOOD_PID0_P, 0);
        hoodPID.setI(Constants.HOOD_PID0_I, 0);
        hoodPID.setD(Constants.HOOD_PID0_D, 0);
        hoodPID.setIZone(0, 0);
        hoodPID.setFF(Constants.HOOD_PID0_F, 0);
        hoodPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
        hoodPID.setP(Constants.HOOD_PID1_P, 1);
        hoodPID.setI(Constants.HOOD_PID1_I, 1);
        hoodPID.setD(Constants.HOOD_PID1_D, 1);
        hoodPID.setIZone(0, 1);
        hoodPID.setFF(Constants.HOOD_PID1_F, 1);
        hoodPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);
        hoodPID.setSmartMotionAllowedClosedLoopError(0.05, 1);
        turretPID.setP(Constants.TURRET_PID0_P, 0);
        turretPID.setI(Constants.TURRET_PID0_I, 0);
        turretPID.setD(Constants.TURRET_PID0_D, 0);
        turretPID.setIZone(0, 0);
        turretPID.setFF(Constants.TURRET_PID0_F, 0);

        turretPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
        turretPID.setP(Constants.TURRET_PID1_P, 1);
        turretPID.setI(Constants.TURRET_PID1_I, 1);
        turretPID.setD(Constants.TURRET_PID1_D, 1);
        turretPID.setIZone(0, 1);
        turretPID.setFF(Constants.TURRET_PID1_F, 1);
        turretPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);
        turretPID.setP(Constants.TURRET_PID2_P, 2);
        turretPID.setI(Constants.TURRET_PID2_I, 2);
        turretPID.setD(Constants.TURRET_PID2_D, 2);
        turretPID.setIZone(0, 2);
        turretPID.setFF(Constants.TURRET_PID2_F, 2);
        turretPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 2);
   
        turretPID.setP(Constants.TURRET_PID3_P, 3);
        turretPID.setI(Constants.TURRET_PID3_I, 3);
        turretPID.setD(Constants.TURRET_PID3_D, 3);
        turretPID.setIZone(0, 3);
        turretPID.setFF(Constants.TURRET_PID3_F, 3);
        turretPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 3);


        turretPID.setSmartMotionMaxVelocity(8000, 1);
        turretPID.setSmartMotionMaxAccel(12000, 1);
        turretPID.setSmartMotionAllowedClosedLoopError(0.1, 1);
        turretPID.setSmartMotionMaxVelocity(8000, 2);
        turretPID.setSmartMotionMaxAccel(12000, 2);
        turretPID.setSmartMotionAllowedClosedLoopError(0.5, 3);


        shooter.setInverted(false);
        booster.setInverted(true);
        hood.setInverted(true);
        shooterFollower.follow(shooter, true);

        turret.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.GEAR_RATIO_TURRET*180/360);
        turret.setSoftLimit(SoftLimitDirection.kReverse, -1*(float)Constants.GEAR_RATIO_TURRET*180/360);
        hood.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.GEAR_RATIO_HOOD*40/360);
        hood.setSoftLimit(SoftLimitDirection.kReverse, 0);
        
        resetEncoders();
        shooter.burnFlash();
        shooterFollower.burnFlash();
        booster.burnFlash();
        hood.burnFlash();
        turret.burnFlash();

        SmartDashboard.putNumber("Shooter Target RPM", 1000);
        SmartDashboard.putNumber("Booster Target RPM", 1000);
        SmartDashboard.putNumber("Hood Position", 0);


    posPIDController.enableContinuousInput(-180, 180);
    posPIDController.setTolerance(1.5);


    }

    public static void turnOffLimelight(){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }
    
    public static void turnOnLimelight(){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public static void resetEncoders(){
        shooterEncoder.setPosition(0);
        boosterEncoder.setPosition(0);
        turretEncoder.setPosition(0);
        hoodEncoder.setPosition(0);

        turretEncoder.setPosition((getAbsTurretPosition()*Constants.GEAR_RATIO_TURRET)/360);

        turret.setSoftLimit(SoftLimitDirection.kForward, (float) (Constants.TURRET_CW_LIMIT));
        turret.setSoftLimit(SoftLimitDirection.kReverse, (float) (Constants.TURRET_CCW_LIMIT));
        turret.enableSoftLimit(SoftLimitDirection.kForward, true);
        turret.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        hood.setSoftLimit(SoftLimitDirection.kForward, (float) (Constants.HOOD_MAX_LIMIT));
        hood.setSoftLimit(SoftLimitDirection.kReverse, (float) (Constants.HOOD_MIN_LIMIT));
        hood.enableSoftLimit(SoftLimitDirection.kForward, true);
        hood.enableSoftLimit(SoftLimitDirection.kReverse, true);

    }
    public static void setShooterSpeed(double RPM) {

      //RPM*=0.98;
        if (RPM == 0) {
          stopShooter();
  
      } else {
        double arbFF = shooterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
        shooterPID.setReference(RPM *  Constants.GEAR_RATIO_SHOOTER, ControlType.kVelocity, 1, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
      }
  
    }

    public static void stopShooter() {
        shooter.set(0);
    }

    public static void setBoosterSpeed(double RPM){
        if (RPM == 0) {
            booster.set(0);
          } else {
            double arbFF = boosterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            boosterPID.setReference(RPM *  Constants.GEAR_RATIO_BOOSTER, ControlType.kVelocity, 1, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
          }      

        }

    public static void stopBooster() {
        booster.set(0);
    }

    public static void setHoodPosition(double angle){
        hoodPID.setReference(angle/360 *  Constants.GEAR_RATIO_HOOD, ControlType.kPosition, 1);  
    }

    public static double getHoodCurrent(){
        return hood.getOutputCurrent();
    }
    
    public static double getHoodVelocity(){
      return hoodEncoder.getVelocity();
    }

    //Used to reset the hood
    public static void setHoodSpeed(double speed){
        //TODO: Ensure right direction of hood
         hood.set(speed);
       // hoodPID.setReference(speed, ControlType.kVelocity, 1);
        
    }

    public static void stopHood(){
        hood.set(0);
    }

    public static void resetHood() {
        hoodEncoder.setPosition(0);
        hoodReset = true;
    }

    public static void disableHoodLimits(){
      hood.enableSoftLimit(SoftLimitDirection.kForward, false);
      hood.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public static void enableHoodLimits(){
      hood.enableSoftLimit(SoftLimitDirection.kForward, true);
      hood.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public static void setTurretSpeed(double RPM){
        if (RPM == 0) {
            stopTurret();
          } else {
            double arbFF = turretFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            turretPID.setReference(RPM *  Constants.GEAR_RATIO_TURRET, ControlType.kVelocity, 0, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);

        }  
    }

    public static void setTurretPosition(double angle){
        if(angle == 180 && getTurretPosition() < 0){ //generalize this
            angle = -180;
        }
        if (angle > Constants.TURRET_ANGLE_THRESHOLD){
            angle -=360;
        }else if (angle < -1 * Constants.TURRET_ANGLE_THRESHOLD) {
            angle +=360;
        }

       // double arbFF = turretFF.calculate(Math.signum(angle)*Constants.TURRET_MAX_SPEED/Constants.SECONDS_PER_MINUTE);
        turretPID.setReference(angle/360 *  Constants.GEAR_RATIO_TURRET, ControlType.kSmartMotion, 1);        
      }
      

      public static double getTurretVelocity() {

        return turretEncoder.getVelocity() * 1 / Constants.GEAR_RATIO_TURRET;
      }

      public static double getAbsTurretPosition(){
        return getTurretAngleFromAbs(turretAbsoluteEncoder.getAbsolutePosition());
      }
    
      public static double getTurretAngleFromAbs(double absPos){
        if(absPos > 180){
          absPos -= 360;
        }
        return absPos/Constants.GEAR_RATIO_TURRET_ENCODER;
      }

      public static void holdBoosterPosition() {
        boosterPID.setReference(boosterEncoder.getPosition(), ControlType.kPosition, 1);        
    }


    public static void stopTurret(){
            //turret.set(0);
            turretPID.setReference(turretEncoder.getPosition(), ControlType.kPosition, 1);        

           // turretPID.setReference(turretEncoder.getPosition(), ControlType.kPosition);
    }

    public static void LLAlignTurret(){
      updateLimelightTracking();
        if (llHasValidTarget) {
          double angle = getTurretPosition() + lltx;
          if (angle > Constants.TURRET_ANGLE_THRESHOLD){
              angle -=360;
          }else if (angle < -1 * Constants.TURRET_ANGLE_THRESHOLD) {
              angle +=360;
          }
          double position = angle/360 *  Constants.GEAR_RATIO_TURRET;

       //   System.out.println("Cur: " + getTurretPosition() + ", lltx: " +  lltx + ", pos: " + position);

          SmartDashboard.putNumber("Turret Target", position);
          turretPID.setReference(position, ControlType.kPosition, 3);
          }

       /* updateLimelightTracking();
        if (llHasValidTarget) {

          double pos = getTurretPosition();
          double arbFF;
          double pidOutput = posPIDController.calculate(-lltx, 0);
          arbFF = turretFFDeg.calculate(pidOutput);
          SmartDashboard.putNumber("Turret angle", lltx);
          SmartDashboard.putNumber("Turret position", pos);
          SmartDashboard.putNumber("Turret arbFF", arbFF);
          SmartDashboard.putNumber("Turret pidOutput", pidOutput);
          
          turretPID.setReference(pos + lltx, ControlType.kSmartMotion, 2, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);

          
        }else{
          turret.set(0);

        }*/
      }


      public static boolean LLHasValidTarget(){
        return llHasValidTarget;
      }
      
      public static double getLLHoodPosition(){

        if (llHasValidTarget)
      {
      return calculateHoodAngle(llty);
    }else{
        return 0; //do this better
    }
    }


    public static double calculateHoodAngle(double ty){
      double hoodAngle = 2e-05*Math.pow(ty, 4)-0.0002*Math.pow(ty, 3) -0.0144*Math.pow(ty, 2) - 0.4064*ty + 14.482;

       if(hoodAngle > Constants.HOOD_MAX_LIMIT){
         hoodAngle = Constants.HOOD_MAX_LIMIT;
       }
       if(hoodAngle < Constants.HOOD_MIN_LIMIT){
         hoodAngle = Constants.HOOD_MIN_LIMIT;
       }
       return hoodAngle;
    }

    public static double getLLShooterSpeed(){

        if (llHasValidTarget)
      {
        return calculateShooterSpeed(llty);
    }else{
        return 0; //do this better
    }
    }

    public static double calculateShooterSpeed(double ty){
      double shooterSpeed = 0.0025*Math.pow(ty,4)+0.0073*Math.pow(ty,3)+0.1006*Math.pow(ty,2) - 16.377*ty + 2054.2;
      if(shooterSpeed > Constants.SHOOTER_MAX_SPEED){
       shooterSpeed = Constants.SHOOTER_MAX_SPEED;
      }
      if(shooterSpeed < 0){
       shooterSpeed = 0;
      }
      return shooterSpeed;
    }
    public static void updateLimelightTracking()
    {
          lltv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
          lltx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
          llty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
          llts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

          if (lltv < 1.0)
          {
            llHasValidTarget = false;
            
            return;
          }
          lldist = Units.inchesToMeters(Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) / Math.tan(Units.degreesToRadians(Constants.LL_MOUNT_ANGLE + llty));
          
           if(llts< -45){
             llts+= 90;
           }
           
          llHasValidTarget = true;
          
    }
/*
     //add boolean argument for whether scoring or missing?
  public void setHoodPosPID(double distance, double paraV) { 
    //paraFeedforward = paraV / distance; //this is just a guess
    double desiredAngle = Data.getHoodAngle(distance);
    double pidOutput = pidController.calculate(getAngleRad(), desiredAngle);

    //double volts = feedforward.calculate(desiredAngle, pidOutput + paraFeedforward);
    double volts = feedforward.calculate(desiredAngle, pidOutput);
    SmartDashboard.putNumber("angle rad", getAngleRad());
    SmartDashboard.putNumber("desired ang rad", desiredAngle);
    SmartDashboard.putNumber("hood pid", pidOutput);
    SmartDashboard.putNumber("hood volts", volts);
    motorHood.setVoltage(volts);

    //angleSetpoint = desiredAngle;
  }

  public void setHoodVelPID(double distance, double paraV) {
    paraFeedforward = paraV / distance; //this is just a guess //radians
    double desiredAngle = Data.getHoodAngle(distance);
    double setpointV = pidController.calculate(getAngleRad(), desiredAngle) + paraFeedforward;
    //this would need two different PID controllers? Waste of time
    motorHood.setVoltage(pidController.calculate(getEncVelocityRad(), setpointV) 
      + feedforward.calculate(getAngleRad(), setpointV));

    //oangleSetpoint = desiredAngle;
  }
*/


    public static double getBoosterSpeed() {
        return boosterEncoder.getVelocity() * 1 / Constants.GEAR_RATIO_BOOSTER;
    }
    
    public static double getShooterSpeed() {
    return shooterEncoder.getVelocity() * 1 / Constants.GEAR_RATIO_SHOOTER;
    }

    public static double getHoodAngle() {
        return 360*hoodEncoder.getPosition() / Constants.GEAR_RATIO_HOOD;
    }

    public static double getTurretSpeed(){
        return turretEncoder.getVelocity() * 1 / Constants.GEAR_RATIO_TURRET;
    } 

    public static double getTurretPosition(){
        return 360*turretEncoder.getPosition() / Constants.GEAR_RATIO_TURRET;
    } 

    
    public static double getTurretEncoder(){
      return turretEncoder.getPosition();
  } 

  
  public static double getTurretABSEncoder(){
    return turretAbsoluteEncoder.getAbsolutePosition();
} 

  //Return number of meters from LL to hub.
  public static double getRobotDistance(){
    double anglesInDegrees = Constants.LL_MOUNT_ANGLE + llty;
    return Units.inchesToMeters(Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) / Math.tan(Math.toRadians(anglesInDegrees));
}

  //Return number of meters from LL to hub.
  public static double calcTyFromDist(double dist){
    return Math.toDegrees(Math.atan(Units.inchesToMeters(Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) / dist)) - Constants.LL_MOUNT_ANGLE;
}

public static void setMotorPosPID(double tx, double perpV, double distance, double angV){
  //tangentialFeedforward = Units.radiansToDegrees(perpV / distance);
  //rotationalFeedforward = -angV;
  //double angle = getAngle();
  //double setpoint = angle - tx;
  double pidOutput = posPIDController.calculate(-1*tx, 0);
  //double pidOutput = 0;
  double turretVolts = turretFFDeg.calculate(    pidOutput);
  SmartDashboard.putNumber("turret volts", turretVolts);
  SmartDashboard.putNumber("pid turret", pidOutput);
  //SmartDashboard.putNumber("tangent ff", tangentialFeedforward);
  //SmartDashboard.putNumber("rot ff", rotationalFeedforward);
  setVoltageBounded(turretVolts);
}

private static void setVoltageBounded(double volts, double turretAngle) {
  SmartDashboard.putNumber("SWD Bound turretAngle", turretAngle);
  if (turretAngle <= -1*Constants.TURRET_ANGLE_THRESHOLD|| resettingForward) {
    resettingForward = true;
    turret.setVoltage(Constants.TURRET_RESET_VOLTAGE);
    if (turretAngle >= -1*Constants.TURRET_ANGLE_THRESHOLD + 350) {
      resettingForward = false;
    }
    SmartDashboard.putBoolean("SWD resettingFwd", resettingForward);

    return;
  }
  else if (turretAngle >= Constants.TURRET_ANGLE_THRESHOLD || resettingBackward) {
    resettingBackward = true;
    turret.setVoltage(-Constants.TURRET_RESET_VOLTAGE);
    if (turretAngle <= Constants.TURRET_ANGLE_THRESHOLD - 350) {
      resettingBackward = false;
    }
    SmartDashboard.putBoolean("SWD resettingBack", resettingBackward);

    return;
  }
  else {
    if(Math.abs(volts) > 5){
      volts = Math.signum(volts)*5;
    }else if(Math.abs(volts) < 0.5){
      volts = 0;
    }
    turret.setVoltage(volts);
  }
}

private static void setVoltageBounded(double volts) {
  double turretAngle = getTurretPosition();
  setVoltageBounded(volts, turretAngle);
}



  public static void calculateDisplacement(){
      Pose2d position = DriveSubsystem.getPose();
      double theta = Math.atan(position.getX()/position.getY());
      double alpha = theta + Math.toRadians(90);
      double dLL = getRobotDistance();
      double dPoseX = position.getX();
      double dPoseY = position.getY();
      double dPose = Math.sqrt(dPoseX * dPoseX + dPoseY * dPoseY);  
      distanceToHub = Math.sqrt(dLL * dLL + dPose * dPose -2 * dLL * dPose * Math.cos(alpha));
      double gamma = Math.acos((distanceToHub * distanceToHub - dLL * dLL - dPose * dPose) / (-2 * dLL * dPose));
      double beta = Units.radiansToDegrees(position.getRotation().getRadians() + gamma);
      if(beta > 180){
        beta -= 360;
      }
      turretAngleToHub = beta;
    }






    public ShooterSubsystem() {
        setupMotors();
        calculateDisplacement();
    }
    
    @Override
    public void periodic() {
      updateLimelightTracking();
    }

}
