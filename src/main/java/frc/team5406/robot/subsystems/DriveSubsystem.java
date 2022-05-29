package frc.team5406.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class DriveSubsystem extends SubsystemBase {
  private static CANSparkMax leftDrive = new CANSparkMax(Constants.MOTOR_DRIVE_LEFT_ONE, MotorType.kBrushless);
  private static CANSparkMax leftDriveFollower = new CANSparkMax(Constants.MOTOR_DRIVE_LEFT_TWO, MotorType.kBrushless);
  private static CANSparkMax rightDrive = new CANSparkMax(Constants.MOTOR_DRIVE_RIGHT_ONE, MotorType.kBrushless);
  private static CANSparkMax rightDriveFollower = new CANSparkMax(Constants.MOTOR_DRIVE_RIGHT_TWO,
      MotorType.kBrushless);
  private static DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);

  static SimpleMotorFeedforward driveTrain = new SimpleMotorFeedforward(Constants.S_VOLTS, Constants.V_VOLTS,
  Constants.A_VOLTS);
  static Pose2d pose = new Pose2d();
  static DifferentialDriveOdometry odometry;
  static AHRS gyro = new AHRS(SPI.Port.kMXP);
  public static boolean baselockStarted = false;
  static int count = 0;
  static int count2 = 0;
  private static RelativeEncoder leftEncoder, rightEncoder;
  private static SparkMaxPIDController leftMotorPID, rightMotorPID;
  private final Field2d m_field = new Field2d();

  private static ChassisSpeeds curChassisSpeeds = new ChassisSpeeds();

    private static final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      getHeading(), new Pose2d(), VecBuilder.fill(0.05, 0.05,0.15, 0.15, Units.degreesToRadians(5)), 
    VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(10)), VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(3)));



    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.TRACK_WIDTH_INCHES));


  public static void setupMotors() {
    leftDriveFollower.restoreFactoryDefaults();
    rightDriveFollower.restoreFactoryDefaults();
    rightDrive.restoreFactoryDefaults();
    leftDrive.restoreFactoryDefaults();


    leftDriveFollower.follow(leftDrive, false);
    rightDriveFollower.follow(rightDrive, false);
    rightDrive.setInverted(false);
    leftDrive.setInverted(true);
    drive.setSafetyEnabled(false);
    
    leftDrive.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_LEFT);
    leftDriveFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_LEFT);
    rightDrive.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_RIGHT);
    rightDriveFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_RIGHT);
    leftMotorPID = leftDrive.getPIDController();
    rightMotorPID = rightDrive.getPIDController();
    leftEncoder = leftDrive.getEncoder();
    rightEncoder = rightDrive.getEncoder();

    leftMotorPID.setP(Constants.LEFT_DRIVE_PID0_P, 0);
    leftMotorPID.setI(Constants.LEFT_DRIVE_PID0_I, 0);
    leftMotorPID.setD(Constants.LEFT_DRIVE_PID0_D, 0);
    leftMotorPID.setIZone(0, 0);
    leftMotorPID.setFF(Constants.LEFT_DRIVE_PID0_F, 0);
    leftMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    leftMotorPID.setP(Constants.LEFT_DRIVE_PID1_P, 1);
    leftMotorPID.setI(Constants.LEFT_DRIVE_PID1_I, 1);
    leftMotorPID.setD(Constants.LEFT_DRIVE_PID1_D, 1);
    leftMotorPID.setIZone(0, 1);
    leftMotorPID.setFF(Constants.LEFT_DRIVE_PID1_F, 1);
    leftMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);



    rightMotorPID.setP(Constants.RIGHT_DRIVE_PID0_P, 0);
    rightMotorPID.setI(Constants.RIGHT_DRIVE_PID0_I, 0);
    rightMotorPID.setD(Constants.RIGHT_DRIVE_PID0_D, 0);
    rightMotorPID.setIZone(0, 0);
    rightMotorPID.setFF(Constants.RIGHT_DRIVE_PID0_F, 0);
    rightMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    rightMotorPID.setP(Constants.RIGHT_DRIVE_PID1_P, 1);
    rightMotorPID.setI(Constants.RIGHT_DRIVE_PID1_I, 1);
    rightMotorPID.setD(Constants.RIGHT_DRIVE_PID1_D, 1);
    rightMotorPID.setIZone(0, 1);
    rightMotorPID.setFF(Constants.RIGHT_DRIVE_PID1_F, 1);
    rightMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);

    rightDrive.setOpenLoopRampRate(0.05);
    leftDrive.setOpenLoopRampRate(0.05);
    

    leftDrive.burnFlash();
    leftDriveFollower.burnFlash();
    rightDrive.burnFlash();
    rightDriveFollower.burnFlash();

    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.001));

  }

  public void curvatureDrive(double speed, double turn, boolean turnInPlace) {
    drive.curvatureDrive(-1*speed, Math.signum(turn)*turn*turn, turnInPlace);
  }

  public static void setBrakeMode(boolean brake) {
    IdleMode brakeMode = (brake ? IdleMode.kBrake : IdleMode.kCoast);
    leftDrive.setIdleMode(brakeMode);
    leftDriveFollower.setIdleMode(brakeMode);
    rightDrive.setIdleMode(brakeMode);
    rightDriveFollower.setIdleMode(brakeMode);
  }

  public static void setNormalMode() {
    leftDrive.setIdleMode(IdleMode.kBrake);
    leftDriveFollower.setIdleMode(IdleMode.kCoast);
    rightDrive.setIdleMode(IdleMode.kBrake);
    rightDriveFollower.setIdleMode(IdleMode.kCoast);
  }

  public static void stopMotors() {
    leftDrive.stopMotor();
    rightDrive.stopMotor();
    leftDriveFollower.stopMotor();
    rightDriveFollower.stopMotor();
  }
 
  public static void baselock() {
    if (!baselockStarted) {
      count++;
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
     //System.out.println(count + " Baselock ON, " + leftEncoder.getPosition() + " " + rightEncoder.getPosition());
      leftMotorPID.setReference(0, ControlType.kPosition, 1);
      rightMotorPID.setReference(0, ControlType.kPosition, 1);
      baselockStarted = true;
    }
  }

  public static void unsetBaselock(String source) {
    if(baselockStarted){
      count2++;
    //System.out.println(count2 + ", " + source + " Baselock OFF, " + leftEncoder.getPosition() + " " + rightEncoder.getPosition());
    stopMotors();
        baselockStarted = false;

    }
  }
//Both unsetBaselock and Baselock are untested

  public static double getLeftSpeed() {
    return Units.inchesToMeters(((leftEncoder.getVelocity() / Constants.GEAR_RATIO_DRIVE) * Math.PI * Constants.DRIVE_WHEEL_DIAMETER)
        / Constants.SECONDS_PER_MINUTE);
  }

  public static double getRightSpeed() {
    return Units.inchesToMeters(((rightEncoder.getVelocity() / Constants.GEAR_RATIO_DRIVE) * Math.PI * Constants.DRIVE_WHEEL_DIAMETER)
        / Constants.SECONDS_PER_MINUTE);
  }

  public static double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  /**
   * @return Left Distance in Meters
   */
  public static double getLeftDistance() {
    return Units.inchesToMeters(
        (leftEncoder.getPosition() / Constants.GEAR_RATIO_DRIVE) * Math.PI * Constants.DRIVE_WHEEL_DIAMETER);
  }

  public static double getRightDistance() {
    return Units.inchesToMeters(
        (rightEncoder.getPosition() / Constants.GEAR_RATIO_DRIVE) * Math.PI * Constants.DRIVE_WHEEL_DIAMETER);
  }

  // Reset Encoders
  public static void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public static Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle() * (Constants.GYRO_REVERSED ? -1.0 : 1.0));
  }
  
  public static void setHeading() {
    gyro.zeroYaw();
  }

  public DriveSubsystem() {
    odometry = new DifferentialDriveOdometry(getHeading());
    setupMotors();
    SmartDashboard.putData("Field", m_field);
  }


  public void outputSpeeds(double leftSpeed, double rightSpeed) {
    double origLeftSpeed = leftSpeed;
    double origRightSpeed = rightSpeed;
    leftSpeed /= Units.inchesToMeters(Constants.INCHES_PER_REV / Constants.SECONDS_PER_MINUTE);
    rightSpeed /= Units.inchesToMeters(Constants.INCHES_PER_REV / Constants.SECONDS_PER_MINUTE);
    // System.out.println("Left Speed, " + leftSpeed);
    // System.out.println("Right Speed, "+ rightSpeed);
    SmartDashboard.putNumber("Orig Left Speed", origLeftSpeed);
    SmartDashboard.putNumber("Orig Right Speed", origRightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);

    SmartDashboard.putNumber("X Translation", pose.getTranslation().getX());
    SmartDashboard.putNumber("Left Speed (A)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed (A)", rightEncoder.getVelocity());

    double arbFFLeft = driveTrain.calculate(origLeftSpeed);
    double arbFFRight = driveTrain.calculate(origRightSpeed);
    SmartDashboard.putNumber("Arb FF L", arbFFLeft);
    SmartDashboard.putNumber("Arb FF R", arbFFRight);

    leftMotorPID.setReference(leftSpeed, ControlType.kVelocity, 0, arbFFLeft, SparkMaxPIDController.ArbFFUnits.kVoltage);
    rightMotorPID.setReference(rightSpeed, ControlType.kVelocity, 0, arbFFRight, SparkMaxPIDController.ArbFFUnits.kVoltage);
    // System.out.println("Pose: " + getPose());
    drive.feed();
  }
  

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    drive.feed();
  }

  public static Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void reset() {
    setHeading();
    resetEncoders();
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  public static void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public static void setPose(Pose2d pose) {
    resetEncoders();
    poseEstimator.resetPosition(pose, getHeading());
  }

  public static void updateOdometry() {
    poseEstimator.update(getHeading(), getCurrentWheelSpeeds(), getLeftDistance(), getRightDistance());
  }

   public static void updateOdometryWithVision(double distance, double gyroAngle, double turretAngle, double tx) {
    updateOdometry();
      turretAngle -= 90;
     
      double angle = Units.radiansToDegrees(Units.degreesToRadians(turretAngle + 90 +tx) - Math.atan2(distance * Math.cos(Units.degreesToRadians(gyroAngle - turretAngle - tx)), distance * Math.sin(Units.degreesToRadians(gyroAngle - turretAngle - tx))));


       Pose2d poseEstimate = new Pose2d(
        distance * Math.cos(Units.degreesToRadians(gyroAngle - turretAngle - tx)), 
        distance * Math.sin(Units.degreesToRadians(gyroAngle - turretAngle - tx)),
        Rotation2d.fromDegrees(angle));
    
      double poseDelta = poseEstimate.getTranslation().getDistance(getPoseEstimate().getTranslation());
      SmartDashboard.putNumber("SWD poseDelta", poseDelta);
      //failsafe in case the calculation is messed up due to bad sensors or whatever
      if (poseDelta > Constants.LL_POSE_TOLERANCE) {
        DriverStation.reportWarning("Pose Estimation error larger than expected.", false);
        poseEstimate = getPoseEstimate();
      }

    poseEstimator.addVisionMeasurement(poseEstimate, Timer.getFPGATimestamp() - Constants.LL_LATENCY);
  }

  public static DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftSpeed(),
      getRightSpeed());
  }

  public static Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  public static double getParaV(double turretAngleDeg) {
    turretAngleDeg -= 90;
    //converts from robot-relative velocities directly to target relative velocities
    //double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    //double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);
    //double output = vx * Math.cos(theta) + vz * Math.sin(theta);

    double output = -curChassisSpeeds.vyMetersPerSecond * Math.cos(theta) + curChassisSpeeds.vxMetersPerSecond * Math.sin(theta);

    SmartDashboard.putNumber("paraV", output);
    return output;
  }

  public static double getPerpV(double turretAngleDeg) {
    turretAngleDeg -= 90;

    //converts from robot-relative velocities directly to target relative velocities
    //double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    //double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);

    //double output = -vx * Math.sin(theta) + vz * Math.cos(theta);
    double output = curChassisSpeeds.vyMetersPerSecond * Math.sin(theta) + curChassisSpeeds.vxMetersPerSecond * Math.cos(theta);

    SmartDashboard.putNumber("perpV", output);
    return output;
  }

  public static double getAngV() {
    double output = Math.toDegrees(curChassisSpeeds.omegaRadiansPerSecond);
    //SmartDashboard.putNumber("angv", output);
    return output;
  }

  public static double getVx() {
    //return -curChassisSpeeds.vyMetersPerSecond;
    return curChassisSpeeds.vxMetersPerSecond;
  }

  public static double getVy() {
    //return curChassisSpeeds.vxMetersPerSecond;
    return curChassisSpeeds.vyMetersPerSecond;
  }

  public static double getGyroAccelX(){
    return gyro.getWorldLinearAccelX();
  }

  public static double getGyroAccelY(){
    return gyro.getWorldLinearAccelY();
  }

  public static double getGyroAngV() {
    return gyro.getRate();
  }
  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLeftDistance(), getRightDistance());
    updateOdometry();
    SmartDashboard.putNumber("Pose X", getPoseEstimate().getX());
    SmartDashboard.putNumber("Pose Y", getPoseEstimate().getY());
    SmartDashboard.putNumber("Pose Rotation", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("NavX Angle",getHeading().getDegrees());
    SmartDashboard.putNumber("X Translation", pose.getTranslation().getX());
    SmartDashboard.putNumber("Y Translation", pose.getTranslation().getY());
    SmartDashboard.putNumber("Left Speed (A)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed (A)", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());

    m_field.setRobotPose(getPoseEstimate());
    curChassisSpeeds = kinematics.toChassisSpeeds(getCurrentWheelSpeeds());



  }

}
