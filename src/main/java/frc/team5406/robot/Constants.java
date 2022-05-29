package frc.team5406.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int MOTOR_DRIVE_LEFT_ONE = 1; //SparkMax, NEO  
    public static final int MOTOR_DRIVE_LEFT_TWO = 2; //SparkMax, NEO  
    public static final int MOTOR_DRIVE_RIGHT_ONE = 3; //SparkMax, NEO
    public static final int MOTOR_DRIVE_RIGHT_TWO = 4; //SparkMax, NEO
    public static final int MOTOR_INTAKE_ROLLERS_ONE = 5; //SparkMax, NEO
   // public static final int MOTOR_INTAKE_ROLLERS_TWO = 6; SparkMax, NEO550 
    public static final int MOTOR_CONVEYOR_BOTTOM_ONE = 9; //SparkMax, NEO550 
    public static final int MOTOR_CONVEYOR_TOP_ONE = 10; //SparkMax, NEO550
    public static final int MOTOR_TURRET_AZIMUTH_ONE = 11; //SparkMax, NEO550
    public static final int MOTOR_BOOSTER_ONE = 12; //SparkMax, NEO
    public static final int MOTOR_HOOD_ONE = 13; //SparkMax, NEO550
    public static final int MOTOR_SHOOTER_WHEEL_ONE = 14; // SparkMax, NEO
    public static final int MOTOR_SHOOTER_WHEEL_TWO = 15; // SparkMax, NEO
    public static final int MOTOR_CLIMBER_ONE = 16; //SparkMax, NEO
    public static final int MOTOR_CLIMBER_TWO = 17; //SparkMax, NEO
    
    public static final int TURRET_ENCODER = 31; //Turret CANCoder

    public static final double GEAR_RATIO_DRIVE = 11.0/1.0;
    public static final double GEAR_RATIO_SERIALIZER_LEFT = 5.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_SERIALIZER_RIGHT = 5.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_INTAKE_ROLLERS= 2.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_CONVEYOR_BOTTOM= 4.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_CONVEYOR_TOP= 4.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_CLIMBER = 320.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_SHOOTER = 4.0 / 3.0;
    public static final double GEAR_RATIO_BOOSTER = 2.0/1.0;
    public static final double GEAR_RATIO_TURRET = 96.5;// 12*138/15;
    public static final double GEAR_RATIO_HOOD =  20*450/24;

    public static final double GEAR_RATIO_TURRET_ENCODER = 316.0/360;
 
  
       
    public static final double DRIVE_WHEEL_DIAMETER = 6.0; //Inches
    public static final int SECONDS_PER_MINUTE = 60;
    public static final int INTAKE_ROLLER_SPEED = 2500; 
    public static final int SERIALIZER_SPEED = 2000;
    public static final int CONVEYOR_SPEED_BOTTOM = 1000;
    public static final int CONVEYOR_SPEED_TOP = 1000;
    public static final int INTAKE_ROLLER_MAX_SPEED = 3500;
    public static final int CONVEYOR_MAX_SPEED = 2500;
    public static final double SERIALIZER_MULTIPLIER_LEFT = 0.5;
    public static final int SERIALIZER_MAX_SPEED = 2000;
    public static final int CLIMBER_MAX_SPEED = 18; //RPM
    public static final int TURRET_MAX_SPEED = 60; //RPM
    public static final int BOOSTER_SPEED = 1000;
    public static final int SHOOTER_MAX_SPEED = 4000; //RPM


    //PCM Ports
    public static final int CYLINDER_CLIMB_HOOK_ONE = 9;
    public static final int CYLINDER_CLIMB_HOOK_TWO = 10; 
    public static final int CYLINDER_INTAKE_PIVOT_ONE = 8;

    //Other Constants
    public static final boolean CLIMB_HOOK_ONE_EXTEND = false;
    public static final boolean CLIMB_HOOK_ONE_RETRACT = true;
    public static final boolean CLIMB_HOOK_TWO_EXTEND = false;
    public static final boolean CLIMB_HOOK_TWO_RETRACT = true;

    //Color Constants
    public static final double COLOR_BLUE_R = 0.156168;
    public static final double COLOR_BLUE_G = 0.388734;
    public static final double COLOR_BLUE_B = 0.455477;
    public static final double COLOR_RED_R = 0.518329;
    public static final double COLOR_RED_G = 0.350874;
    public static final double COLOR_RED_B = 0.131163;
    public static final double COLOR_BLACK_R = 0.277588;
    public static final double COLOR_BLACK_G = 0.480225;
    public static final double COLOR_BLACK_B = 0.242432;
    public static final double COLOR_WHITE_R = 0.284251;
    public static final double COLOR_WHITE_G = 0.478355;
    public static final double COLOR_WHITE_B = 0.2378;

    public static final int CS_PROXIMITY_THRESHOLD = 20;

    //Current Limits
    public static final int CURRENT_LIMIT_SHOOTER_WHEEL = 60;
    public static final int CURRENT_LIMIT_BOOSTER = 20;
    public static final int CURRENT_LIMIT_TURRET_AZIMUTH = 20;
    public static final int CURRENT_LIMIT_HOOD = 20;
    public static final int CURRENT_LIMIT_SERIALIZER_LEFT = 20;
    public static final int CURRENT_LIMIT_SERIALIZER_RIGHT = 20;
    public static final int CURRENT_LIMIT_DRIVE_LEFT = 40;
    public static final int CURRENT_LIMIT_DRIVE_RIGHT = 40;
    public static final int CURRENT_LIMIT_CLIMBER = 80;
    public static final int CURRENT_LIMIT_INTAKE_ROLLERS = 60;
    public static final int CURRENT_LIMIT_CONVEYOR_BOTTOM = 20;
    public static final int CURRENT_LIMIT_CONVEYOR_TOP = 20;
    public static final boolean INTAKE_EXTEND = true;
    public static final boolean INTAKE_RETRACT = false;


    public static final double LEFT_DRIVE_PID0_P = 9e-5;
    public static final double LEFT_DRIVE_PID0_I = 0;
    public static final double LEFT_DRIVE_PID0_D = 0;
    public static final double LEFT_DRIVE_PID0_F = 0;

    public static final double RIGHT_DRIVE_PID0_P = 9e-5;
    public static final double RIGHT_DRIVE_PID0_I = 0;
    public static final double RIGHT_DRIVE_PID0_D = 0;
    public static final double RIGHT_DRIVE_PID0_F = 0;

    public static final double LEFT_DRIVE_PID1_P = 3e-1;
    public static final double LEFT_DRIVE_PID1_I = 0;
    public static final double LEFT_DRIVE_PID1_D = 0;
    public static final double LEFT_DRIVE_PID1_F = 1.5e-1;

    public static final double RIGHT_DRIVE_PID1_P = 3e-1;
    public static final double RIGHT_DRIVE_PID1_I = 0;
    public static final double RIGHT_DRIVE_PID1_D = 0;
    public static final double RIGHT_DRIVE_PID1_F = 1.5e-1;

    public static final double SERIALIZER_LEFT_PID1_P = 6.0919E-13;
    public static final double SERIALIZER_LEFT_PID1_I = 0;
    public static final double SERIALIZER_LEFT_PID1_D = 0;
    public static final double SERIALIZER_LEFT_PID1_F = 1.5e-5;
    public static final double SERIALIZER_LEFT_KS = 0.20404;
    public static final double SERIALIZER_LEFT_KV = 0.2649;
    public static final double SERIALIZER_LEFT_KA = 0.006719;

    public static final double SERIALIZER_RIGHT_PID1_P = 2.5254E-11;
    public static final double SERIALIZER_RIGHT_PID1_I = 0;
    public static final double SERIALIZER_RIGHT_PID1_D = 0;
    public static final double SERIALIZER_RIGHT_PID1_F = 1.5e-5;
    public static final double SERIALIZER_RIGHT_KS = 0.092841;
    public static final double SERIALIZER_RIGHT_KV = 0.25003;
    public static final double SERIALIZER_RIGHT_KA = 0.0085248;

    public static final double INTAKE_ROLLERS_PID1_P = 2.5475E-10;
    public static final double INTAKE_ROLLERS_PID1_I = 0;
    public static final double INTAKE_ROLLERS_PID1_D = 0;
    public static final double INTAKE_ROLLERS_PID1_F = 1.5e-5;
    public static final double INTAKE_ROLLERS_KS = 0.36666;
    public static final double INTAKE_ROLLERS_KV = 0.1621;
    public static final double INTAKE_ROLLERS_KA = 0.0074968;


    public static final double CONVEYOR_BOTTOM_PID1_P = 8.7729E-5;
    public static final double CONVEYOR_BOTTOM_PID1_I = 0;
    public static final double CONVEYOR_BOTTOM_PID1_D = 0;
    public static final double CONVEYOR_BOTTOM_PID1_F = 1.5e-5;
    public static final double CONVEYOR_BOTTOM_KS = 0.22569;
    public static final double CONVEYOR_BOTTOM_KV = 0.25313;
    public static final double CONVEYOR_BOTTOM_KA = 0.0068695;

    public static final double CONVEYOR_TOP_PID1_P = 2.7764E-14;
    public static final double CONVEYOR_TOP_PID1_I = 0;
    public static final double CONVEYOR_TOP_PID1_D = 0;
    public static final double CONVEYOR_TOP_PID1_F = 1.5e-5;
    public static final double CONVEYOR_TOP_KS = 0.16838;
    public static final double CONVEYOR_TOP_KV = 0.24884;
    public static final double CONVEYOR_TOP_KA = 0.0043763;

    public static final double CLIMBER_PID1_P = 9e-5;
    public static final double CLIMBER_PID1_I = 0;
    public static final double CLIMBER_PID1_D = 0;
    public static final double CLIMBER_PID1_F = 1.7e-4;

    public static final double CLIMBER_PID2_P = 1.5e-1;
    public static final double CLIMBER_PID2_I = 0;
    public static final double CLIMBER_PID2_D = 0;
    public static final double CLIMBER_PID2_F = 1.7e-3;

    public static final double CLIMBER_KS = 0.62365;
    public static final double CLIMBER_KV = 42.554;
    public static final double CLIMBER_KA = 4.1536;

    public static final double CLIMBER_PID3_P = 2.6109E-06; //velocity
    public static final double CLIMBER_PID3_I = 0;
    public static final double CLIMBER_PID3_D = 0;
    public static final double CLIMBER_PID3_F = 1.5e-5;

    public static final double CLIMBER_PID4_P = 0.0036653; //position
    public static final double CLIMBER_PID4_I = 0;
    public static final double CLIMBER_PID4_D = 4.9336;
    public static final double CLIMBER_PID4_F = 1.5e-5;

    public static final double SHOOTER_PID0_P = 1.2903E-07;
    public static final double SHOOTER_PID0_I = 0;
    public static final double SHOOTER_PID0_D = 0;
    public static final double SHOOTER_PID0_F = 1.5e-5;
    public static final double SHOOTER_KS = 0.067839;
    public static final double SHOOTER_KV = 0.16819;
    public static final double SHOOTER_KA = 0.076352;

    public static final double BOOSTER_PID0_P = 2.2062E-09;
    public static final double BOOSTER_PID0_I = 0;
    public static final double BOOSTER_PID0_D = 0;
    public static final double BOOSTER_PID0_F = 1.5e-5;
    public static final double BOOSTER_KS = 0.15925;
    public static final double BOOSTER_KV = 0.12087;
    public static final double BOOSTER_KA = 0.0038805;

    public static final double HOOD_PID0_P = 1.3066E-11; //velocity
    public static final double HOOD_PID0_I = 0;
    public static final double HOOD_PID0_D = 0;
    public static final double HOOD_PID0_F = 1.5e-5;
    public static final double HOOD_PID1_P = 1E-00; //position
    public static final double HOOD_PID1_I = 0;
    public static final double HOOD_PID1_D = 0.1;
    public static final double HOOD_PID1_F = 3e-3;
    public static final double HOOD_KS = 0.18418;
    public static final double HOOD_KV = 0.056822;
    public static final double HOOD_KA = 0.0056543;


    public static final double TURRET_PID0_P = 4.9778E-08; //velocity, rotations/s
    public static final double TURRET_PID0_I = 0;
    public static final double TURRET_PID0_D = 0;
    public static final double TURRET_PID0_F = 1.5e-5;

    public static final double TURRET_PID1_P = 0.00003; //position, rotations
    public static final double TURRET_PID1_I = 0;
    public static final double TURRET_PID1_D = 0.00;
    public static final double TURRET_PID1_F = 6e-5;

    public static final double TURRET_PID2_P = 0.000001; //limelight, rotations
    public static final double TURRET_PID2_I = 0;
    public static final double TURRET_PID2_D = 0.00000001;
    public static final double TURRET_PID2_F = 1.5e-5;
    public static final double TURRET_PID3_P = 0.08; //position, rotations
    public static final double TURRET_PID3_I = 0;
    public static final double TURRET_PID3_D = 0.005;
    public static final double TURRET_PID3_F = 5.5e-4;
    public static final double TURRET_KS = 0.33173;
    public static final double TURRET_KV = 7.1211;
    public static final double TURRET_KA = 0.38588;
    public static final double TURRET_MAX_V = 180; //deg/s
    public static final double TURRET_MAX_A = 180; //deg/s/s

    


    //enum for Ball and Alliance colors
    public enum BallColor {RED, BLUE, INVALID, NO_BALL}

    //XBox controller ports
    public static final int OPERATOR_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
    public static final double TRIGGER_THRESHOLD = 0.1;
    public static final double JOYSTICK_THRESHOLD = 0.1;

    //Color Sensor MUX Ports
    public static final int COLOR_SENSOR_ONE_PORT = 4;
    public static final int COLOR_SENSOR_TWO_PORT = 5;

    public static final int DISTANCE_SENSOR_ONE_PORT = 0;
    public static final int DISTANCE_SENSOR_TWO_PORT = 2;

    public static final double S_VOLTS = 0.209;
    public static final double V_VOLTS = 2.5;
    public static final double A_VOLTS = 0.55;
    
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double TRACK_WIDTH_INCHES = 25.6; //experimentally determined
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(Units.inchesToMeters(TRACK_WIDTH_INCHES));

    public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
    public static final double MAX_ACTUAL_SPEED_METERS_PER_SECOND = 4;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

    public static final double DELTA_V_THRESHOLD = 1.9; // Metres per second per loop cycle

    public static final boolean GYRO_REVERSED = true;
    public static double INCHES_PER_REV = 1.7164; //Experimentally obtained Math.PI * DRIVE_WHEEL_DIAMETER / GEAR_RATIO_DRIVE;
    public static final double OUTPUT_RANGE_MIN = -1;
    public static final double OUTPUT_RANGE_MAX = 1;

    public static final boolean CLIMBER_EXTEND = true;
    public static final boolean CLIMBER_RETRACT = false;

    public static final int NEO550_CURRENT_SPIKE = 20;

    public static final double HOOD_ZEROING_SPEED = -0.165;
  // Low lob
    public static final int HOOD_ANGLE_LOW_LOB = 20;
    public static final int FLYWHEEL_SPEED_LOW_LOB = 1000;

    public static final int FLYWHEEL_SPEED_FENDER_HIGH = 2250;
    public static final double HOOD_ANGLE_FENDER_HIGH = 1.5;

    public static final int HOOD_MAX_LIMIT = 22;
    public static final int HOOD_MIN_LIMIT = 0;

    public static final int TURRET_CW_LIMIT = 53;
    public static final int TURRET_CCW_LIMIT = -53; 
    public static final double TURRET_RESET_VOLTAGE = 5;
    public static final int TURRET_ANGLE_THRESHOLD = 190;
    
    public static final double LL_TARGET_HEIGHT  = 101.625 +1.2;
    public static final double LL_MOUNT_ANGLE = 40.0;
    public static final double LL_MOUNT_HEIGHT = 42.9;
    public static final double LL_POSE_TOLERANCE = 3.5; //meters
    public static final double LL_LATENCY = 0.02; //seconds

    public static final Translation2d HUB_LOCATION = new Translation2d(0, 0);
    public static final double LOOP_TIME = 0.02; //seconds
    public static final double HUB_RADIUS = 0.6858; //m = 2.25ft



    public static final double MATCH_RPM_LOWER_THRESHOLD = 0.95;
    public static final double MATCH_RPM_UPPER_THRESHOLD = 1.05;

    public static final int MIN_PRESSURE = 90;
    public static final int MAX_PRESSURE = 110;

    public static final double BL_DRIVETRAIN_SPEED = Units.inchesToMeters(6); //meters/s

    public static final double HOOD_ANGLE_OFFSET = 3.0;

    public static final double KEEP_SPINNING_TIME_CONVEY = 1.0;

    public static final double KEEP_SPINNING_TIME_INTAKE = 1.0;
}

