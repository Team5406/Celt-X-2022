package frc.team5406.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;


public class ConveyorSubsystem extends SubsystemBase {
  
    private static CANSparkMax conveyorBottom = new CANSparkMax(Constants.MOTOR_CONVEYOR_BOTTOM_ONE,
            MotorType.kBrushless);
    private static CANSparkMax conveyorTop = new CANSparkMax(Constants.MOTOR_CONVEYOR_TOP_ONE, MotorType.kBrushless);

    private static RelativeEncoder conveyorBottomEncoder, conveyorTopEncoder;
    private static SparkMaxPIDController conveyorBottomPID, conveyorTopPID;


    static SimpleMotorFeedforward conveyorBottomFF = new SimpleMotorFeedforward(Constants.CONVEYOR_BOTTOM_KS, Constants.CONVEYOR_BOTTOM_KV, Constants.CONVEYOR_BOTTOM_KA);
    static SimpleMotorFeedforward conveyorTopFF = new SimpleMotorFeedforward(Constants.CONVEYOR_TOP_KS, Constants.CONVEYOR_TOP_KV, Constants.CONVEYOR_TOP_KA);
    
  
  public static double lastTime;
  public static boolean keepSpinning;

    public static void setupMotors() {
        conveyorBottom.setInverted(false);
        conveyorTop.setInverted(true);
        conveyorBottomEncoder = conveyorBottom.getEncoder();
        conveyorTopEncoder = conveyorTop.getEncoder();

        conveyorBottomPID = conveyorBottom.getPIDController();
        conveyorTopPID = conveyorTop.getPIDController();

        conveyorBottom.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CONVEYOR_BOTTOM);
        conveyorTop.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CONVEYOR_TOP);

        conveyorBottomPID.setP(Constants.CONVEYOR_BOTTOM_PID1_P, 1);
        conveyorBottomPID.setI(Constants.CONVEYOR_BOTTOM_PID1_I, 1);
        conveyorBottomPID.setD(Constants.CONVEYOR_BOTTOM_PID1_D, 1);
        conveyorBottomPID.setIZone(0, 1);
        conveyorBottomPID.setFF(Constants.CONVEYOR_BOTTOM_PID1_F, 1);
        conveyorBottomPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);


        conveyorTopPID.setP(Constants.CONVEYOR_TOP_PID1_P, 1);
        conveyorTopPID.setI(Constants.CONVEYOR_TOP_PID1_I, 1);
        conveyorTopPID.setD(Constants.CONVEYOR_TOP_PID1_D, 1);
        conveyorTopPID.setIZone(0, 1);
        conveyorTopPID.setFF(Constants.CONVEYOR_TOP_PID1_F, 1);
        conveyorTopPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);


        resetEncoders();
        conveyorBottom.burnFlash();
        conveyorTop.burnFlash();
    }


    public static void resetEncoders() {
        conveyorBottomEncoder.setPosition(0);
        conveyorTopEncoder.setPosition(0);
    }

    public static void setConveyorBottomSpeed(double RPM) {

        if (RPM == 0) {
            stopConveyorBottom();

        } else {
            double arbFF = conveyorBottomFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            conveyorBottomPID.setReference(RPM *  Constants.GEAR_RATIO_CONVEYOR_BOTTOM, ControlType.kVelocity, 1, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
        }
    }

    public static void setConveyorTopSpeed(double RPM) {

        if (RPM == 0) {
            stopConveyorTop();

        } else {
            double arbFF = conveyorTopFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            conveyorTopPID.setReference(RPM *  Constants.GEAR_RATIO_CONVEYOR_TOP, ControlType.kVelocity, 1, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
        }
    }

    public static void holdConveyorPosition() {
        conveyorTopPID.setReference(conveyorTopEncoder.getPosition(), ControlType.kPosition, 1);        
    }

    public static void stopConveyorBottom() {
        conveyorBottom.set(0);
    }

    public static void stopConveyorTop() {
        conveyorTop.set(0);
    }

    public static void stopConveyor() {
        stopConveyorBottom();
        stopConveyorTop();
    }
  
    public ConveyorSubsystem() {
        setupMotors();
    }

    @Override
    public void periodic() {

    }

}
