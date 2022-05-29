package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;


public class IntakeSubsystem extends SubsystemBase {
    private static CANSparkMax intakeRollers = new CANSparkMax(Constants.MOTOR_INTAKE_ROLLERS_ONE, MotorType.kBrushless);

    private static RelativeEncoder intakeRollersEncoder;
    private static SparkMaxPIDController intakeRollersPID;
    static SimpleMotorFeedforward intakeRollersFF = new SimpleMotorFeedforward(Constants.INTAKE_ROLLERS_KS, Constants.INTAKE_ROLLERS_KV, Constants.INTAKE_ROLLERS_KA);

    private static Solenoid intakePivotSolenoid;
     
    static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);


    public static double lastTime;
    public static boolean keepSpinning;

    public static void setupMotors() {
        intakeRollers.setInverted(false);
        intakeRollersEncoder = intakeRollers.getEncoder();
        intakeRollersPID = intakeRollers.getPIDController();
        intakeRollers.setSmartCurrentLimit(Constants.CURRENT_LIMIT_INTAKE_ROLLERS);
        intakeRollersPID.setP(Constants.INTAKE_ROLLERS_PID1_P, 1);
        intakeRollersPID.setI(Constants.INTAKE_ROLLERS_PID1_I, 1);
        intakeRollersPID.setD(Constants.INTAKE_ROLLERS_PID1_D, 1);
        intakeRollersPID.setIZone(0, 1);
        intakeRollersPID.setFF(Constants.INTAKE_ROLLERS_PID1_F, 1);
        intakeRollersPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);

        
        resetEncoders();
        intakeRollers.burnFlash();

        intakePivotSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_INTAKE_PIVOT_ONE);
    }

    public static void resetEncoders(){
        intakeRollersEncoder.setPosition(0);
    }


    public static void setIntakeRollersSpeed(double RPM){

        if (RPM == 0) {
            stopIntakeRollers();

        } else {
            double arbFF = intakeRollersFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            intakeRollersPID.setReference(RPM *  Constants.GEAR_RATIO_INTAKE_ROLLERS, ControlType.kVelocity, 1, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);

        }
    }

    public static void intakeExtend() {
        intakePivotSolenoid.set(Constants.INTAKE_EXTEND);
    }

    public static void intakeRetract() {
        intakePivotSolenoid.set(Constants.INTAKE_RETRACT);
    }
    public static void stopIntakeRollers(){
        intakeRollers.set(0);
    }

    public static void resetIntakePivot(){

    }
    public static void intake(){
        intakeExtend();
        setIntakeRollersSpeed(Constants.INTAKE_ROLLER_SPEED);
    }
    public static void stopIntake(){
        intakeRetract();
        stopIntakeRollers();
    }

    public static void outake(){
        intakeExtend();
        setIntakeRollersSpeed(-1*Constants.INTAKE_ROLLER_SPEED);
    }

    public static double getIntakeSpeed(){
        return intakeRollersEncoder.getVelocity() * 1/  Constants.GEAR_RATIO_INTAKE_ROLLERS;
    }

    public static void compressorEnabled() {
        compressor.enableAnalog(Constants.MIN_PRESSURE, Constants.MAX_PRESSURE);
      }
      public static void compressorDisabled() {
        //compressor.disable();
      }
    public static double getPressureValue(){
        return compressor.getPressure();
    }
    

    public IntakeSubsystem() {
        setupMotors();
        compressorEnabled();
    }

    @Override
    public void periodic() {
        
    }

}
